#!/usr/bin/env python3
"""Run the TBB-only V2 MKL Bounded1/Multi benchmark in randomized process blocks."""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import re
import statistics
import subprocess
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


POLICIES = ("Bounded1", "Multi")
BASELINE = "Bounded1"
TIME_SCALE = {"ns": 1e-9, "us": 1e-6, "ms": 1e-3, "s": 1.0}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmark", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--filter",
        default=r"^(MklTbbBoundDecision|MklTbbBoundRealFemDecision)/",
        help="Regex selecting TBB-only V2 benchmark names.",
    )
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--min-time", default="0.10s")
    parser.add_argument("--warmup-time", type=float, default=0.20)
    parser.add_argument("--seed", type=int, default=20260713)
    parser.add_argument("--bootstrap-samples", type=int, default=2000)
    parser.add_argument(
        "--checksum-relative-tolerance",
        type=float,
        default=2e-5,
        help="Tolerance for threaded MKL and atomic assembly reductions.",
    )
    parser.add_argument("--case-limit", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def run(command: list[str], environment: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
        env=environment,
    )


def discover_cases(benchmark: Path, pattern: str) -> dict[str, dict[str, str]]:
    result = run([str(benchmark), "--benchmark_list_tests"])
    if result.returncode != 0:
        raise RuntimeError(result.stdout)

    selected = re.compile(pattern)
    cases: dict[str, dict[str, str]] = {}
    for raw_name in result.stdout.splitlines():
        name = raw_name.strip()
        if not selected.search(name):
            continue
        components = name.split("/")
        positions = [index for index, component in enumerate(components) if component in POLICIES]
        if len(positions) != 1:
            continue
        position = positions[0]
        policy = components[position]
        components[position] = "{policy}"
        cases.setdefault("/".join(components), {})[policy] = name

    return {
        key: values
        for key, values in sorted(cases.items())
        if set(values) == set(POLICIES)
    }


def runtime_concurrency_for_case(name: str) -> int | None:
    match = re.search(r"(?:^|/)runtime_workers_(\d+)(?:/|$)", name)
    return int(match.group(1)) if match else None


def exact_filter(name: str) -> str:
    return f"^{re.escape(name)}$"


def run_one(
    benchmark: Path, name: str, min_time: str, warmup_time: float, output: Path
) -> dict[str, Any]:
    environment = os.environ.copy()
    runtime_concurrency = runtime_concurrency_for_case(name)
    if runtime_concurrency is not None:
        environment["PGO_BENCHMARK_RUNTIME_CONCURRENCY"] = str(runtime_concurrency)

    command = [
        str(benchmark),
        f"--benchmark_filter={exact_filter(name)}",
        f"--benchmark_min_time={min_time}",
        # Google Benchmark parses min_time as a duration string, but the
        # min_warmup_time flag is a plain floating-point number of seconds.
        f"--benchmark_min_warmup_time={warmup_time:g}",
        "--benchmark_repetitions=1",
        f"--benchmark_out={output}",
        "--benchmark_out_format=json",
    ]
    result = run(command, environment)
    if result.returncode != 0:
        raise RuntimeError(f"{' '.join(command)}\n{result.stdout}")

    payload = json.loads(output.read_text())
    rows = [row for row in payload.get("benchmarks", []) if row.get("run_type") != "aggregate"]
    if len(rows) != 1:
        raise RuntimeError(f"Expected one result for {name}, found {len(rows)}")
    row = rows[0]
    scale = TIME_SCALE.get(row["time_unit"])
    if scale is None:
        raise RuntimeError(f"Unsupported time unit {row['time_unit']} for {name}")
    row["wall_seconds"] = row["real_time"] * scale
    row["cpu_seconds"] = row["cpu_time"] * scale
    return {"context": payload.get("context", {}), "measurement": row}


def validate_checksums(
    block: dict[str, dict[str, Any]], key: str, relative_tolerance: float
) -> None:
    baseline = float(block[BASELINE]["measurement"].get("checksum", math.nan))
    if not math.isfinite(baseline):
        raise RuntimeError(f"Missing finite {BASELINE} checksum for {key}")
    for policy, result in block.items():
        checksum = float(result["measurement"].get("checksum", math.nan))
        tolerance = max(1e-10, relative_tolerance * max(abs(checksum), abs(baseline)))
        if not math.isfinite(checksum) or abs(checksum - baseline) > tolerance:
            raise RuntimeError(
                f"Checksum mismatch for {key}: {policy}={checksum}, {BASELINE}={baseline}"
            )


def as_positive_counter(row: dict[str, Any], key: str, case_key: str) -> int:
    value = int(row.get(key, 0))
    if value <= 0:
        raise RuntimeError(f"Missing positive {key} for {case_key}: {value}")
    return value


def validate_policy_telemetry(block: dict[str, dict[str, Any]], key: str) -> None:
    bounded = block["Bounded1"]["measurement"]
    multi = block["Multi"]["measurement"]
    runtime_c = as_positive_counter(bounded, "runtime_concurrency", key)
    executor_e = as_positive_counter(
        multi,
        "arena_concurrency" if "arena_concurrency" in multi else "executor_concurrency",
        key,
    )
    expected = {"Bounded1": 1, "Multi": executor_e}
    for policy, row in (("Bounded1", bounded), ("Multi", multi)):
        observed = as_positive_counter(row, "worker_arena_concurrency", key)
        if observed != expected[policy]:
            raise RuntimeError(
                f"Worker TBB arena limit mismatch for {key}/{policy}: "
                f"expected {expected[policy]}, got {observed}"
            )
        arenaThreads = as_positive_counter(row, "observed_peak_arena_threads", key)
        if arenaThreads > expected[policy]:
            raise RuntimeError(
                f"TBB arena-thread bound exceeded for {key}/{policy}: "
                f"limit {expected[policy]}, observed {arenaThreads}"
            )
        if as_positive_counter(row, "runtime_concurrency", key) != runtime_c:
            raise RuntimeError(f"Runtime C changed within policy block for {key}")


def percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = probability * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def bootstrap_median_ci(values: list[float], rng: random.Random, samples: int) -> list[float]:
    medians = [
        statistics.median(values[rng.randrange(len(values))] for _ in values)
        for _ in range(samples)
    ]
    return [percentile(medians, 0.025), percentile(medians, 0.975)]


def geometric_mean(values: list[float]) -> float | None:
    return math.exp(sum(math.log(value) for value in values) / len(values)) if values else None


def summarize(records: list[dict[str, Any]], seed: int, bootstrap_samples: int) -> dict[str, Any]:
    rng = random.Random(seed ^ 0xC4A)
    by_case: dict[str, list[dict[str, Any]]] = {}
    for record in records:
        by_case.setdefault(record["case_key"], []).append(record)

    cases = []
    for key, values in sorted(by_case.items()):
        cases.append(
            {
                "case_key": key,
                "repetitions": len(values),
                "median_ratio_over_bounded1": {
                    policy: statistics.median(record["ratios_over_bounded1"][policy] for record in values)
                    for policy in POLICIES
                    if policy != BASELINE
                },
                "bootstrap_95pct_ci": {
                    policy: bootstrap_median_ci(
                        [record["ratios_over_bounded1"][policy] for record in values],
                        rng,
                        bootstrap_samples,
                    )
                    for policy in POLICIES
                    if policy != BASELINE
                },
            }
        )

    real_fem = [case for case in cases if case["case_key"].startswith("MklTbbBoundRealFemDecision/")]
    return {
        "cases": cases,
        "real_fem_geomean_ratio_over_bounded1": {
            policy: geometric_mean(
                [case["median_ratio_over_bounded1"][policy] for case in real_fem]
            )
            for policy in POLICIES
            if policy != BASELINE
        },
    }


def main() -> int:
    args = parse_args()
    if args.repetitions < 1 or args.bootstrap_samples < 1 or args.warmup_time < 0:
        raise SystemExit("repetitions, bootstrap samples, and warm-up time must be valid")

    benchmark = args.benchmark.resolve()
    if not benchmark.exists():
        raise SystemExit(f"Benchmark executable does not exist: {benchmark}")
    cases = discover_cases(benchmark, args.filter)
    if args.case_limit:
        cases = dict(list(cases.items())[: args.case_limit])
    if not cases:
        raise SystemExit("No complete Bounded1/Multi configurations matched")

    print(f"Matched {len(cases)} policy configuration(s); {args.repetitions} blocks each.")
    for key in cases:
        print(key)
    if args.dry_run:
        return 0

    schedule = [(repetition, key) for repetition in range(1, args.repetitions + 1) for key in cases]
    randomizer = random.Random(args.seed)
    randomizer.shuffle(schedule)
    records: list[dict[str, Any]] = []
    context: dict[str, Any] = {}
    with tempfile.TemporaryDirectory(prefix="pgo-mkl-tbb-bound-") as temporary_directory:
        temporary = Path(temporary_directory)
        for index, (repetition, key) in enumerate(schedule, start=1):
            policy_order = list(POLICIES)
            randomizer.shuffle(policy_order)
            print(
                f"[{index}/{len(schedule)}] repetition={repetition} "
                f"order={','.join(policy_order)} {key}",
                flush=True,
            )
            block: dict[str, dict[str, Any]] = {}
            for policy in policy_order:
                result = run_one(
                    benchmark,
                    cases[key][policy],
                    args.min_time,
                    args.warmup_time,
                    temporary / f"run-{index:04d}-{policy}.json",
                )
                context = result["context"]
                block[policy] = result
            validate_checksums(block, key, args.checksum_relative_tolerance)
            validate_policy_telemetry(block, key)
            baseline_time = block[BASELINE]["measurement"]["wall_seconds"]
            records.append(
                {
                    "case_key": key,
                    "repetition": repetition,
                    "policy_order": policy_order,
                    "ratios_over_bounded1": {
                        policy: block[policy]["measurement"]["wall_seconds"] / baseline_time
                        for policy in POLICIES
                        if policy != BASELINE
                    },
                    "policies": block,
                }
            )

    payload = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "benchmark": str(benchmark),
        "benchmark_context": context,
        "settings": {
            "filter": args.filter,
            "repetitions": args.repetitions,
            "min_time": args.min_time,
            "warmup_time_seconds": args.warmup_time,
            "seed": args.seed,
            "checksum_relative_tolerance": args.checksum_relative_tolerance,
            "policies": POLICIES,
            "fresh_process_per_policy": True,
            "randomized_policy_order_within_block": True,
        },
        "summary": summarize(records, args.seed, args.bootstrap_samples),
        "raw_blocks": records,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    print(f"Wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
