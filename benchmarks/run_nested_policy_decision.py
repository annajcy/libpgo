#!/usr/bin/env python3
"""Run Suppress/Inherit cases as randomized fresh-process pairs and summarize JSON."""

from __future__ import annotations

import argparse
import json
import math
import random
import re
import statistics
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EQUIVALENCE_FRACTION = 0.05
IMPORTANT_REGRESSION_FRACTION = 0.10
POLICIES = ("Suppress", "Inherit")
TIME_SCALE = {"ns": 1e-9, "us": 1e-6, "ms": 1e-3, "s": 1.0}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmark", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--filter",
        default=r"^(PolicyDecision|NestedParallelDgemmAccelerate|NestedParallelDgemm)/",
        help="Regex selecting discovered benchmark names.",
    )
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--min-time", default="0.03s")
    parser.add_argument("--seed", type=int, default=20260712)
    parser.add_argument("--case-limit", type=int, default=0, help="Limit paired configurations.")
    parser.add_argument("--bootstrap-samples", type=int, default=2000)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def run(command: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def discover_cases(benchmark: Path, pattern: str) -> dict[str, dict[str, str]]:
    result = run([str(benchmark), "--benchmark_list_tests"])
    if result.returncode != 0:
        raise RuntimeError(result.stdout)
    selected = re.compile(pattern)
    pairs: dict[str, dict[str, str]] = {}
    for line in result.stdout.splitlines():
        name = line.strip()
        if not selected.search(name):
            continue
        components = name.split("/")
        policy_positions = [i for i, component in enumerate(components) if component in POLICIES]
        if len(policy_positions) != 1:
            continue
        position = policy_positions[0]
        policy = components[position]
        components[position] = "{policy}"
        key = "/".join(components)
        pairs.setdefault(key, {})[policy] = name
    return {key: value for key, value in sorted(pairs.items()) if set(value) == set(POLICIES)}


def exact_filter(name: str) -> str:
    return f"^{re.escape(name)}$"


def run_one(benchmark: Path, name: str, min_time: str, output: Path) -> dict[str, Any]:
    command = [
        str(benchmark),
        f"--benchmark_filter={exact_filter(name)}",
        f"--benchmark_min_time={min_time}",
        "--benchmark_repetitions=1",
        f"--benchmark_out={output}",
        "--benchmark_out_format=json",
    ]
    result = run(command)
    if result.returncode != 0:
        raise RuntimeError(f"{' '.join(command)}\n{result.stdout}")
    payload = json.loads(output.read_text())
    rows = [row for row in payload.get("benchmarks", []) if row.get("run_type") != "aggregate"]
    if len(rows) != 1:
        raise RuntimeError(f"Expected one result for {name}, found {len(rows)}")
    row = rows[0]
    unit = row["time_unit"]
    if unit not in TIME_SCALE:
        raise RuntimeError(f"Unsupported time unit: {unit}")
    row["wall_seconds"] = row["real_time"] * TIME_SCALE[unit]
    row["cpu_seconds"] = row["cpu_time"] * TIME_SCALE[unit]
    return {"context": payload.get("context", {}), "measurement": row}


def percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = probability * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def bootstrap_median_ci(values: list[float], rng: random.Random, samples: int) -> list[float]:
    bootstrapped = []
    for _ in range(samples):
        sample = [values[rng.randrange(len(values))] for _ in values]
        bootstrapped.append(statistics.median(sample))
    return [percentile(bootstrapped, 0.025), percentile(bootstrapped, 0.975)]


def geometric_mean(values: list[float]) -> float:
    return math.exp(sum(math.log(value) for value in values) / len(values))


def family_for_key(key: str) -> str:
    components = key.split("/")
    if components[0] == "PolicyDecision":
        return components[1]
    if components[0] in {"NestedParallelDgemmAccelerate", "NestedParallelDgemm"}:
        return "Dgemm"
    return components[0]


def classify_ratio(ratio: float) -> str:
    if ratio < 1.0 - EQUIVALENCE_FRACTION:
        return "inherit_faster"
    if ratio > 1.0 + EQUIVALENCE_FRACTION:
        return "suppress_faster"
    return "equivalent"


def validate_checksum(suppress: dict[str, Any], inherit: dict[str, Any], key: str) -> None:
    lhs = float(suppress["measurement"].get("checksum", math.nan))
    rhs = float(inherit["measurement"].get("checksum", math.nan))
    if not math.isfinite(lhs) or not math.isfinite(rhs):
        raise RuntimeError(f"Missing finite checksum for {key}")
    tolerance = max(1e-10, 1e-10 * max(abs(lhs), abs(rhs)))
    if abs(lhs - rhs) > tolerance:
        raise RuntimeError(f"Checksum mismatch for {key}: Suppress={lhs}, Inherit={rhs}")


def summarize(records: list[dict[str, Any]], seed: int, bootstrap_samples: int) -> dict[str, Any]:
    by_key: dict[str, list[dict[str, Any]]] = {}
    for record in records:
        by_key.setdefault(record["case_key"], []).append(record)

    rng = random.Random(seed ^ 0xB0057)
    cases = []
    for key, case_records in sorted(by_key.items()):
        ratios = [record["inherit_over_suppress"] for record in case_records]
        median_ratio = statistics.median(ratios)
        suppress_threads = [record["Suppress"]["measurement"].get("peak_threads", 0) for record in case_records]
        inherit_threads = [record["Inherit"]["measurement"].get("peak_threads", 0) for record in case_records]
        suppress_participants = [
            record["Suppress"]["measurement"].get("peak_participants", 0) for record in case_records
        ]
        inherit_participants = [
            record["Inherit"]["measurement"].get("peak_participants", 0) for record in case_records
        ]
        cases.append(
            {
                "case_key": key,
                "family": family_for_key(key),
                "repetitions": len(ratios),
                "median_inherit_over_suppress": median_ratio,
                "bootstrap_95pct_ci": bootstrap_median_ci(ratios, rng, bootstrap_samples),
                "classification": classify_ratio(median_ratio),
                "important_regression": median_ratio > 1.0 + IMPORTANT_REGRESSION_FRACTION,
                "median_peak_threads": {
                    "Suppress": statistics.median(suppress_threads),
                    "Inherit": statistics.median(inherit_threads),
                },
                "median_peak_participants": {
                    "Suppress": statistics.median(suppress_participants),
                    "Inherit": statistics.median(inherit_participants),
                },
            }
        )

    families: dict[str, list[float]] = {}
    for case in cases:
        families.setdefault(case["family"], []).append(case["median_inherit_over_suppress"])
    family_summary = {
        family: {
            "case_count": len(ratios),
            "geomean_inherit_over_suppress": geometric_mean(ratios),
            "classification": classify_ratio(geometric_mean(ratios)),
        }
        for family, ratios in sorted(families.items())
    }

    representative = [
        case["median_inherit_over_suppress"]
        for case in cases
        if case["family"] in {"FemElementBatch", "ContactPairBatch"}
    ]
    representative_geomean = geometric_mean(representative) if representative else None
    important_regressions = [case["case_key"] for case in cases if case["important_regression"]]
    inherit_clears_speed_bar = representative_geomean is not None and representative_geomean <= 0.95
    if important_regressions:
        recommendation = "retain_suppress_default_important_regressions"
    elif not inherit_clears_speed_bar:
        recommendation = "retain_suppress_default_no_representative_5pct_gain"
    else:
        recommendation = "do_not_change_yet_cross_platform_evidence_required"

    return {
        "cases": cases,
        "families": family_summary,
        "representative_geomean_inherit_over_suppress": representative_geomean,
        "important_regressions": important_regressions,
        "recommendation": recommendation,
        "recommendation_scope": "current platform only; Linux MKL-TBB is required before changing the default",
    }


def main() -> int:
    args = parse_args()
    if args.repetitions < 1 or args.bootstrap_samples < 1:
        print("--repetitions and --bootstrap-samples must be positive", file=sys.stderr)
        return 2
    benchmark = args.benchmark.resolve()
    if not benchmark.exists():
        print(f"Benchmark executable does not exist: {benchmark}", file=sys.stderr)
        return 2

    pairs = discover_cases(benchmark, args.filter)
    if args.case_limit > 0:
        pairs = dict(list(pairs.items())[: args.case_limit])
    if not pairs:
        print("No complete Suppress/Inherit pairs matched", file=sys.stderr)
        return 1
    print(f"Matched {len(pairs)} paired configuration(s); {args.repetitions} fresh pairs each.")
    for key in pairs:
        print(key)
    if args.dry_run:
        return 0

    rng = random.Random(args.seed)
    schedule = [(repetition, key) for repetition in range(1, args.repetitions + 1) for key in pairs]
    rng.shuffle(schedule)
    records: list[dict[str, Any]] = []
    benchmark_context: dict[str, Any] = {}
    with tempfile.TemporaryDirectory(prefix="pgo-policy-decision-") as temp_dir:
        temp = Path(temp_dir)
        for run_index, (repetition, key) in enumerate(schedule, start=1):
            policy_order = list(POLICIES)
            rng.shuffle(policy_order)
            pair_result: dict[str, Any] = {}
            print(
                f"[{run_index}/{len(schedule)}] repetition={repetition} "
                f"order={','.join(policy_order)} {key}",
                flush=True,
            )
            for policy in policy_order:
                output = temp / f"run-{run_index:04d}-{policy}.json"
                result = run_one(benchmark, pairs[key][policy], args.min_time, output)
                benchmark_context = result["context"]
                pair_result[policy] = result
            validate_checksum(pair_result["Suppress"], pair_result["Inherit"], key)
            suppress_time = pair_result["Suppress"]["measurement"]["wall_seconds"]
            inherit_time = pair_result["Inherit"]["measurement"]["wall_seconds"]
            records.append(
                {
                    "case_key": key,
                    "repetition": repetition,
                    "policy_order": policy_order,
                    "inherit_over_suppress": inherit_time / suppress_time,
                    "Suppress": pair_result["Suppress"],
                    "Inherit": pair_result["Inherit"],
                }
            )

    payload = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "benchmark": str(benchmark),
        "benchmark_context": benchmark_context,
        "settings": {
            "filter": args.filter,
            "repetitions": args.repetitions,
            "min_time": args.min_time,
            "seed": args.seed,
            "fresh_process_per_policy": True,
            "paired_randomized_order": True,
        },
        "decision_rules_fixed_before_run": {
            "equivalence_interval_fraction": EQUIVALENCE_FRACTION,
            "important_regression_fraction": IMPORTANT_REGRESSION_FRACTION,
            "inherit_representative_geomean_speedup_required_fraction": 0.05,
            "cross_platform_direction_required": True,
        },
        "summary": summarize(records, args.seed, args.bootstrap_samples),
        "raw_pairs": records,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    print(f"Wrote {args.out}")
    print(f"Recommendation: {payload['summary']['recommendation']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
