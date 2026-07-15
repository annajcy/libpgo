#!/usr/bin/env python3
"""Measure the direct Eigen/oneMKL single-to-global TBB crossover."""

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


POLICIES = ("MklTbbSingle", "MklTbbGlobal")
TIME_SCALE = {"ns": 1e-9, "us": 1e-6, "ms": 1e-3, "s": 1.0}
CASE_PATTERN = re.compile(
    r"^EigenMklCrossover/(MklTbbSingle|MklTbbGlobal)"
    r"/n_(\d+)(?:/real_time)?$"
)
CONCURRENCY_ENVIRONMENT = "PGO_EIGEN_MKL_CROSSOVER_MAX_CONCURRENCY"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmark", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--max-concurrency", type=int)
    parser.add_argument("--repetitions", type=int, default=7)
    parser.add_argument("--min-time", default="0.05s")
    parser.add_argument("--warmup-time", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=20260715)
    parser.add_argument("--bootstrap-samples", type=int, default=2000)
    parser.add_argument("--checksum-relative-tolerance", type=float, default=1e-10)
    parser.add_argument("--practical-speedup", type=float, default=0.05)
    parser.add_argument("--stable-points", type=int, default=3)
    parser.add_argument("--case-limit", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def checked_output(
    command: list[str], environment: dict[str, str] | None = None
) -> str:
    result = subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
        env=environment,
    )
    if result.returncode != 0:
        raise RuntimeError(f"{' '.join(command)}\n{result.stdout}")
    return result.stdout


def benchmark_environment(max_concurrency: int | None) -> dict[str, str]:
    environment = os.environ.copy()
    environment["MKL_THREADING_LAYER"] = "TBB"
    if max_concurrency is not None:
        environment[CONCURRENCY_ENVIRONMENT] = str(max_concurrency)
    else:
        environment.pop(CONCURRENCY_ENVIRONMENT, None)
    return environment


def verify_linkage(executable: Path) -> dict[str, str]:
    dependencies = checked_output(["ldd", str(executable)])
    undefined_symbols = checked_output(["nm", "-D", "-u", str(executable)])
    dependencies_lower = dependencies.lower()

    required = {
        "MKL core": r"libmkl_core",
        "MKL LP64 interface": r"libmkl_(?:intel|gf)_lp64",
        "MKL TBB threading layer": r"libmkl_tbb_thread",
        "oneTBB": r"libtbb",
    }
    for label, pattern in required.items():
        if not re.search(pattern, dependencies_lower):
            raise RuntimeError(f"Benchmark is missing {label} in ldd output.")

    forbidden = ("libiomp5", "libgomp", "libomp.so")
    if any(library in dependencies_lower for library in forbidden):
        raise RuntimeError(
            "Benchmark links an OpenMP runtime instead of a pure MKL-TBB stack."
        )

    dgemm = re.compile(r"(?:^|\s)_?(?:cblas_)?dgemm_?(?:@\S+)?(?:\s|$)", re.I | re.M)
    if not dgemm.search(undefined_symbols):
        raise RuntimeError("Benchmark does not expose a dynamic DGEMM reference.")

    return {
        "dependencies": dependencies,
        "undefined_symbols": undefined_symbols,
    }


def discover_cases(
    executable: Path, environment: dict[str, str]
) -> dict[int, dict[str, str]]:
    output = checked_output([str(executable), "--benchmark_list_tests"], environment)
    discovered: dict[int, dict[str, str]] = {}
    for line in output.splitlines():
        name = line.strip()
        match = CASE_PATTERN.fullmatch(name)
        if not match:
            continue
        policy, matrix_n = match.groups()
        discovered.setdefault(int(matrix_n), {})[policy] = name

    return {
        matrix_n: cases
        for matrix_n, cases in sorted(discovered.items())
        if set(cases) == set(POLICIES)
    }


def exact_filter(name: str) -> str:
    return f"^{re.escape(name)}$"


def run_one(
    executable: Path,
    name: str,
    output: Path,
    min_time: str,
    warmup_time: float,
    environment: dict[str, str],
) -> dict[str, Any]:
    checked_output(
        [
            str(executable),
            f"--benchmark_filter={exact_filter(name)}",
            f"--benchmark_min_time={min_time}",
            f"--benchmark_min_warmup_time={warmup_time:g}",
            "--benchmark_repetitions=1",
            f"--benchmark_out={output}",
            "--benchmark_out_format=json",
        ],
        environment,
    )
    payload = json.loads(output.read_text())
    rows = [
        row
        for row in payload.get("benchmarks", [])
        if row.get("run_type") != "aggregate"
    ]
    if len(rows) != 1:
        raise RuntimeError(f"Expected one result for {name}, found {len(rows)}.")

    row = rows[0]
    scale = TIME_SCALE.get(row.get("time_unit"))
    if scale is None:
        raise RuntimeError(f"Unsupported time unit for {name}: {row.get('time_unit')}")
    row["wall_seconds"] = float(row["real_time"]) * scale
    return row


def integer_counter(row: dict[str, Any], name: str) -> int:
    value = float(row.get(name, math.nan))
    if not math.isfinite(value):
        raise RuntimeError(f"Missing finite counter {name} in {row.get('name')}.")
    return round(value)


def validate_block(
    matrix_n: int,
    measurements: dict[str, dict[str, Any]],
    requested_concurrency: int | None,
    checksum_relative_tolerance: float,
) -> int:
    checksums = [
        float(measurements[policy].get("checksum", math.nan)) for policy in POLICIES
    ]
    if not all(math.isfinite(value) for value in checksums):
        raise RuntimeError(f"Missing finite checksum for n={matrix_n}.")
    tolerance = max(
        1e-12,
        checksum_relative_tolerance * max(abs(value) for value in checksums),
    )
    if max(checksums) - min(checksums) > tolerance:
        raise RuntimeError(f"Checksum mismatch for n={matrix_n}.")

    for policy, row in measurements.items():
        if integer_counter(row, "matrix_n") != matrix_n:
            raise RuntimeError(f"{policy} reported the wrong matrix size.")

    effective_values = {
        integer_counter(row, "effective_concurrency") for row in measurements.values()
    }
    if len(effective_values) != 1:
        raise RuntimeError(
            f"Policies used different effective concurrency for n={matrix_n}."
        )
    effective_concurrency = effective_values.pop()

    if requested_concurrency is not None:
        for policy, row in measurements.items():
            if integer_counter(row, "requested_concurrency") != requested_concurrency:
                raise RuntimeError(
                    f"{policy} did not receive concurrency={requested_concurrency}."
                )
        if effective_concurrency != requested_concurrency:
            raise RuntimeError(
                f"Requested concurrency={requested_concurrency}, "
                f"but oneTBB established {effective_concurrency}."
            )

    if integer_counter(measurements["MklTbbSingle"], "arena_concurrency") != 1:
        raise RuntimeError("MklTbbSingle did not use a one-participant arena.")
    if (
        integer_counter(measurements["MklTbbGlobal"], "arena_concurrency")
        != effective_concurrency
    ):
        raise RuntimeError("MklTbbGlobal did not use the global-aligned arena.")

    return effective_concurrency


def percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = probability * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def bootstrap_median_ci(
    values: list[float], randomizer: random.Random, samples: int
) -> list[float]:
    medians = [
        statistics.median(values[randomizer.randrange(len(values))] for _ in values)
        for _ in range(samples)
    ]
    return [percentile(medians, 0.025), percentile(medians, 0.975)]


def summarize(
    records: list[dict[str, Any]],
    seed: int,
    bootstrap_samples: int,
    practical_speedup: float,
) -> list[dict[str, Any]]:
    grouped: dict[int, list[dict[str, Any]]] = {}
    for record in records:
        grouped.setdefault(record["matrix_n"], []).append(record)

    randomizer = random.Random(seed ^ 0xC20550)
    summary: list[dict[str, Any]] = []
    for matrix_n, blocks in sorted(grouped.items()):
        ratios = [float(block["global_over_single"]) for block in blocks]
        confidence_interval = bootstrap_median_ci(ratios, randomizer, bootstrap_samples)
        median_ratio = statistics.median(ratios)
        if confidence_interval[1] < 1.0:
            evidence = "global_faster"
        elif confidence_interval[0] > 1.0:
            evidence = "single_faster"
        else:
            evidence = "indistinguishable"

        summary.append(
            {
                "matrix_n": matrix_n,
                "repetitions": len(blocks),
                "effective_concurrency": blocks[0]["effective_concurrency"],
                "median_wall_seconds": {
                    policy: statistics.median(
                        float(block["measurements"][policy]["wall_seconds"])
                        for block in blocks
                    )
                    for policy in POLICIES
                },
                "median_global_over_single": median_ratio,
                "bootstrap_95pct_ci": confidence_interval,
                "evidence": evidence,
                "practical_global_win": (
                    median_ratio <= 1.0 - practical_speedup
                    and confidence_interval[1] < 1.0
                ),
            }
        )
    return summary


def stable_crossover(summary: list[dict[str, Any]], stable_points: int) -> int | None:
    run: list[int] = []
    for row in summary:
        matrix_n = int(row["matrix_n"])
        if row["practical_global_win"] and (not run or matrix_n == run[-1] + 4):
            run.append(matrix_n)
        elif row["practical_global_win"]:
            run = [matrix_n]
        else:
            run = []
        if len(run) == stable_points:
            return run[0]
    return None


def main() -> int:
    args = parse_args()
    if args.max_concurrency is not None and args.max_concurrency <= 0:
        raise SystemExit("--max-concurrency must be positive.")
    if args.repetitions < 1 or args.bootstrap_samples < 1:
        raise SystemExit("--repetitions and --bootstrap-samples must be positive.")
    if args.warmup_time < 0 or not 0.0 <= args.practical_speedup < 1.0:
        raise SystemExit("Warm-up and practical speedup values are invalid.")
    if args.stable_points < 1 or args.case_limit < 0:
        raise SystemExit(
            "--stable-points must be positive and --case-limit non-negative."
        )

    executable = args.benchmark.resolve()
    if not executable.exists():
        raise SystemExit(f"Benchmark executable does not exist: {executable}")

    environment = benchmark_environment(args.max_concurrency)
    linkage = verify_linkage(executable)
    cases = discover_cases(executable, environment)
    if args.case_limit:
        cases = dict(list(cases.items())[: args.case_limit])
    if not cases:
        raise SystemExit("No complete MKL crossover cases were found.")

    print(f"Verified MKL-TBB linkage; matched {len(cases)} matrix size(s).")
    print("matrix sizes:", ", ".join(str(matrix_n) for matrix_n in cases))
    if args.dry_run:
        return 0

    schedule = [
        (repetition, matrix_n)
        for repetition in range(1, args.repetitions + 1)
        for matrix_n in cases
    ]
    randomizer = random.Random(args.seed)
    randomizer.shuffle(schedule)
    records: list[dict[str, Any]] = []

    with tempfile.TemporaryDirectory(prefix="pgo-eigen-mkl-crossover-") as temp_dir:
        temporary = Path(temp_dir)
        for block_index, (repetition, matrix_n) in enumerate(schedule, start=1):
            policies = list(POLICIES)
            randomizer.shuffle(policies)
            print(
                f"[{block_index}/{len(schedule)}] repetition={repetition} "
                f"n={matrix_n} order={','.join(policies)}",
                flush=True,
            )
            measurements: dict[str, dict[str, Any]] = {}
            for policy in policies:
                measurements[policy] = run_one(
                    executable,
                    cases[matrix_n][policy],
                    temporary / f"{block_index}-{policy}.json",
                    args.min_time,
                    args.warmup_time,
                    environment,
                )

            effective_concurrency = validate_block(
                matrix_n,
                measurements,
                args.max_concurrency,
                args.checksum_relative_tolerance,
            )
            single_time = float(measurements["MklTbbSingle"]["wall_seconds"])
            global_time = float(measurements["MklTbbGlobal"]["wall_seconds"])
            records.append(
                {
                    "repetition": repetition,
                    "matrix_n": matrix_n,
                    "order": policies,
                    "effective_concurrency": effective_concurrency,
                    "measurements": measurements,
                    "global_over_single": global_time / single_time,
                }
            )

    summary = summarize(
        records,
        args.seed,
        args.bootstrap_samples,
        args.practical_speedup,
    )
    crossover = stable_crossover(summary, args.stable_points)
    payload = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "benchmark": str(executable),
        "max_concurrency": args.max_concurrency,
        "seed": args.seed,
        "repetitions": args.repetitions,
        "min_time": args.min_time,
        "warmup_time": args.warmup_time,
        "bootstrap_samples": args.bootstrap_samples,
        "decision_rule": {
            "practical_speedup": args.practical_speedup,
            "stable_adjacent_points": args.stable_points,
            "requires_bootstrap_ci_below_one": True,
        },
        "stable_crossover_n": crossover,
        "environment": {
            "MKL_THREADING_LAYER": environment["MKL_THREADING_LAYER"],
            CONCURRENCY_ENVIRONMENT: environment.get(CONCURRENCY_ENVIRONMENT),
        },
        "linkage": linkage,
        "records": records,
        "summary": summary,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2) + "\n")

    print("\n   n   single_us   global_us  global/single          95% CI  evidence")
    for row in summary:
        times = row["median_wall_seconds"]
        confidence_interval = row["bootstrap_95pct_ci"]
        print(
            f"{row['matrix_n']:4d}  "
            f"{times['MklTbbSingle'] * 1e6:10.3f}  "
            f"{times['MklTbbGlobal'] * 1e6:10.3f}  "
            f"{row['median_global_over_single']:13.4f}  "
            f"[{confidence_interval[0]:.4f}, {confidence_interval[1]:.4f}]  "
            f"{row['evidence']}"
        )
    if crossover is None:
        print("\nNo stable crossover satisfied the configured decision rule.")
    else:
        print(f"\nStable global-parallel crossover starts at n={crossover}.")
    print(f"Wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
