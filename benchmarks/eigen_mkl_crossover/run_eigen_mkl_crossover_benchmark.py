#!/usr/bin/env python3
"""Measure the Eigen/oneMKL local-budget-1 to budget-C crossover."""

from __future__ import annotations

import argparse
import math
import os
import random
import re
import statistics
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


BENCHMARKS_ROOT = Path(__file__).resolve().parents[1]
if str(BENCHMARKS_ROOT) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS_ROOT))

from benchmark_support.harness import (  # noqa: E402
    add_benchmark_harness_arguments,
    prepare_benchmark_host,
)
from benchmark_support.artifact import JsonArtifact, runner_manifest  # noqa: E402
from benchmark_support.schedule import balanced_order, order_configuration  # noqa: E402
from benchmark_support.warmup import (  # noqa: E402
    add_workload_warmup_arguments,
    validate_workload_warmup_arguments,
    workload_warmup_configuration,
)
from benchmark_support.google_benchmark import (  # noqa: E402
    integer_counter,
    list_cases,
    run_case,
)
from benchmark_support.mkl import (  # noqa: E402
    mkl_tbb_environment,
    verify_mkl_tbb_benchmark_linkage,
)
from benchmark_support.statistics import bootstrap_median_ci  # noqa: E402


POLICIES = ("ExecutorLocal1", "ExecutorMKLC")
CASE_PATTERN = re.compile(
    r"^EigenMklCrossover/(ExecutorLocal1|ExecutorMKLC)"
    r"/n_(\d+)(?:/real_time)?$"
)
CONCURRENCY_ENVIRONMENT = "PGO_EIGEN_MKL_CROSSOVER_MAX_CONCURRENCY"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmark", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--max-concurrency", type=int)
    parser.add_argument("--repetitions", type=int, default=8)
    parser.add_argument("--min-time", default="0.05s")
    parser.add_argument("--seed", type=int, default=20260715)
    parser.add_argument("--bootstrap-samples", type=int, default=2000)
    parser.add_argument("--checksum-relative-tolerance", type=float, default=1e-10)
    parser.add_argument("--practical-speedup", type=float, default=0.05)
    parser.add_argument("--stable-points", type=int, default=3)
    parser.add_argument("--case-limit", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true")
    add_workload_warmup_arguments(
        parser, default_seconds=1.0, default_min_operations=10
    )
    add_benchmark_harness_arguments(parser)
    return parser.parse_args()


def benchmark_environment(max_concurrency: int | None) -> dict[str, str]:
    overrides: dict[str, str] = {}
    if max_concurrency is not None:
        overrides[CONCURRENCY_ENVIRONMENT] = str(max_concurrency)
    else:
        overrides[CONCURRENCY_ENVIRONMENT] = os.environ.get(
            CONCURRENCY_ENVIRONMENT, ""
        )
    environment = mkl_tbb_environment(overrides)
    if max_concurrency is None:
        environment.pop(CONCURRENCY_ENVIRONMENT, None)
    return environment


def discover_cases(
    executable: Path, environment: dict[str, str]
) -> dict[int, dict[str, str]]:
    discovered: dict[int, dict[str, str]] = {}
    for name in list_cases(executable, environment):
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
        integer_counter(row, "effective_global_concurrency")
        for row in measurements.values()
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

    for policy, expected_budget in (
        ("ExecutorLocal1", 1),
        ("ExecutorMKLC", effective_concurrency),
    ):
        row = measurements[policy]
        if (
            integer_counter(row, "configured_arena_concurrency")
            != effective_concurrency
        ):
            raise RuntimeError(f"{policy} did not use the configured arena.")
        if integer_counter(row, "observed_arena_concurrency") != effective_concurrency:
            raise RuntimeError(f"{policy} observed the wrong arena concurrency.")
        if integer_counter(row, "configured_mkl_local_budget") != expected_budget:
            raise RuntimeError(f"{policy} used the wrong MKL local budget.")

    return effective_concurrency


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
        ratios = [float(block["mkl_c_over_local1"]) for block in blocks]
        confidence_interval = bootstrap_median_ci(ratios, randomizer, bootstrap_samples)
        median_ratio = statistics.median(ratios)
        if confidence_interval[1] < 1.0:
            evidence = "mkl_c_faster"
        elif confidence_interval[0] > 1.0:
            evidence = "local1_faster"
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
                "median_mkl_c_over_local1": median_ratio,
                "bootstrap_95pct_ci": confidence_interval,
                "evidence": evidence,
                "practical_mkl_c_win": (
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
        if row["practical_mkl_c_win"] and (not run or matrix_n == run[-1] + 4):
            run.append(matrix_n)
        elif row["practical_mkl_c_win"]:
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
    if not 0.0 <= args.practical_speedup < 1.0:
        raise SystemExit("Warm-up and practical speedup values are invalid.")
    try:
        validate_workload_warmup_arguments(args)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    if args.stable_points < 1 or args.case_limit < 0:
        raise SystemExit(
            "--stable-points must be positive and --case-limit non-negative."
        )

    executable = args.benchmark.resolve()
    if not executable.exists():
        raise SystemExit(f"Benchmark executable does not exist: {executable}")

    environment = benchmark_environment(args.max_concurrency)
    linkage = verify_mkl_tbb_benchmark_linkage(executable, environment)
    cases = discover_cases(executable, environment)
    if args.case_limit:
        cases = dict(list(cases.items())[: args.case_limit])
    if not cases:
        raise SystemExit("No complete MKL crossover cases were found.")

    print(f"Verified MKL-TBB linkage; matched {len(cases)} matrix size(s).")
    print("matrix sizes:", ", ".join(str(matrix_n) for matrix_n in cases))
    order = order_configuration(
        args.repetitions,
        [("policies", POLICIES)],
        allow_incomplete=args.allow_incomplete_order_cycle,
    )
    host_environment = prepare_benchmark_host(
        args, workers=args.max_concurrency or 8, dry_run=args.dry_run
    )
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
    payload = {
        "runner": runner_manifest(Path(__file__)),
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "benchmark": str(executable),
        "max_concurrency": args.max_concurrency,
        "seed": args.seed,
        "repetitions": args.repetitions,
        "min_time": args.min_time,
        "warmup": workload_warmup_configuration(args),
        "bootstrap_samples": args.bootstrap_samples,
        "host_environment": host_environment,
        "order": order,
        "decision_rule": {
            "practical_speedup": args.practical_speedup,
            "stable_adjacent_points": args.stable_points,
            "requires_bootstrap_ci_below_one": True,
        },
        "stable_crossover_n": None,
        "environment": {
            "MKL_THREADING_LAYER": environment["MKL_THREADING_LAYER"],
            CONCURRENCY_ENVIRONMENT: environment.get(CONCURRENCY_ENVIRONMENT),
        },
        "linkage": linkage,
        "records": records,
        "summary": [],
    }
    artifact = JsonArtifact(args.out, scheduled_units=len(schedule))

    with tempfile.TemporaryDirectory(prefix="pgo-eigen-mkl-crossover-") as temp_dir:
        temporary = Path(temp_dir)
        for block_index, (repetition, matrix_n) in enumerate(schedule, start=1):
            policies = balanced_order(
                POLICIES,
                repetition=repetition - 1,
                seed=args.seed,
                block_key=f"n={matrix_n}",
            )
            print(
                f"[{block_index}/{len(schedule)}] repetition={repetition} "
                f"n={matrix_n} order={','.join(policies)}",
                flush=True,
            )
            measurements: dict[str, dict[str, Any]] = {}
            for policy in policies:
                label = f"r={repetition}:n={matrix_n}:policy={policy}"
                artifact.set_active(label)
                with artifact.capture_failures(lambda: payload):
                    measurements[policy] = run_case(
                        executable,
                        cases[matrix_n][policy],
                        temporary / f"{block_index}-{policy}.json",
                        args.min_time,
                        args.warmup_seconds,
                        environment,
                    )

            with artifact.capture_failures(lambda: payload):
                effective_concurrency = validate_block(
                    matrix_n,
                    measurements,
                    args.max_concurrency,
                    args.checksum_relative_tolerance,
                )
            local1_time = float(measurements["ExecutorLocal1"]["wall_seconds"])
            mkl_c_time = float(measurements["ExecutorMKLC"]["wall_seconds"])
            records.append(
                {
                    "repetition": repetition,
                    "matrix_n": matrix_n,
                    "order": policies,
                    "effective_concurrency": effective_concurrency,
                    "measurements": measurements,
                    "mkl_c_over_local1": mkl_c_time / local1_time,
                }
            )
            artifact.checkpoint(payload, completed_units=len(records))

    with artifact.capture_failures(lambda: payload):
        summary = summarize(
            records,
            args.seed,
            args.bootstrap_samples,
            args.practical_speedup,
        )
        crossover = stable_crossover(summary, args.stable_points)
        payload["summary"] = summary
        payload["stable_crossover_n"] = crossover
        artifact.complete(payload)

    print("\n   n   local1_us     mkl_c_us  mkl_c/local1          95% CI  evidence")
    for row in summary:
        times = row["median_wall_seconds"]
        confidence_interval = row["bootstrap_95pct_ci"]
        print(
            f"{row['matrix_n']:4d}  "
            f"{times['ExecutorLocal1'] * 1e6:10.3f}  "
            f"{times['ExecutorMKLC'] * 1e6:10.3f}  "
            f"{row['median_mkl_c_over_local1']:13.4f}  "
            f"[{confidence_interval[0]:.4f}, {confidence_interval[1]:.4f}]  "
            f"{row['evidence']}"
        )
    if crossover is None:
        print("\nNo stable crossover satisfied the configured decision rule.")
    else:
        print(f"\nStable MKL-budget-C crossover starts at n={crossover}.")
    print(f"Wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
