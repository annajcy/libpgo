#!/usr/bin/env python3
"""Compare persistent and per-invocation nested arena-executor lifetimes."""

from __future__ import annotations

import argparse
import hashlib
import os
import random
import statistics
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


BENCHMARKS_ROOT = Path(__file__).resolve().parents[1]
if str(BENCHMARKS_ROOT) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS_ROOT))

from benchmark_support.artifact import JsonArtifact, runner_manifest  # noqa: E402
from benchmark_support.cpp_probe import integer_counter, run_cpp_probe  # noqa: E402
from benchmark_support.harness import (  # noqa: E402
    add_benchmark_harness_arguments,
    finish_benchmark_host,
    prepare_benchmark_host,
)
from benchmark_support.schedule import balanced_order, order_configuration  # noqa: E402
from benchmark_support.statistics import (  # noqa: E402
    bootstrap_median_ci,
    median_and_mad,
)
from benchmark_support.validation import require_positive  # noqa: E402
from benchmark_support.warmup import (  # noqa: E402
    add_workload_warmup_arguments,
    validate_workload_warmup_arguments,
    workload_warmup_configuration,
)


CASES = ("reuse", "recreate")
RECORDED_ENVIRONMENT_VARIABLES = (
    "MKL_THREADING_LAYER",
    "MKL_NUM_THREADS",
    "MKL_DYNAMIC",
    "OMP_NUM_THREADS",
    "OMP_DYNAMIC",
)
RESULT_MARKER = "PGO_ARENA_EXECUTOR_LIFECYCLE_RESULT"
RESULT_INTEGER_FIELDS = frozenset(
    {
        "configured_concurrency",
        "effective_concurrency",
        "outer_tasks",
        "configured_inner_concurrency",
        "configured_measurement_min_operations",
        "measurement_inner_calls",
        "timed_inner_constructions",
        "retained_attachments",
        "retired_after_warmup",
        "retired_after_measurement",
        "retired_after_attachment_release",
        "retired_after_reuse_release",
        "retired_after_drain",
        "drain_succeeded",
        "checksum",
    }
)
RESULT_FLOAT_FIELDS = frozenset(
    {
        "process_cpu_seconds",
        "attachment_release_seconds",
        "drain_seconds",
    }
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmark", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--concurrency", type=int, default=8)
    parser.add_argument("--outer-tasks", type=int, nargs="+", default=[1, 8, 32])
    parser.add_argument("--repetitions", type=int, default=8)
    parser.add_argument("--min-time", default="0.25s")
    parser.add_argument("--seed", type=int, default=20260722)
    parser.add_argument("--bootstrap-samples", type=int, default=10000)
    parser.add_argument(
        "--equivalence-band",
        type=float,
        default=0.05,
        help="Relative recreate/reuse equivalence half-width.",
    )
    parser.add_argument("--drain-timeout-ms", type=int, default=5000)
    parser.add_argument("--dry-run", action="store_true")
    add_workload_warmup_arguments(
        parser, default_seconds=0.0, default_min_operations=10
    )
    add_benchmark_harness_arguments(parser)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    require_positive(args.concurrency, "--concurrency")
    require_positive(args.repetitions, "--repetitions")
    require_positive(args.bootstrap_samples, "--bootstrap-samples")
    validate_workload_warmup_arguments(args)
    if not 0.0 <= args.equivalence_band < 1.0:
        raise ValueError("--equivalence-band must be in [0, 1).")
    if args.drain_timeout_ms < 0:
        raise ValueError("--drain-timeout-ms must be nonnegative.")
    if not args.outer_tasks:
        raise ValueError("--outer-tasks must not be empty.")
    for value in args.outer_tasks:
        require_positive(value, "--outer-tasks")
    if len(set(args.outer_tasks)) != len(args.outer_tasks):
        raise ValueError("--outer-tasks must not contain duplicates.")


def validate_measurement(
    row: dict[str, Any], lifecycle: str, outer_tasks: int, concurrency: int
) -> None:
    if row["case"] != lifecycle:
        raise RuntimeError(f"{lifecycle} probe reported case={row['case']}.")
    if integer_counter(row, "configured_concurrency") != concurrency:
        raise RuntimeError(f"{lifecycle} used the wrong configured concurrency.")
    if integer_counter(row, "effective_concurrency") != concurrency:
        raise RuntimeError(f"{lifecycle} established the wrong effective concurrency.")
    if integer_counter(row, "outer_tasks") != outer_tasks:
        raise RuntimeError(f"{lifecycle} used the wrong outer-task count.")
    if integer_counter(row, "configured_inner_concurrency") != 1:
        raise RuntimeError(f"{lifecycle} used the wrong inner concurrency.")

    batches = integer_counter(row, "measurement_operations")
    inner_calls = integer_counter(row, "measurement_inner_calls")
    if inner_calls != batches * outer_tasks:
        raise RuntimeError(f"{lifecycle} reported inconsistent inner-call counts.")
    if integer_counter(row, "checksum") != inner_calls:
        raise RuntimeError(f"{lifecycle} failed its checksum validation.")

    expected_constructions = 0 if lifecycle == "reuse" else inner_calls
    if integer_counter(row, "timed_inner_constructions") != expected_constructions:
        raise RuntimeError(f"{lifecycle} reported the wrong construction count.")
    if integer_counter(row, "retained_attachments") != 0:
        raise RuntimeError(f"{lifecycle} unexpectedly retained attached arenas.")
    if integer_counter(row, "drain_succeeded") != 1:
        raise RuntimeError(f"{lifecycle} did not drain successfully.")
    if integer_counter(row, "retired_after_drain") != 0:
        raise RuntimeError(f"{lifecycle} left retired States after drain.")

    row["amortized_wall_nanoseconds_per_inner_call"] = (
        float(row["measurement_wall_seconds"]) * 1.0e9 / inner_calls
    )
    row["amortized_cpu_nanoseconds_per_inner_call"] = (
        float(row["process_cpu_seconds"]) * 1.0e9 / inner_calls
    )


def summarize(
    records: list[dict[str, Any]],
    seed: int,
    bootstrap_samples: int,
    equivalence_band: float,
) -> list[dict[str, Any]]:
    summary: list[dict[str, Any]] = []
    for outer_tasks in sorted({int(record["outer_tasks"]) for record in records}):
        blocks = [
            record for record in records if int(record["outer_tasks"]) == outer_tasks
        ]
        ratios = [float(block["recreate_over_reuse"]) for block in blocks]
        deltas = [
            float(block["extra_amortized_wall_ns_per_inner_call"])
            for block in blocks
        ]
        ratio_randomizer = random.Random(seed ^ (outer_tasks << 8) ^ 0xA11CE)
        delta_randomizer = random.Random(seed ^ (outer_tasks << 8) ^ 0xD371A)
        ratio_ci = bootstrap_median_ci(
            ratios, ratio_randomizer, bootstrap_samples
        )
        delta_ci = bootstrap_median_ci(
            deltas, delta_randomizer, bootstrap_samples
        )
        median_ratio, ratio_mad = median_and_mad(ratios)
        median_delta, delta_mad = median_and_mad(deltas)

        max_retired_after_measurement = {
            lifecycle: max(
                integer_counter(
                    block["measurements"][lifecycle],
                    "retired_after_measurement",
                )
                for block in blocks
            )
            for lifecycle in CASES
        }
        lower = 1.0 - equivalence_band
        upper = 1.0 + equivalence_band
        if ratio_ci[0] > upper:
            evidence = "recreate_slower_beyond_equivalence_band"
        elif ratio_ci[1] < lower:
            evidence = "recreate_faster_beyond_equivalence_band"
        elif ratio_ci[0] >= lower and ratio_ci[1] <= upper:
            evidence = "equivalent_within_band"
        else:
            evidence = "inconclusive"

        summary.append(
            {
                "outer_tasks": outer_tasks,
                "repetitions": len(blocks),
                "median_amortized_wall_nanoseconds_per_inner_call": {
                    lifecycle: statistics.median(
                        float(block["measurements"][lifecycle][
                            "amortized_wall_nanoseconds_per_inner_call"
                        ])
                        for block in blocks
                    )
                    for lifecycle in CASES
                },
                "median_amortized_cpu_nanoseconds_per_inner_call": {
                    lifecycle: statistics.median(
                        float(block["measurements"][lifecycle][
                            "amortized_cpu_nanoseconds_per_inner_call"
                        ])
                        for block in blocks
                    )
                    for lifecycle in CASES
                },
                "median_recreate_over_reuse": median_ratio,
                "recreate_over_reuse_mad": ratio_mad,
                "recreate_over_reuse_bootstrap_95pct_ci": ratio_ci,
                "median_extra_amortized_wall_nanoseconds_per_inner_call": median_delta,
                "extra_amortized_wall_nanoseconds_per_inner_call_mad": delta_mad,
                "extra_amortized_wall_nanoseconds_per_inner_call_bootstrap_95pct_ci": delta_ci,
                "max_retired_after_measurement": max_retired_after_measurement,
                "retirement_backlog_observed": any(
                    max_retired_after_measurement.values()
                ),
                "evidence": evidence,
            }
        )
    return summary


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        order = order_configuration(
            args.repetitions,
            [("lifecycle_cases", CASES)],
            allow_incomplete=args.allow_incomplete_order_cycle,
        )
    except ValueError as error:
        raise SystemExit(str(error)) from error

    executable = args.benchmark.resolve()
    if not executable.is_file():
        raise SystemExit(f"Benchmark executable does not exist: {executable}")

    host_environment = prepare_benchmark_host(
        args, workers=args.concurrency, dry_run=args.dry_run
    )
    if args.dry_run:
        return 0

    schedule = [
        (repetition, outer_tasks)
        for repetition in range(1, args.repetitions + 1)
        for outer_tasks in args.outer_tasks
    ]
    random.Random(args.seed).shuffle(schedule)
    records: list[dict[str, Any]] = []
    payload: dict[str, Any] = {
        "runner": runner_manifest(Path(__file__)),
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "benchmark": str(executable),
        "benchmark_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
        "source_sha256": {
            name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
            for name in (
                "CMakeLists.txt",
                "arena_executor_lifecycle_benchmark.cpp",
            )
        },
        "environment": {
            name: os.environ[name]
            for name in RECORDED_ENVIRONMENT_VARIABLES
            if name in os.environ
        },
        "concurrency": args.concurrency,
        "outer_tasks": args.outer_tasks,
        "repetitions": args.repetitions,
        "min_time": args.min_time,
        "warmup": workload_warmup_configuration(args),
        "seed": args.seed,
        "bootstrap_samples": args.bootstrap_samples,
        "equivalence_band": args.equivalence_band,
        "drain_timeout_ms": args.drain_timeout_ms,
        "host_environment": host_environment,
        "order": order,
        "records": records,
        "summary": [],
    }
    artifact = JsonArtifact(args.out, scheduled_units=len(schedule))

    for block_index, (repetition, outer_tasks) in enumerate(schedule, start=1):
        lifecycle_order = balanced_order(
            CASES,
            repetition=repetition - 1,
            seed=args.seed,
            block_key=f"outer_tasks={outer_tasks}",
        )
        print(
            f"[{block_index}/{len(schedule)}] repetition={repetition} "
            f"outer_tasks={outer_tasks} order={','.join(lifecycle_order)}",
            flush=True,
        )
        measurements: dict[str, dict[str, Any]] = {}
        for lifecycle in lifecycle_order:
            artifact.set_active(
                f"r={repetition}:outer_tasks={outer_tasks}:case={lifecycle}"
            )
            with artifact.capture_failures(lambda: payload):
                row = run_cpp_probe(
                    executable,
                    [
                        f"--case={lifecycle}",
                        f"--concurrency={args.concurrency}",
                        f"--outer-tasks={outer_tasks}",
                        f"--drain-timeout-ms={args.drain_timeout_ms}",
                    ],
                    marker=RESULT_MARKER,
                    min_time=args.min_time,
                    warmup_seconds=args.warmup_seconds,
                    warmup_min_operations=args.warmup_min_operations,
                    string_fields=frozenset({"case"}),
                    integer_fields=RESULT_INTEGER_FIELDS,
                    float_fields=RESULT_FLOAT_FIELDS,
                )
                validate_measurement(
                    row, lifecycle, outer_tasks, args.concurrency
                )
                measurements[lifecycle] = row

        reuse = measurements["reuse"]
        recreate = measurements["recreate"]
        reuse_batch_seconds = float(reuse["wall_seconds"])
        recreate_batch_seconds = float(recreate["wall_seconds"])
        records.append(
            {
                "repetition": repetition,
                "outer_tasks": outer_tasks,
                "order": lifecycle_order,
                "measurements": measurements,
                "recreate_over_reuse": recreate_batch_seconds / reuse_batch_seconds,
                "extra_amortized_wall_ns_per_inner_call": (
                    (recreate_batch_seconds - reuse_batch_seconds)
                    * 1.0e9
                    / outer_tasks
                ),
            }
        )
        artifact.checkpoint(payload, completed_units=len(records))

    with artifact.capture_failures(lambda: payload):
        payload["summary"] = summarize(
            records,
            args.seed,
            args.bootstrap_samples,
            args.equivalence_band,
        )
        finish_benchmark_host(host_environment)
        artifact.complete(payload)

    print(
        "\n tasks  reuse_amort_ns  recreate_amort_ns  recreate/reuse  "
        "extra_amort_ns  evidence"
    )
    for row in payload["summary"]:
        times = row["median_amortized_wall_nanoseconds_per_inner_call"]
        print(
            f"{row['outer_tasks']:6d}  {times['reuse']:9.2f}  "
            f"{times['recreate']:11.2f}  "
            f"{row['median_recreate_over_reuse']:14.3f}  "
            f"{row['median_extra_amortized_wall_nanoseconds_per_inner_call']:13.2f}  "
            f"{row['evidence']}"
        )
    print(f"Wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
