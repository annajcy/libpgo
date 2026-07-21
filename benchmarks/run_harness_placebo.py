#!/usr/bin/env python3
"""Gate timing runs with two labels that execute one identical MKL benchmark."""

from __future__ import annotations

import argparse
import json
import math
import random
import statistics
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


BENCHMARKS_ROOT = Path(__file__).resolve().parent
if str(BENCHMARKS_ROOT) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS_ROOT))

from benchmark_support.cpp_probe import (  # noqa: E402
    integer_counter,
    run_cpp_probe,
)
from benchmark_support.artifact import JsonArtifact, runner_manifest  # noqa: E402
from benchmark_support.mkl import (  # noqa: E402
    mkl_tbb_environment,
    verify_mkl_tbb_benchmark_linkage,
)
from benchmark_support.statistics import bootstrap_median_ci  # noqa: E402
from benchmark_support.harness import (  # noqa: E402
    add_benchmark_harness_arguments,
    prepare_benchmark_host,
)
from benchmark_support.schedule import balanced_order, order_configuration  # noqa: E402
from benchmark_support.warmup import (  # noqa: E402
    add_workload_warmup_arguments,
    validate_workload_warmup_arguments,
    workload_warmup_configuration,
)


LABELS = ("placebo_a", "placebo_b")
DEFAULT_MEDIAN_TOLERANCE = 0.02
DEFAULT_CI_TOLERANCE = 0.05
INVARIANT_COUNTERS = (
    "matrix_n",
    "requested_concurrency",
    "effective_global_concurrency",
    "configured_arena_concurrency",
    "configured_mkl_local_budget",
    "observed_arena_concurrency",
    "configured_warmup_min_operations",
    "actual_warmup_operations",
)
CONCURRENCY_ENVIRONMENT = "PGO_EIGEN_MKL_CROSSOVER_MAX_CONCURRENCY"
POLICIES = ("ExecutorLocal1", "ExecutorMKLC")
RESULT_MARKER = "PGO_EIGEN_MKL_CROSSOVER_RESULT"
RESULT_INTEGER_FIELDS = frozenset(INVARIANT_COUNTERS)
RESULT_FLOAT_FIELDS = frozenset({"checksum", "flops_per_operation"})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmark", type=Path)
    parser.add_argument("--policy", choices=POLICIES, default="ExecutorLocal1")
    parser.add_argument("--matrix-n", type=int, default=128)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--concurrency", type=int, default=8)
    parser.add_argument("--repetitions", type=int, default=16)
    parser.add_argument("--min-time", default="0.25s")
    parser.add_argument("--seed", type=int, default=20260720)
    parser.add_argument("--bootstrap-samples", type=int, default=5000)
    parser.add_argument(
        "--median-tolerance", type=float, default=DEFAULT_MEDIAN_TOLERANCE
    )
    parser.add_argument("--ci-tolerance", type=float, default=DEFAULT_CI_TOLERANCE)
    parser.add_argument("--checksum-relative-tolerance", type=float, default=1e-10)
    parser.add_argument("--dry-run", action="store_true")
    add_workload_warmup_arguments(
        parser, default_seconds=0.0, default_min_operations=10
    )
    add_benchmark_harness_arguments(parser)
    return parser.parse_args()


def ratio_summary(
    values: list[float], *, seed: int, bootstrap_samples: int
) -> dict[str, Any]:
    if not values or any(not math.isfinite(value) or value <= 0 for value in values):
        raise ValueError("placebo ratios must be finite and positive")
    return {
        "sample_count": len(values),
        "values": values,
        "median": statistics.median(values),
        "bootstrap_95pct_ci": bootstrap_median_ci(
            values, random.Random(seed), bootstrap_samples
        ),
    }


def equivalence_failures(
    summaries: dict[str, dict[str, Any]],
    *,
    median_tolerance: float,
    ci_tolerance: float,
) -> list[str]:
    failures: list[str] = []
    for name, summary in summaries.items():
        median = float(summary["median"])
        lower, upper = (float(value) for value in summary["bootstrap_95pct_ci"])
        if not 1.0 - median_tolerance <= median <= 1.0 + median_tolerance:
            failures.append(
                f"{name} median {median:.6f} is outside "
                f"[1-{median_tolerance:.2%}, 1+{median_tolerance:.2%}]"
            )
        if lower < 1.0 - ci_tolerance or upper > 1.0 + ci_tolerance:
            failures.append(
                f"{name} 95% interval [{lower:.6f}, {upper:.6f}] is not "
                f"contained in [1-{ci_tolerance:.2%}, 1+{ci_tolerance:.2%}]"
            )
    return failures


def validate_identical_results(
    measurements: dict[str, dict[str, Any]], checksum_relative_tolerance: float
) -> None:
    left = measurements[LABELS[0]]
    right = measurements[LABELS[1]]
    for counter in INVARIANT_COUNTERS:
        if integer_counter(left, counter) != integer_counter(right, counter):
            raise RuntimeError(f"Placebo labels reported different {counter} values.")

    checksums = [
        float(left.get("checksum", math.nan)),
        float(right.get("checksum", math.nan)),
    ]
    if not all(math.isfinite(value) for value in checksums):
        raise RuntimeError("Placebo labels did not report finite checksums.")
    tolerance = max(
        1e-12,
        checksum_relative_tolerance * max(abs(value) for value in checksums),
    )
    if abs(checksums[0] - checksums[1]) > tolerance:
        raise RuntimeError("Placebo labels produced different checksums.")


def analyze_records(
    records: list[dict[str, Any]], *, seed: int, bootstrap_samples: int
) -> dict[str, dict[str, Any]]:
    workload_label_ratios: list[float] = []
    workload_position_ratios: list[float] = []
    for record in records:
        measurements = record["measurements"]
        order = record["order"]
        workload_label_ratios.append(
            float(measurements[LABELS[1]]["wall_seconds"])
            / float(measurements[LABELS[0]]["wall_seconds"])
        )
        workload_position_ratios.append(
            float(measurements[order[1]]["wall_seconds"])
            / float(measurements[order[0]]["wall_seconds"])
        )

    ratio_groups = {
        "workload_placebo_b_over_a": workload_label_ratios,
        "workload_second_over_first": workload_position_ratios,
    }
    return {
        name: ratio_summary(
            values,
            seed=seed ^ sum(ord(character) for character in name),
            bootstrap_samples=bootstrap_samples,
        )
        for name, values in ratio_groups.items()
    }


def main() -> int:
    args = parse_args()
    if args.concurrency <= 0 or args.repetitions <= 0 or args.matrix_n <= 0:
        raise SystemExit(
            "--concurrency, --repetitions, and --matrix-n must be positive."
        )
    try:
        validate_workload_warmup_arguments(args)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    if args.bootstrap_samples <= 0:
        raise SystemExit("--bootstrap-samples must be positive.")
    if not 0 < args.median_tolerance <= args.ci_tolerance < 1:
        raise SystemExit("Require 0 < --median-tolerance <= --ci-tolerance < 1.")

    executable = args.benchmark.resolve()
    if not executable.exists():
        raise SystemExit(f"Benchmark executable does not exist: {executable}")
    environment = mkl_tbb_environment({CONCURRENCY_ENVIRONMENT: str(args.concurrency)})
    linkage = verify_mkl_tbb_benchmark_linkage(executable, environment)
    case = f"{args.policy}/n_{args.matrix_n}"

    order = order_configuration(
        args.repetitions,
        [("placebo_labels", LABELS)],
        allow_incomplete=args.allow_incomplete_order_cycle,
    )
    acceptance_criteria = {
        "median_ratio_range": [
            1.0 - args.median_tolerance,
            1.0 + args.median_tolerance,
        ],
        "bootstrap_95pct_ci_must_be_contained_in": [
            1.0 - args.ci_tolerance,
            1.0 + args.ci_tolerance,
        ],
        "ratios_checked": [
            "workload_placebo_b_over_a",
            "workload_second_over_first",
        ],
        "requires_identical_commands_counters_and_checksums": True,
    }
    runner = runner_manifest(Path(__file__))
    host_environment = prepare_benchmark_host(
        args, workers=args.concurrency, dry_run=args.dry_run
    )
    if args.dry_run:
        print(
            json.dumps(
                {
                    "runner": runner,
                    "case": case,
                    "order": order,
                    "acceptance_criteria": acceptance_criteria,
                    "host_environment": host_environment,
                },
                indent=2,
            )
        )
        return 0

    semantic_command = {
        "executable": str(executable),
        "case": case,
        "min_time": args.min_time,
        "warmup": workload_warmup_configuration(args),
        "environment": {
            "MKL_THREADING_LAYER": environment["MKL_THREADING_LAYER"],
            CONCURRENCY_ENVIRONMENT: environment[CONCURRENCY_ENVIRONMENT],
        },
    }
    records: list[dict[str, Any]] = []
    active_record: dict[str, Any] | None = None
    artifact = JsonArtifact(args.out, scheduled_units=args.repetitions)
    try:
        for repetition in range(1, args.repetitions + 1):
            labels = balanced_order(
                LABELS,
                repetition=repetition - 1,
                seed=args.seed,
                block_key=f"case={case}",
            )
            print(
                f"[{repetition}/{args.repetitions}] order={','.join(labels)}",
                flush=True,
            )
            active_record = {
                "repetition": repetition,
                "order": labels,
                "semantic_command": semantic_command,
                "measurements": {},
            }
            artifact.set_active(f"repetition={repetition}")
            measurements = active_record["measurements"]
            for label in labels:
                measurements[label] = run_cpp_probe(
                    executable,
                    [f"--policy={args.policy}", f"--matrix-n={args.matrix_n}"],
                    marker=RESULT_MARKER,
                    min_time=args.min_time,
                    warmup_seconds=args.warmup_seconds,
                    warmup_min_operations=args.warmup_min_operations,
                    string_fields=frozenset({"policy"}),
                    integer_fields=RESULT_INTEGER_FIELDS,
                    float_fields=RESULT_FLOAT_FIELDS,
                    environment=environment,
                )
            validate_identical_results(
                measurements,
                checksum_relative_tolerance=args.checksum_relative_tolerance,
            )
            records.append(active_record)
            active_record = None
            artifact.checkpoint(
                {
                    "runner": runner,
                    "benchmark": str(executable),
                    "case": case,
                    "acceptance_criteria": acceptance_criteria,
                    "host_environment": host_environment,
                    "records": records,
                },
                completed_units=len(records),
            )
    except Exception as error:
        payload = {
            "runner": runner,
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "benchmark": str(executable),
            "case": case,
            "concurrency": args.concurrency,
            "repetitions": args.repetitions,
            "min_time": args.min_time,
            "warmup": workload_warmup_configuration(args),
            "seed": args.seed,
            "bootstrap_samples": args.bootstrap_samples,
            "order": order,
            "acceptance_criteria": acceptance_criteria,
            "status": "aborted",
            "failures": [f"{type(error).__name__}: {error}"],
            "completed_repetitions": len(records),
            "active_record": active_record,
            "host_environment": host_environment,
            "linkage": linkage,
            "records": records,
        }
        artifact.fail(payload, error)
        print(f"Placebo gate aborted; wrote partial evidence to {args.out}")
        print(payload["failures"][0], file=sys.stderr)
        return 1

    summaries = analyze_records(
        records, seed=args.seed, bootstrap_samples=args.bootstrap_samples
    )
    failures = equivalence_failures(
        summaries,
        median_tolerance=args.median_tolerance,
        ci_tolerance=args.ci_tolerance,
    )
    payload = {
        "runner": runner,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "benchmark": str(executable),
        "case": case,
        "concurrency": args.concurrency,
        "repetitions": args.repetitions,
        "min_time": args.min_time,
        "warmup": workload_warmup_configuration(args),
        "seed": args.seed,
        "bootstrap_samples": args.bootstrap_samples,
        "order": order,
        "acceptance_criteria": acceptance_criteria,
        "status": "passed" if not failures else "failed",
        "failures": failures,
        "summaries": summaries,
        "host_environment": host_environment,
        "linkage": linkage,
        "records": records,
    }
    artifact.complete(payload)

    for name, summary in summaries.items():
        lower, upper = summary["bootstrap_95pct_ci"]
        print(
            f"{name}: median={summary['median']:.6f}, 95% CI=[{lower:.6f}, {upper:.6f}]"
        )
    print(f"Placebo gate {payload['status']}; wrote {args.out}")
    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
