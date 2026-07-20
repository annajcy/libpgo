#!/usr/bin/env python3
"""Run the diagnostic 2x3 oneMKL-budget/executor-arena experiment."""

from __future__ import annotations

import argparse
import math
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

from benchmark_support.conditioning import (  # noqa: E402
    add_benchmark_harness_arguments,
    guard_host_condition,
    precondition_host,
)
from benchmark_support.artifact import JsonArtifact, runner_manifest  # noqa: E402
from benchmark_support.schedule import balanced_order, order_configuration  # noqa: E402
from benchmark_support.google_benchmark import (  # noqa: E402
    integer_counter,
    list_cases,
    run_case,
)
from benchmark_support.mkl import (  # noqa: E402
    mkl_tbb_environment,
    verify_mkl_tbb_benchmark_linkage,
)
from benchmark_support.process import checked_output  # noqa: E402
from benchmark_support.statistics import bootstrap_median_ci  # noqa: E402


POLICIES = (
    "ExecutorDefault",
    "ExecutorLocal1",
    "ExecutorDefaultArena1",
    "ExecutorLocal1Arena1",
    "ExecutorMKLC",
    "ExecutorMKLCArena1",
)
WORKLOADS = ("EigenMklGemm", "NoBlas")
POLICY_PATTERN = "|".join(POLICIES)
CASE_PATTERN = re.compile(
    r"^EigenMklControlMatrix/"
    r"(EigenMklGemm|NoBlas)/"
    rf"({POLICY_PATTERN})"
    r"/c_(\d+)/tasks_(\d+)/n_(\d+)(?:/real_time)?$"
)


def executor_spec(policy: str, concurrency: int) -> tuple[int, int]:
    if policy == "ExecutorDefault":
        return concurrency, 0
    if policy == "ExecutorLocal1":
        return concurrency, 1
    if policy == "ExecutorDefaultArena1":
        return 1, 0
    if policy == "ExecutorLocal1Arena1":
        return 1, 1
    if policy == "ExecutorMKLC":
        return concurrency, concurrency
    if policy == "ExecutorMKLCArena1":
        return 1, concurrency
    raise ValueError(f"Unknown policy: {policy}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmark", type=Path)
    parser.add_argument(
        "--no-blas-benchmark",
        type=Path,
        help="Eigen-internal-GEMM executable (default: sibling no-BLAS target).",
    )
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=12)
    parser.add_argument("--min-time", default="0.05s")
    parser.add_argument("--warmup-time", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=20260715)
    parser.add_argument("--bootstrap-samples", type=int, default=2000)
    parser.add_argument("--checksum-relative-tolerance", type=float, default=1e-10)
    parser.add_argument("--case-limit", type=int, default=0)
    parser.add_argument(
        "--probe",
        type=Path,
        help="Untimed MKL_VERBOSE probe executable (default: sibling probe target).",
    )
    parser.add_argument("--skip-mkl-verbose-probe", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    add_benchmark_harness_arguments(parser)
    return parser.parse_args()


def benchmark_environment() -> dict[str, str]:
    return mkl_tbb_environment()


def discover_cases(
    executable: Path, environment: dict[str, str], expected_workload: str
) -> dict[tuple[str, int, int, int], dict[str, str]]:
    discovered: dict[tuple[str, int, int, int], dict[str, str]] = {}
    for name in list_cases(executable, environment):
        match = CASE_PATTERN.fullmatch(name)
        if not match:
            continue
        workload, policy, concurrency, outer_tasks, matrix_n = match.groups()
        key = (workload, int(concurrency), int(outer_tasks), int(matrix_n))
        discovered.setdefault(key, {})[policy] = name

    complete = {
        key: cases
        for key, cases in sorted(discovered.items())
        if set(cases) == set(POLICIES)
    }
    invalid = [key for key in complete if key[0] != expected_workload]
    if invalid:
        raise RuntimeError(
            f"{executable} exposed unexpected control-matrix workload(s): {invalid}."
        )
    return complete


def validate_block(
    key: tuple[str, int, int, int],
    measurements: dict[str, dict[str, Any]],
    checksum_relative_tolerance: float,
) -> None:
    workload, concurrency, outer_tasks, matrix_n = key
    checksums = [
        float(measurements[policy].get("checksum", math.nan)) for policy in POLICIES
    ]
    if not all(math.isfinite(value) for value in checksums):
        raise RuntimeError(
            f"Missing finite checksum for c={concurrency}, tasks={outer_tasks}, n={matrix_n}."
        )
    tolerance = max(
        1e-12,
        checksum_relative_tolerance * max(abs(value) for value in checksums),
    )
    if max(checksums) - min(checksums) > tolerance:
        raise RuntimeError(
            f"Checksum mismatch for c={concurrency}, tasks={outer_tasks}, n={matrix_n}."
        )

    for policy, row in measurements.items():
        expected_arena_concurrency, expected_mkl_budget = executor_spec(
            policy, concurrency
        )
        expected_calls = int(row["iterations"]) * outer_tasks
        expected = {
            "policy": POLICIES.index(policy),
            "workload": WORKLOADS.index(workload),
            "uses_blas": 1 if workload == "EigenMklGemm" else 0,
            "configured_global_concurrency": concurrency,
            "effective_global_concurrency": concurrency,
            "configured_arena_concurrency": expected_arena_concurrency,
            "configured_mkl_local_budget": expected_mkl_budget,
            "outer_tasks": outer_tasks,
            "matrix_n": matrix_n,
            "observed_arena_concurrency": expected_arena_concurrency,
            "body_calls": expected_calls,
        }
        for counter_name, expected_value in expected.items():
            observed = integer_counter(row, counter_name)
            if observed != expected_value:
                raise RuntimeError(
                    f"{policy} reported {counter_name}={observed}; "
                    f"expected {expected_value}."
                )


def ratios(measurements: dict[str, dict[str, Any]]) -> dict[str, float]:
    times = {policy: float(measurements[policy]["wall_seconds"]) for policy in POLICIES}
    return {
        "local1_over_default_at_arena_c": (
            times["ExecutorLocal1"] / times["ExecutorDefault"]
        ),
        "mkl_c_over_default_at_arena_c": (
            times["ExecutorMKLC"] / times["ExecutorDefault"]
        ),
        "local1_over_default_at_arena_1": (
            times["ExecutorLocal1Arena1"] / times["ExecutorDefaultArena1"]
        ),
        "mkl_c_over_default_at_arena_1": (
            times["ExecutorMKLCArena1"] / times["ExecutorDefaultArena1"]
        ),
        "arena_1_over_c_at_budget_0": (
            times["ExecutorDefaultArena1"] / times["ExecutorDefault"]
        ),
        "arena_1_over_c_at_budget_1": (
            times["ExecutorLocal1Arena1"] / times["ExecutorLocal1"]
        ),
        "arena_1_over_c_at_budget_c": (
            times["ExecutorMKLCArena1"] / times["ExecutorMKLC"]
        ),
    }


def summarize(
    records: list[dict[str, Any]], seed: int, bootstrap_samples: int
) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, int, int, int], list[dict[str, Any]]] = {}
    for record in records:
        key = (
            record["workload"],
            record["concurrency"],
            record["outer_tasks"],
            record["matrix_n"],
        )
        grouped.setdefault(key, []).append(record)

    randomizer = random.Random(seed ^ 0x2B2B)
    summary: list[dict[str, Any]] = []
    for (workload, concurrency, outer_tasks, matrix_n), blocks in sorted(
        grouped.items()
    ):
        ratio_names = tuple(blocks[0]["ratios"])
        ratio_values = {
            name: [float(block["ratios"][name]) for block in blocks]
            for name in ratio_names
        }
        summary.append(
            {
                "workload": workload,
                "concurrency": concurrency,
                "outer_tasks": outer_tasks,
                "matrix_n": matrix_n,
                "repetitions": len(blocks),
                "median_wall_seconds": {
                    policy: statistics.median(
                        float(block["measurements"][policy]["wall_seconds"])
                        for block in blocks
                    )
                    for policy in POLICIES
                },
                "median_ratios": {
                    name: statistics.median(values)
                    for name, values in ratio_values.items()
                },
                "bootstrap_95pct_ci": {
                    name: bootstrap_median_ci(values, randomizer, bootstrap_samples)
                    for name, values in ratio_values.items()
                },
                "median_extra_threads": {
                    policy: statistics.median(
                        float(block["measurements"][policy]["extra_threads"])
                        for block in blocks
                    )
                    for policy in POLICIES
                },
            }
        )
    return summary


def run_mkl_verbose_probe(
    executable: Path,
    cases: dict[tuple[str, int, int, int], dict[str, str]],
    environment: dict[str, str],
) -> list[dict[str, Any]]:
    probe_environment = environment.copy()
    probe_environment["MKL_VERBOSE"] = "1"
    probe_cases = sorted(
        key for key in cases if key[0] == "EigenMklGemm" and key[1] == key[2]
    )
    records: list[dict[str, Any]] = []
    for _, concurrency, outer_tasks, matrix_n in probe_cases:
        for policy in POLICIES:
            output = checked_output(
                [
                    str(executable),
                    f"--policy={policy}",
                    f"--concurrency={concurrency}",
                    f"--outer-tasks={outer_tasks}",
                    f"--matrix-n={matrix_n}",
                ],
                probe_environment,
            )
            dgemm_lines = [
                line for line in output.splitlines() if "dgemm" in line.lower()
            ]
            if not dgemm_lines:
                raise RuntimeError(
                    f"MKL_VERBOSE probe emitted no DGEMM line for {policy}, c={concurrency}."
                )
            if "PGO_MKL_VERBOSE_PROBE_BEGIN" not in output or (
                "PGO_MKL_VERBOSE_PROBE_END" not in output
            ):
                raise RuntimeError(
                    f"Probe markers are missing for {policy}, c={concurrency}."
                )
            records.append(
                {
                    "policy": policy,
                    "concurrency": concurrency,
                    "outer_tasks": outer_tasks,
                    "matrix_n": matrix_n,
                    "dgemm_lines": dgemm_lines,
                }
            )
    return records


def main() -> int:
    args = parse_args()
    if args.repetitions < 1 or args.bootstrap_samples < 1:
        raise SystemExit("--repetitions and --bootstrap-samples must be positive.")
    if args.warmup_time < 0 or args.case_limit < 0:
        raise SystemExit("--warmup-time and --case-limit must be non-negative.")

    executable = args.benchmark.resolve()
    if not executable.exists():
        raise SystemExit(f"Benchmark executable does not exist: {executable}")
    no_blas_executable = (
        args.no_blas_benchmark
        or executable.with_name("eigen_mkl_control_matrix_no_blas_benchmark")
    ).resolve()
    if not no_blas_executable.exists():
        raise SystemExit(
            f"No-BLAS benchmark executable does not exist: {no_blas_executable}"
        )

    environment = benchmark_environment()
    linkage = {
        "eigen_mkl_gemm": verify_mkl_tbb_benchmark_linkage(
            executable, environment, require_dgemm=True
        ),
        "eigen_internal_gemm": verify_mkl_tbb_benchmark_linkage(
            no_blas_executable, environment, require_dgemm=False
        ),
    }
    cases = discover_cases(executable, environment, "EigenMklGemm")
    cases.update(discover_cases(no_blas_executable, environment, "NoBlas"))
    if args.case_limit:
        cases = dict(list(cases.items())[: args.case_limit])
    if not cases:
        raise SystemExit("No complete six-policy control-matrix cases were found.")

    probe: Path | None = None
    if not args.skip_mkl_verbose_probe:
        probe = (
            args.probe or executable.with_name("eigen_mkl_control_matrix_probe")
        ).resolve()
        if not probe.exists():
            raise SystemExit(f"MKL_VERBOSE probe executable does not exist: {probe}")
        verify_mkl_tbb_benchmark_linkage(probe, environment, require_dgemm=True)

    print(f"Verified MKL-TBB linkage; matched {len(cases)} six-policy case(s).")
    for workload, concurrency, outer_tasks, matrix_n in cases:
        print(f"{workload}: c={concurrency} tasks={outer_tasks} n={matrix_n}")
    order = order_configuration(
        args.repetitions,
        [("policies", POLICIES)],
        allow_incomplete=args.allow_incomplete_order_cycle,
    )
    host_preconditioning = precondition_host(
        args,
        workers=max(key[1] for key in cases),
        dry_run=args.dry_run,
    )
    if args.dry_run:
        return 0

    schedule = [
        (repetition, key)
        for repetition in range(1, args.repetitions + 1)
        for key in cases
    ]
    randomizer = random.Random(args.seed)
    randomizer.shuffle(schedule)
    records: list[dict[str, Any]] = []
    runner = runner_manifest(Path(__file__))
    checkpoint_payload = {
        "runner": runner,
        "schema_version": 2,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "benchmark": str(executable),
        "no_blas_benchmark": str(no_blas_executable),
        "seed": args.seed,
        "repetitions": args.repetitions,
        "min_time": args.min_time,
        "warmup_time": args.warmup_time,
        "bootstrap_samples": args.bootstrap_samples,
        "host_preconditioning": host_preconditioning,
        "order": order,
        "linkage": linkage,
        "records": records,
    }
    artifact = JsonArtifact(args.out, scheduled_units=len(schedule))

    with tempfile.TemporaryDirectory(
        prefix="pgo-eigen-mkl-control-matrix-"
    ) as temp_dir:
        temporary = Path(temp_dir)
        for block_index, (repetition, key) in enumerate(schedule, start=1):
            workload, concurrency, outer_tasks, matrix_n = key
            policies = balanced_order(
                POLICIES,
                repetition=repetition - 1,
                seed=args.seed,
                block_key=(
                    f"{workload}:c={concurrency}:tasks={outer_tasks}:n={matrix_n}"
                ),
            )
            print(
                f"[{block_index}/{len(schedule)}] repetition={repetition} "
                f"{workload}: c={concurrency} tasks={outer_tasks} n={matrix_n} "
                f"order={','.join(policies)}",
                flush=True,
            )
            measurements: dict[str, dict[str, Any]] = {}
            for policy in policies:
                label = (
                    f"r={repetition}:workload={workload}:c={concurrency}:"
                    f"tasks={outer_tasks}:n={matrix_n}:policy={policy}"
                )
                artifact.set_active(label)
                with artifact.capture_failures(lambda: checkpoint_payload):
                    guard_host_condition(
                        args, host_preconditioning, label=label
                    )
                    measurements[policy] = run_case(
                        executable
                        if workload == "EigenMklGemm"
                        else no_blas_executable,
                        cases[key][policy],
                        temporary / f"{block_index}-{policy}.json",
                        args.min_time,
                        args.warmup_time,
                        environment,
                    )

            with artifact.capture_failures(lambda: checkpoint_payload):
                validate_block(key, measurements, args.checksum_relative_tolerance)
            records.append(
                {
                    "repetition": repetition,
                    "workload": workload,
                    "concurrency": concurrency,
                    "outer_tasks": outer_tasks,
                    "matrix_n": matrix_n,
                    "order": policies,
                    "measurements": measurements,
                    "ratios": ratios(measurements),
                }
            )
            artifact.checkpoint(checkpoint_payload, completed_units=len(records))

    with artifact.capture_failures(lambda: checkpoint_payload):
        verbose_probe = (
            [] if probe is None else run_mkl_verbose_probe(probe, cases, environment)
        )
        summary = summarize(records, args.seed, args.bootstrap_samples)
    payload = {
        "runner": runner,
        "schema_version": 2,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "benchmark": str(executable),
        "no_blas_benchmark": str(no_blas_executable),
        "seed": args.seed,
        "repetitions": args.repetitions,
        "min_time": args.min_time,
        "warmup_time": args.warmup_time,
        "bootstrap_samples": args.bootstrap_samples,
        "thread_telemetry_source": "untimed_policy_warmup",
        "host_preconditioning": host_preconditioning,
        "order": order,
        "mkl_verbose_probe": verbose_probe,
        "environment": {"MKL_THREADING_LAYER": environment["MKL_THREADING_LAYER"]},
        "executor_case_note": (
            "Every case applies its MKL TLS budget only through "
            "ArenaThreadingExecutor; the benchmark never calls an MKL "
            "threading-control API directly."
        ),
        "factor_mapping": {
            "ExecutorDefault": {
                "arena_concurrency": "C",
                "mkl_local_thread_budget": 0,
            },
            "ExecutorLocal1": {
                "arena_concurrency": "C",
                "mkl_local_thread_budget": 1,
            },
            "ExecutorDefaultArena1": {
                "arena_concurrency": 1,
                "mkl_local_thread_budget": 0,
            },
            "ExecutorLocal1Arena1": {
                "arena_concurrency": 1,
                "mkl_local_thread_budget": 1,
            },
            "ExecutorMKLC": {
                "arena_concurrency": "C",
                "mkl_local_thread_budget": "C",
            },
            "ExecutorMKLCArena1": {
                "arena_concurrency": 1,
                "mkl_local_thread_budget": "C",
            },
        },
        "linkage": linkage,
        "records": records,
        "summary": summary,
    }
    artifact.complete(payload)

    print(
        "\nworkload       c tasks    n  default_c   local1_c     mklc_c  "
        "default_a1  local1_a1    mklc_a1  local1/default(c)  local1/default(1)"
    )
    for row in summary:
        times = row["median_wall_seconds"]
        row_ratios = row["median_ratios"]
        print(
            f"{row['workload']:<12} {row['concurrency']:2d} {row['outer_tasks']:5d} {row['matrix_n']:4d}  "
            f"{times['ExecutorDefault'] * 1e3:9.3f}  "
            f"{times['ExecutorLocal1'] * 1e3:9.3f}  "
            f"{times['ExecutorMKLC'] * 1e3:9.3f}  "
            f"{times['ExecutorDefaultArena1'] * 1e3:10.3f}  "
            f"{times['ExecutorLocal1Arena1'] * 1e3:9.3f}  "
            f"{times['ExecutorMKLCArena1'] * 1e3:9.3f}  "
            f"{row_ratios['local1_over_default_at_arena_c']:17.3f}  "
            f"{row_ratios['local1_over_default_at_arena_1']:17.3f}"
        )
    print(f"\nWrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
