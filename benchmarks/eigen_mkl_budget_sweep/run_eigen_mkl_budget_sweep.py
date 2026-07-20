#!/usr/bin/env python3
"""Measure oneMKL task decomposition and timing over arena and budget settings."""

from __future__ import annotations

import argparse
import csv
import json
import os
import random
import re
import shlex
import statistics
import subprocess
import sys
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


BENCHMARKS_ROOT = Path(__file__).resolve().parents[1]
if str(BENCHMARKS_ROOT) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS_ROOT))

from host_preconditioning import (  # noqa: E402
    add_host_preconditioning_arguments,
    balanced_order,
    guard_host_condition,
    order_configuration,
    precondition_host,
)
from benchmark_support.mkl import (  # noqa: E402
    mkl_tbb_environment,
    verify_mkl_tbb_probe_linkage,
)
from benchmark_support.process import (  # noqa: E402
    checked_output,
    parse_key_value_marker,
    report_output,
    resolve_file,
    resolve_vtune,
)
from benchmark_support.statistics import median_and_mad  # noqa: E402
from benchmark_support.validation import (  # noqa: E402
    require_nonnegative,
    require_positive,
)


RESULT_PREFIX = "PGO_MKL_BUDGET_SWEEP_RESULT"
INTEGER_FIELDS = frozenset({
    "configured_global_concurrency",
    "effective_global_concurrency",
    "configured_arena_concurrency",
    "configured_mkl_local_budget",
    "observed_arena_concurrency",
    "observed_mkl_max_threads_min",
    "observed_mkl_max_threads_max",
    "outer_tasks",
    "matrix_n",
    "warmup_iterations",
    "profile_iterations",
    "measured_gemm_calls",
    "process_gemm_calls",
})
FLOAT_FIELDS = frozenset({"wall_seconds", "process_cpu_seconds", "checksum"})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("probe", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--concurrency", type=int, default=8)
    parser.add_argument(
        "--arena-concurrency",
        type=int,
        help=(
            "Deprecated compatibility alias for one private arena width. "
            "Use --arena-concurrencies for the threading surface."
        ),
    )
    parser.add_argument(
        "--arena-concurrencies",
        type=int,
        nargs="+",
        help=(
            "Private arena widths to sweep. Defaults to powers of two up to "
            "--concurrency, plus --concurrency when needed."
        ),
    )
    parser.add_argument(
        "--mkl-local-thread-budgets",
        type=int,
        nargs="+",
        default=[0, 1, 2, 4, 8, 16],
    )
    parser.add_argument(
        "--outer-tasks",
        type=int,
        nargs="+",
        default=[1, 8, 32],
        help="Outer-task counts used for unprofiled timing runs.",
    )
    parser.add_argument(
        "--profile-outer-tasks",
        type=int,
        nargs="+",
        default=[1],
        help=(
            "Outer-task counts collected with VTune when --collect-vtune is set. "
            "Only 1 yields a clean internal-oneMKL task/GEMM measurement."
        ),
    )
    parser.add_argument("--matrix-n", type=int, default=1024)
    parser.add_argument("--warmup-iterations", type=int, default=3)
    parser.add_argument("--profile-iterations", type=int, default=50)
    parser.add_argument("--timing-repetitions", type=int, default=12)
    parser.add_argument("--profile-repetitions", type=int, default=12)
    parser.add_argument("--seed", type=int, default=20260716)
    parser.add_argument("--collect-vtune", action="store_true")
    parser.add_argument("--vtune", type=Path)
    parser.add_argument(
        "--sudo",
        action="store_true",
        help="Run VTune through sudo and restore ownership of its result directories.",
    )
    parser.add_argument("--dry-run", action="store_true")
    add_host_preconditioning_arguments(parser)
    return parser.parse_args()


def default_arena_concurrencies(concurrency: int) -> list[int]:
    values: list[int] = []
    value = 1
    while value <= concurrency:
        values.append(value)
        value *= 2
    if values[-1] != concurrency:
        values.append(concurrency)
    return values


def validate_args(args: argparse.Namespace) -> None:
    require_positive(args.concurrency, "--concurrency")
    if args.arena_concurrency is not None and args.arena_concurrencies is not None:
        raise ValueError(
            "Pass either --arena-concurrency or --arena-concurrencies, not both."
        )
    if args.arena_concurrency is not None:
        args.arena_concurrencies = [args.arena_concurrency]
    elif args.arena_concurrencies is None:
        args.arena_concurrencies = default_arena_concurrencies(args.concurrency)
    for arena in args.arena_concurrencies:
        require_positive(arena, "--arena-concurrencies")
        if arena > args.concurrency:
            raise ValueError(
                "--arena-concurrencies cannot exceed --concurrency; use a larger "
                "global control explicitly if that is the intended experiment."
            )
    require_positive(args.matrix_n, "--matrix-n")
    require_nonnegative(args.warmup_iterations, "--warmup-iterations")
    require_positive(args.profile_iterations, "--profile-iterations")
    require_positive(args.timing_repetitions, "--timing-repetitions")
    require_positive(args.profile_repetitions, "--profile-repetitions")
    for budget in args.mkl_local_thread_budgets:
        require_nonnegative(budget, "--mkl-local-thread-budgets")
    for outer_tasks in args.outer_tasks:
        require_positive(outer_tasks, "--outer-tasks")
    for outer_tasks in args.profile_outer_tasks:
        require_positive(outer_tasks, "--profile-outer-tasks")
    if len(set(args.mkl_local_thread_budgets)) != len(args.mkl_local_thread_budgets):
        raise ValueError("--mkl-local-thread-budgets must not contain duplicates.")
    if len(set(args.arena_concurrencies)) != len(args.arena_concurrencies):
        raise ValueError("--arena-concurrencies must not contain duplicates.")
    if len(set(args.outer_tasks)) != len(args.outer_tasks):
        raise ValueError("--outer-tasks must not contain duplicates.")
    if len(set(args.profile_outer_tasks)) != len(args.profile_outer_tasks):
        raise ValueError("--profile-outer-tasks must not contain duplicates.")


def probe_command(
    probe: Path,
    args: argparse.Namespace,
    arena: int,
    budget: int,
    outer_tasks: int,
) -> list[str]:
    return [
        str(probe),
        f"--concurrency={args.concurrency}",
        f"--arena-concurrency={arena}",
        f"--mkl-local-thread-budget={budget}",
        f"--outer-tasks={outer_tasks}",
        f"--matrix-n={args.matrix_n}",
        f"--warmup-iterations={args.warmup_iterations}",
        f"--profile-iterations={args.profile_iterations}",
    ]


def sudo_environment_prefix(
    use_sudo: bool, environment: Mapping[str, str]
) -> list[str]:
    if not use_sudo:
        return []
    assignments = ["MKL_THREADING_LAYER=TBB"]
    for name in ("MKL_NUM_THREADS", "MKL_DYNAMIC"):
        value = environment.get(name)
        if value is not None:
            assignments.append(f"{name}={value}")
    return ["sudo", "env", *assignments]


def case_name(arena: int, budget: int, outer_tasks: int, repetition: int) -> str:
    return f"arena{arena}-budget{budget}-tasks{outer_tasks}-r{repetition}"


def make_jobs(
    args: argparse.Namespace,
    outer_tasks_values: list[int],
    repetitions: int,
    seed: int,
) -> list[dict[str, int]]:
    repetition_blocks = [
        (repetition, outer_tasks)
        for repetition in range(1, repetitions + 1)
        for outer_tasks in outer_tasks_values
    ]
    random.Random(seed).shuffle(repetition_blocks)
    jobs: list[dict[str, int]] = []
    for repetition, outer_tasks in repetition_blocks:
        arenas = balanced_order(
            args.arena_concurrencies,
            repetition=repetition - 1,
            seed=seed,
            block_key=f"outer_tasks={outer_tasks}:arenas",
        )
        for arena in arenas:
            budgets = balanced_order(
                args.mkl_local_thread_budgets,
                repetition=repetition - 1,
                seed=seed,
                block_key=f"outer_tasks={outer_tasks}:arena={arena}",
            )
            jobs.extend(
                {
                    "arena": arena,
                    "budget": budget,
                    "outer_tasks": outer_tasks,
                    "repetition": repetition,
                }
                for budget in budgets
            )
    return jobs


def summarize_timing(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[tuple[int, int, int], list[dict[str, Any]]] = {}
    for record in records:
        result = record["result"]
        key = (
            int(result["configured_arena_concurrency"]),
            int(result["configured_mkl_local_budget"]),
            int(result["outer_tasks"]),
        )
        groups.setdefault(key, []).append(record)

    summary = []
    for (arena, budget, outer_tasks), group in sorted(groups.items()):
        wall_values = [float(record["result"]["wall_seconds"]) for record in group]
        cpu_values = [
            float(record["result"]["process_cpu_seconds"]) for record in group
        ]
        wall_median, wall_mad = median_and_mad(wall_values)
        cpu_median, cpu_mad = median_and_mad(cpu_values)
        summary.append(
            {
                "configured_arena_concurrency": arena,
                "configured_mkl_local_budget": budget,
                "outer_tasks": outer_tasks,
                "repetitions": len(group),
                "median_wall_seconds": wall_median,
                "wall_seconds_mad": wall_mad,
                "median_process_cpu_seconds": cpu_median,
                "process_cpu_seconds_mad": cpu_mad,
                "observed_mkl_max_threads_min": min(
                    int(record["result"]["observed_mkl_max_threads_min"])
                    for record in group
                ),
                "observed_mkl_max_threads_max": max(
                    int(record["result"]["observed_mkl_max_threads_max"])
                    for record in group
                ),
            }
        )
    return summary


def optional_median(values: list[int | float | None]) -> float | None:
    present = [float(value) for value in values if value is not None]
    return statistics.median(present) if present else None


def summarize_profiles(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    complete = [record for record in records if "task_metrics" in record]
    groups: dict[tuple[int, int, int], list[dict[str, Any]]] = {}
    for record in complete:
        result = record["result"]
        key = (
            int(result["configured_arena_concurrency"]),
            int(result["configured_mkl_local_budget"]),
            int(result["outer_tasks"]),
        )
        groups.setdefault(key, []).append(record)

    summary = []
    for (arena, budget, outer_tasks), group in sorted(groups.items()):
        summary.append(
            {
                "configured_arena_concurrency": arena,
                "configured_mkl_local_budget": budget,
                "outer_tasks": outer_tasks,
                "repetitions": len(group),
                "median_probe_wall_seconds": optional_median(
                    [record["result"]["wall_seconds"] for record in group]
                ),
                "median_probe_process_cpu_seconds": optional_median(
                    [record["result"]["process_cpu_seconds"] for record in group]
                ),
                "median_total_thread_count": optional_median(
                    [record["vtune_summary"]["total_thread_count"] for record in group]
                ),
                "median_effective_cpu_count": optional_median(
                    [record["vtune_summary"]["effective_cpu_count"] for record in group]
                ),
                "median_spin_and_overhead_seconds": optional_median(
                    [
                        record["vtune_summary"]["spin_and_overhead_seconds"]
                        for record in group
                    ]
                ),
                "median_poor_utilization_wait_seconds": optional_median(
                    [
                        record["vtune_summary"]["poor_utilization_wait_seconds"]
                        for record in group
                    ]
                ),
                "median_tbb_parallel_for_task_count": optional_median(
                    [
                        record["task_metrics"]["tbb_parallel_for_task_count"]
                        for record in group
                    ]
                ),
                "median_tbb_tasks_per_process_gemm": optional_median(
                    [
                        record["task_metrics"]["tbb_tasks_per_process_gemm"]
                        for record in group
                    ]
                ),
            }
        )
    return summary


def parse_vtune_summary(text: str) -> dict[str, int | float | None]:
    patterns: dict[str, tuple[str, type[int] | type[float]]] = {
        "elapsed_seconds": (r"^Elapsed Time:\s+([0-9.]+)s", float),
        "effective_cpu_count": (
            r"^Effective CPU Utilization:.*\(([0-9.]+) out of",
            float,
        ),
        "total_thread_count": (r"^\s+Total Thread Count:\s+([0-9]+)", int),
        "poor_utilization_wait_seconds": (
            r"^\s+Wait Time with poor CPU Utilization:\s+([0-9.]+)s",
            float,
        ),
        "spin_and_overhead_seconds": (
            r"^\s+Spin and Overhead Time:\s+([0-9.]+)s",
            float,
        ),
    }
    parsed: dict[str, int | float | None] = {}
    for key, (pattern, conversion) in patterns.items():
        match = re.search(pattern, text, flags=re.MULTILINE)
        parsed[key] = conversion(match.group(1)) if match else None
    return parsed


def parse_task_report(path: Path) -> dict[str, int | float | None]:
    if not path.stat().st_size:
        return {
            "tbb_parallel_for_task_count": 0,
            "tbb_parallel_for_average_task_seconds": None,
            "tbb_parallel_for_task_seconds": None,
        }
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            if row.get("Task Type") != "tbb_parallel_for":
                continue
            return {
                "tbb_parallel_for_task_count": int(float(row["Task Count"])),
                "tbb_parallel_for_average_task_seconds": float(
                    row["Average Task Time"]
                ),
                "tbb_parallel_for_task_seconds": float(row["Task Time"]),
            }
    return {
        "tbb_parallel_for_task_count": 0,
        "tbb_parallel_for_average_task_seconds": None,
        "tbb_parallel_for_task_seconds": None,
    }


def write_results(path: Path, results: dict[str, Any]) -> None:
    path.write_text(json.dumps(results, indent=2) + "\n")


def main() -> int:
    args = parse_args()
    validate_args(args)
    probe = resolve_file(args.probe, "Probe executable")
    output = args.out.expanduser().resolve()
    if output.exists() and not args.dry_run:
        raise FileExistsError(f"Output path already exists: {output}")

    order_groups = [
        ("arenas", args.arena_concurrencies),
        ("mkl_local_thread_budgets", args.mkl_local_thread_budgets),
    ]
    timing_order = order_configuration(
        args.timing_repetitions,
        order_groups,
        allow_incomplete=args.allow_incomplete_order_cycle,
    )
    profile_order = (
        order_configuration(
            args.profile_repetitions,
            order_groups,
            allow_incomplete=args.allow_incomplete_order_cycle,
        )
        if args.collect_vtune
        else None
    )

    timing_jobs = make_jobs(args, args.outer_tasks, args.timing_repetitions, args.seed)
    profile_jobs = (
        make_jobs(
            args,
            args.profile_outer_tasks,
            args.profile_repetitions,
            args.seed + 1,
        )
        if args.collect_vtune
        else []
    )
    vtune = resolve_vtune(args.vtune) if args.collect_vtune else None

    if args.dry_run:
        for job in timing_jobs:
            print(
                shlex.join(
                    probe_command(
                        probe,
                        args,
                        job["arena"],
                        job["budget"],
                        job["outer_tasks"],
                    )
                )
            )
        if vtune is not None:
            for job in profile_jobs:
                name = case_name(
                    job["arena"],
                    job["budget"],
                    job["outer_tasks"],
                    job["repetition"],
                )
                command = [
                    *sudo_environment_prefix(args.sudo, os.environ),
                    str(vtune),
                    "-collect",
                    "threading",
                    "-result-dir",
                    str(output / "profiles" / f"vtune-{name}"),
                    "--",
                    *probe_command(
                        probe,
                        args,
                        job["arena"],
                        job["budget"],
                        job["outer_tasks"],
                    ),
                ]
                print(shlex.join(command))
        return 0

    environment = mkl_tbb_environment()
    linkage = verify_mkl_tbb_probe_linkage(probe, environment)
    output.mkdir(parents=True)
    host_preconditioning = precondition_host(args, workers=args.concurrency)
    timing_log_directory = output / "timing-logs"
    timing_log_directory.mkdir()

    results: dict[str, Any] = {
        "schema_version": 2,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "probe": str(probe),
        "vtune": str(vtune) if vtune is not None else None,
        "configuration": {
            "concurrency": args.concurrency,
            "arena_concurrency": (
                args.arena_concurrencies[0]
                if len(args.arena_concurrencies) == 1
                else None
            ),
            "arena_concurrencies": args.arena_concurrencies,
            "mkl_local_thread_budgets": args.mkl_local_thread_budgets,
            "outer_tasks": args.outer_tasks,
            "profile_outer_tasks": args.profile_outer_tasks,
            "matrix_n": args.matrix_n,
            "warmup_iterations": args.warmup_iterations,
            "profile_iterations": args.profile_iterations,
            "timing_repetitions": args.timing_repetitions,
            "profile_repetitions": args.profile_repetitions,
            "timing_order": timing_order,
            "profile_order": profile_order,
            "seed": args.seed,
            "collect_vtune": args.collect_vtune,
            "sudo": args.sudo,
        },
        "environment": {
            "MKL_THREADING_LAYER": "TBB",
            "MKL_NUM_THREADS": environment.get("MKL_NUM_THREADS"),
            "MKL_DYNAMIC": environment.get("MKL_DYNAMIC"),
        },
        "linkage": linkage,
        "host_preconditioning": host_preconditioning,
        "timing_records": [],
        "timing_summary": [],
        "profile_records": [],
        "profile_summary": [],
    }
    results_path = output / "budget-sweep.json"

    for index, job in enumerate(timing_jobs, start=1):
        block = (job["repetition"], job["outer_tasks"], job["arena"])
        guard_host_condition(
            args,
            host_preconditioning,
            label=(
                f"timing:r={block[0]}:outer_tasks={block[1]}:arena={block[2]}:"
                f"budget={job['budget']}"
            ),
        )
        name = case_name(
            job["arena"],
            job["budget"],
            job["outer_tasks"],
            job["repetition"],
        )
        command = probe_command(
            probe,
            args,
            job["arena"],
            job["budget"],
            job["outer_tasks"],
        )
        print(f"[timing {index}/{len(timing_jobs)}] {name}", flush=True)
        completed = subprocess.run(
            command,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
            env=environment,
        )
        log_path = timing_log_directory / f"{name}.log"
        log_path.write_text(completed.stdout)
        record: dict[str, Any] = {
            "case": name,
            "order_index": index,
            "repetition": job["repetition"],
            "command": command,
            "returncode": completed.returncode,
            "log": str(log_path),
        }
        results["timing_records"].append(record)
        if completed.returncode != 0:
            write_results(results_path, results)
            raise RuntimeError(f"Timing run failed for {name}; see {log_path}.")
        record["result"] = parse_key_value_marker(
            completed.stdout,
            RESULT_PREFIX,
            integer_fields=INTEGER_FIELDS,
            float_fields=FLOAT_FIELDS,
        )
        results["timing_summary"] = summarize_timing(results["timing_records"])
        write_results(results_path, results)

    if vtune is None:
        print(f"Results written to {results_path}")
        return 0

    profiles_directory = output / "profiles"
    profiles_directory.mkdir()
    for index, job in enumerate(profile_jobs, start=1):
        profile_block = (job["repetition"], job["outer_tasks"], job["arena"])
        guard_host_condition(
            args,
            host_preconditioning,
            label=(
                "profile:"
                f"r={profile_block[0]}:outer_tasks={profile_block[1]}:"
                f"arena={profile_block[2]}:budget={job['budget']}"
            ),
        )
        name = case_name(
            job["arena"],
            job["budget"],
            job["outer_tasks"],
            job["repetition"],
        )
        result_directory = profiles_directory / f"vtune-{name}"
        probe_args = probe_command(
            probe,
            args,
            job["arena"],
            job["budget"],
            job["outer_tasks"],
        )
        command = [
            *sudo_environment_prefix(args.sudo, environment),
            str(vtune),
            "-collect",
            "threading",
            "-result-dir",
            str(result_directory),
            "--",
            *probe_args,
        ]
        print(f"[profile {index}/{len(profile_jobs)}] {name}", flush=True)
        completed = subprocess.run(
            command,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
            env=environment,
        )
        if args.sudo and result_directory.exists():
            checked_output(
                [
                    "sudo",
                    "chown",
                    "-R",
                    f"{os.getuid()}:{os.getgid()}",
                    str(result_directory),
                ],
                environment,
            )
        log_path = profiles_directory / f"{name}.log"
        log_path.write_text(completed.stdout)
        profile_record: dict[str, Any] = {
            "case": name,
            "order_index": index,
            "repetition": job["repetition"],
            "command": command,
            "result_directory": str(result_directory),
            "returncode": completed.returncode,
            "log": str(log_path),
        }
        results["profile_records"].append(profile_record)
        if completed.returncode != 0:
            write_results(results_path, results)
            raise RuntimeError(f"VTune failed for {name}; see {log_path}.")

        reports = {
            "summary": [
                str(vtune),
                "-quiet",
                "-report",
                "summary",
                "-result-dir",
                str(result_directory),
                "-report-knob",
                "show-issues=false",
            ],
            "hotspots.csv": [
                str(vtune),
                "-quiet",
                "-report",
                "hotspots",
                "-result-dir",
                str(result_directory),
                "-format=csv",
                "-csv-delimiter=comma",
            ],
            "tasks.csv": [
                str(vtune),
                "-quiet",
                "-report",
                "hotspots",
                "-result-dir",
                str(result_directory),
                "-group-by",
                "task",
                "-format=csv",
                "-csv-delimiter=comma",
            ],
        }
        report_paths: dict[str, str] = {}
        for suffix, report_command in reports.items():
            report_path = profiles_directory / f"{name}.{suffix}"
            report_path.write_text(report_output(report_command, environment))
            report_paths[suffix] = str(report_path)
        profile_record["reports"] = report_paths
        profile_record["result"] = parse_key_value_marker(
            completed.stdout,
            RESULT_PREFIX,
            integer_fields=INTEGER_FIELDS,
            float_fields=FLOAT_FIELDS,
        )
        summary_text = Path(report_paths["summary"]).read_text()
        profile_record["vtune_summary"] = parse_vtune_summary(summary_text)
        task_metrics = parse_task_report(Path(report_paths["tasks.csv"]))
        profile_record["task_metrics"] = task_metrics
        if job["outer_tasks"] == 1:
            task_count = int(task_metrics["tbb_parallel_for_task_count"] or 0)
            process_calls = int(profile_record["result"]["process_gemm_calls"])
            profile_record["task_metrics"]["tbb_tasks_per_process_gemm"] = (
                task_count / process_calls
            )
        else:
            profile_record["task_metrics"]["tbb_tasks_per_process_gemm"] = None
            profile_record["task_metrics"]["task_count_note"] = (
                "Includes outer and oneMKL TBB tasks; do not interpret as internal "
                "oneMKL decomposition."
            )
        results["profile_summary"] = summarize_profiles(results["profile_records"])
        write_results(results_path, results)

    print(f"Results and profiles written to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
