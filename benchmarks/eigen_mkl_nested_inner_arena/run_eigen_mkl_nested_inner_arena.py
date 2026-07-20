#!/usr/bin/env python3
"""Compare per-task nested arenas with an explicit oneMKL local budget of one."""

from __future__ import annotations

import argparse
import csv
import math
import os
import random
import shlex
import subprocess
import sys
from collections.abc import Mapping
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
from benchmark_support.validation import require_positive  # noqa: E402


POLICIES = (
    "OuterDefault",
    "OuterLocal1",
    "InnerArena1Default",
    "InnerArena1Local1",
)
RESULT_PREFIX = "PGO_MKL_NESTED_INNER_ARENA_RESULT"
STRING_FIELDS = frozenset({"policy"})
INTEGER_FIELDS = frozenset({
    "configured_global_concurrency",
    "effective_global_concurrency",
    "process_default_mkl_max_threads",
    "configured_outer_arena_concurrency",
    "configured_outer_mkl_local_budget",
    "uses_inner_arena",
    "configured_inner_arena_concurrency",
    "configured_inner_mkl_local_budget",
    "observed_outer_arena_concurrency_min",
    "observed_outer_arena_concurrency_max",
    "observed_inner_arena_concurrency_min",
    "observed_inner_arena_concurrency_max",
    "observed_mkl_max_threads_min",
    "observed_mkl_max_threads_max",
    "outer_active_peak",
    "outer_tasks",
    "matrix_n",
    "warmup_iterations",
    "profile_iterations",
    "measured_gemm_calls",
    "process_gemm_calls",
    "baseline_threads",
    "peak_threads",
    "extra_threads",
})
FLOAT_FIELDS = frozenset({"wall_seconds", "process_cpu_seconds", "checksum"})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("probe", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--concurrency", type=int, default=8)
    parser.add_argument(
        "--outer-tasks",
        type=int,
        nargs="+",
        default=[8, 32],
        help="Outer-task counts used for randomized timing runs.",
    )
    parser.add_argument(
        "--profile-outer-tasks",
        type=int,
        nargs="+",
        default=[8, 32],
        help="Outer-task counts collected when --collect-vtune is set.",
    )
    parser.add_argument("--matrix-n", type=int, default=1024)
    parser.add_argument("--warmup-iterations", type=int, default=3)
    parser.add_argument("--profile-iterations", type=int, default=50)
    parser.add_argument("--timing-repetitions", type=int, default=8)
    parser.add_argument("--seed", type=int, default=20260717)
    parser.add_argument("--checksum-relative-tolerance", type=float, default=1e-10)
    parser.add_argument("--collect-vtune", action="store_true")
    parser.add_argument("--vtune", type=Path)
    parser.add_argument(
        "--sudo",
        action="store_true",
        help="Run VTune through sudo and restore result-directory ownership.",
    )
    parser.add_argument("--dry-run", action="store_true")
    add_benchmark_harness_arguments(parser)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    require_positive(args.concurrency, "--concurrency")
    require_positive(args.matrix_n, "--matrix-n")
    require_positive(args.profile_iterations, "--profile-iterations")
    require_positive(args.timing_repetitions, "--timing-repetitions")
    if args.warmup_iterations < 0:
        raise ValueError("--warmup-iterations must be nonnegative.")
    if args.checksum_relative_tolerance < 0:
        raise ValueError("--checksum-relative-tolerance must be nonnegative.")
    for value in args.outer_tasks:
        require_positive(value, "--outer-tasks")
    for value in args.profile_outer_tasks:
        require_positive(value, "--profile-outer-tasks")
    if len(set(args.outer_tasks)) != len(args.outer_tasks):
        raise ValueError("--outer-tasks must not contain duplicates.")
    if len(set(args.profile_outer_tasks)) != len(args.profile_outer_tasks):
        raise ValueError("--profile-outer-tasks must not contain duplicates.")


def probe_command(
    probe: Path,
    args: argparse.Namespace,
    policy: str,
    outer_tasks: int,
    *,
    warmup_iterations: int | None = None,
    profile_iterations: int | None = None,
) -> list[str]:
    return [
        str(probe),
        f"--policy={policy}",
        f"--concurrency={args.concurrency}",
        f"--outer-tasks={outer_tasks}",
        f"--matrix-n={args.matrix_n}",
        "--warmup-iterations="
        f"{args.warmup_iterations if warmup_iterations is None else warmup_iterations}",
        "--profile-iterations="
        f"{args.profile_iterations if profile_iterations is None else profile_iterations}",
    ]


def make_timing_jobs(args: argparse.Namespace) -> list[dict[str, Any]]:
    # Keep each repetition as a temporal block so ratios paired by repetition
    # remain meaningful, while randomizing policy and task order within it.
    randomizer = random.Random(args.seed)
    blocks = [
        (repetition, outer_tasks)
        for repetition in range(1, args.timing_repetitions + 1)
        for outer_tasks in args.outer_tasks
    ]
    randomizer.shuffle(blocks)
    jobs: list[dict[str, Any]] = []
    for repetition, outer_tasks in blocks:
        policies = balanced_order(
            POLICIES,
            repetition=repetition - 1,
            seed=args.seed,
            block_key=f"outer_tasks={outer_tasks}",
        )
        jobs.extend(
            {
                "policy": policy,
                "outer_tasks": outer_tasks,
                "repetition": repetition,
            }
            for policy in policies
        )
    return jobs


def case_name(policy: str, outer_tasks: int, repetition: int) -> str:
    return f"{policy.lower()}-tasks{outer_tasks}-r{repetition}"


def paired_ratios(
    records: list[dict[str, Any]], numerator: str, denominator: str, outer_tasks: int
) -> list[float]:
    by_key = {
        (
            record["policy"],
            int(record["outer_tasks"]),
            int(record["repetition"]),
        ): float(record["result"]["wall_seconds"])
        for record in records
    }
    ratios = []
    repetitions = sorted(
        int(record["repetition"])
        for record in records
        if record["policy"] == denominator and int(record["outer_tasks"]) == outer_tasks
    )
    for repetition in repetitions:
        numerator_value = by_key[(numerator, outer_tasks, repetition)]
        denominator_value = by_key[(denominator, outer_tasks, repetition)]
        ratios.append(numerator_value / denominator_value)
    return ratios


def summarize_timing(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    summary = []
    for outer_tasks in sorted({int(record["outer_tasks"]) for record in records}):
        for policy in POLICIES:
            group = [
                record
                for record in records
                if record["policy"] == policy
                and int(record["outer_tasks"]) == outer_tasks
            ]
            wall_values = [float(record["result"]["wall_seconds"]) for record in group]
            cpu_values = [
                float(record["result"]["process_cpu_seconds"]) for record in group
            ]
            throughput_values = [
                int(record["result"]["measured_gemm_calls"])
                / float(record["result"]["wall_seconds"])
                for record in group
            ]
            ratio_values = paired_ratios(records, policy, "OuterLocal1", outer_tasks)
            wall_median, wall_mad = median_and_mad(wall_values)
            cpu_median, cpu_mad = median_and_mad(cpu_values)
            throughput_median, throughput_mad = median_and_mad(throughput_values)
            ratio_median, ratio_mad = median_and_mad(ratio_values)
            summary.append(
                {
                    "policy": policy,
                    "outer_tasks": outer_tasks,
                    "samples": len(group),
                    "wall_seconds_median": wall_median,
                    "wall_seconds_mad": wall_mad,
                    "process_cpu_seconds_median": cpu_median,
                    "process_cpu_seconds_mad": cpu_mad,
                    "gemm_per_second_median": throughput_median,
                    "gemm_per_second_mad": throughput_mad,
                    "paired_wall_ratio_to_outer_local1_median": ratio_median,
                    "paired_wall_ratio_to_outer_local1_mad": ratio_mad,
                    "outer_active_peak_values": sorted(
                        {int(record["result"]["outer_active_peak"]) for record in group}
                    ),
                    "observed_inner_arena_concurrency_values": sorted(
                        {
                            int(
                                record["result"]["observed_inner_arena_concurrency_max"]
                            )
                            for record in group
                        }
                    ),
                    "observed_mkl_max_threads_values": sorted(
                        {
                            int(record["result"]["observed_mkl_max_threads_max"])
                            for record in group
                        }
                    ),
                }
            )
    return summary


def build_comparisons(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    comparisons = (
        (
            "candidate_vs_recommended",
            "InnerArena1Default",
            "OuterLocal1",
        ),
        (
            "mkl_budget_effect_at_fixed_inner_topology",
            "InnerArena1Default",
            "InnerArena1Local1",
        ),
        (
            "nested_arena_overhead_with_budget1",
            "InnerArena1Local1",
            "OuterLocal1",
        ),
    )
    result = []
    for outer_tasks in sorted({int(record["outer_tasks"]) for record in records}):
        for name, numerator, denominator in comparisons:
            ratios = paired_ratios(records, numerator, denominator, outer_tasks)
            median, mad = median_and_mad(ratios)
            result.append(
                {
                    "name": name,
                    "outer_tasks": outer_tasks,
                    "numerator": numerator,
                    "denominator": denominator,
                    "paired_wall_ratio_median": median,
                    "paired_wall_ratio_mad": mad,
                    "paired_wall_ratios": ratios,
                }
            )
    return result


def validate_results(
    records: list[dict[str, Any]], args: argparse.Namespace
) -> dict[str, Any]:
    expected_calls = args.profile_iterations
    checksum_by_tasks: dict[int, float] = {}
    maximum_checksum_relative_error = 0.0

    for record in records:
        result = record["result"]
        policy = str(record["policy"])
        outer_tasks = int(record["outer_tasks"])
        expected_measured_calls = expected_calls * outer_tasks
        if result["policy"] != policy:
            raise RuntimeError(f"Policy mismatch in {record['log']}.")
        if int(result["outer_tasks"]) != outer_tasks:
            raise RuntimeError(f"Outer-task mismatch in {record['log']}.")
        if int(result["measured_gemm_calls"]) != expected_measured_calls:
            raise RuntimeError(f"Measured GEMM count mismatch in {record['log']}.")
        checksum = float(result["checksum"])
        if not math.isfinite(checksum):
            raise RuntimeError(f"Non-finite checksum in {record['log']}.")
        reference = checksum_by_tasks.setdefault(outer_tasks, checksum)
        relative_error = abs(checksum - reference) / max(1.0, abs(reference))
        maximum_checksum_relative_error = max(
            maximum_checksum_relative_error, relative_error
        )
        if relative_error > args.checksum_relative_tolerance:
            raise RuntimeError(
                f"Checksum mismatch for outer_tasks={outer_tasks}: "
                f"{relative_error:g} > {args.checksum_relative_tolerance:g}."
            )

        uses_inner = policy.startswith("InnerArena1")
        expected_inner_concurrency = 1 if uses_inner else 0
        if int(result["uses_inner_arena"]) != int(uses_inner):
            raise RuntimeError(f"Inner-arena policy mismatch in {record['log']}.")
        if (
            int(result["observed_inner_arena_concurrency_max"])
            != expected_inner_concurrency
        ):
            raise RuntimeError(f"Inner-arena observation mismatch in {record['log']}.")

    return {
        "checksum_relative_tolerance": args.checksum_relative_tolerance,
        "maximum_checksum_relative_error": maximum_checksum_relative_error,
        "checksums_by_outer_tasks": checksum_by_tasks,
    }


def write_timing_csv(path: Path, records: list[dict[str, Any]]) -> None:
    result_fields = [
        "wall_seconds",
        "process_cpu_seconds",
        "outer_active_peak",
        "observed_outer_arena_concurrency_max",
        "observed_inner_arena_concurrency_max",
        "observed_mkl_max_threads_min",
        "observed_mkl_max_threads_max",
        "baseline_threads",
        "peak_threads",
        "extra_threads",
        "checksum",
    ]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["policy", "outer_tasks", "repetition", *result_fields],
        )
        writer.writeheader()
        for record in records:
            writer.writerow(
                {
                    "policy": record["policy"],
                    "outer_tasks": record["outer_tasks"],
                    "repetition": record["repetition"],
                    **{field: record["result"][field] for field in result_fields},
                }
            )


def sudo_environment_prefix(
    use_sudo: bool, environment: Mapping[str, str]
) -> list[str]:
    if not use_sudo:
        return []
    assignments = ["MKL_THREADING_LAYER=TBB"]
    for name in ("LD_LIBRARY_PATH", "PATH"):
        value = environment.get(name)
        if value:
            assignments.append(f"{name}={value}")
    return ["sudo", "env", *assignments]


def collect_vtune_profiles(
    probe: Path,
    vtune: Path,
    output: Path,
    args: argparse.Namespace,
    environment: dict[str, str],
    manifest: dict[str, Any],
    artifact: JsonArtifact,
    completed_offset: int,
) -> None:
    profile_root = output / "vtune"
    profile_root.mkdir()
    manifest["vtune_runs"] = []

    profile_index = 0
    for outer_tasks in args.profile_outer_tasks:
        for policy in POLICIES:
            profile_index += 1
            case = f"{policy.lower()}-tasks{outer_tasks}"
            label = f"profile:outer_tasks={outer_tasks}:policy={policy}"
            artifact.set_active(label)
            result_directory = profile_root / f"result-{case}"
            command = [
                *sudo_environment_prefix(args.sudo, environment),
                str(vtune),
                "-collect",
                "threading",
                "-result-dir",
                str(result_directory),
                "--",
                *probe_command(
                    probe,
                    args,
                    policy,
                    outer_tasks,
                ),
            ]
            with artifact.capture_failures(lambda: manifest):
                guard_host_condition(args, manifest["host_preconditioning"], label=label)
                print(f"Profiling {policy}, outer_tasks={outer_tasks}...", flush=True)
                result = subprocess.run(
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
                log_path = profile_root / f"{case}.log"
                log_path.write_text(result.stdout)
                run: dict[str, Any] = {
                    "policy": policy,
                    "outer_tasks": outer_tasks,
                    "command": command,
                    "returncode": result.returncode,
                    "result_directory": str(result_directory),
                    "log": str(log_path),
                    "reports": {},
                }
                manifest["vtune_runs"].append(run)
                if result.returncode != 0:
                    raise RuntimeError(f"VTune failed for {case}; see {log_path}.")
                run["result"] = parse_key_value_marker(
                    result.stdout,
                    RESULT_PREFIX,
                    string_fields=STRING_FIELDS,
                    integer_fields=INTEGER_FIELDS,
                    float_fields=FLOAT_FIELDS,
                )

                reports = {
                "summary.txt": [
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
                for suffix, report_command in reports.items():
                    report_path = profile_root / f"{case}.{suffix}"
                    report_path.write_text(report_output(report_command, environment))
                    run["reports"][suffix] = str(report_path)
            artifact.checkpoint(
                manifest, completed_units=completed_offset + profile_index
            )


def main() -> int:
    args = parse_args()
    validate_args(args)
    probe = resolve_file(args.probe, "Probe executable")
    output = args.out.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"Output path already exists: {output}")
    environment = mkl_tbb_environment()
    timing_jobs = make_timing_jobs(args)
    timing_order = order_configuration(
        args.timing_repetitions,
        [("policies", POLICIES)],
        allow_incomplete=args.allow_incomplete_order_cycle,
    )
    vtune = resolve_vtune(args.vtune) if args.collect_vtune else None

    if args.dry_run:
        for job in timing_jobs:
            print(
                shlex.join(
                    probe_command(probe, args, job["policy"], int(job["outer_tasks"]))
                )
            )
        if vtune is not None:
            for outer_tasks in args.profile_outer_tasks:
                for policy in POLICIES:
                    result_directory = (
                        output
                        / "vtune"
                        / (f"result-{policy.lower()}-tasks{outer_tasks}")
                    )
                    print(
                        shlex.join(
                            [
                                *sudo_environment_prefix(args.sudo, environment),
                                str(vtune),
                                "-collect",
                                "threading",
                                "-result-dir",
                                str(result_directory),
                                "--",
                                *probe_command(
                                    probe,
                                    args,
                                    policy,
                                    outer_tasks,
                                ),
                            ]
                        )
                    )
        return 0

    linkage = verify_mkl_tbb_probe_linkage(probe, environment)
    output.mkdir(parents=True)
    host_preconditioning = precondition_host(args, workers=args.concurrency)
    logs = output / "logs"
    logs.mkdir()
    manifest: dict[str, Any] = {
        "runner": runner_manifest(Path(__file__)),
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "probe": str(probe),
        "configuration": {
            "policies": POLICIES,
            "concurrency": args.concurrency,
            "outer_tasks": args.outer_tasks,
            "profile_outer_tasks": args.profile_outer_tasks,
            "matrix_n": args.matrix_n,
            "warmup_iterations": args.warmup_iterations,
            "profile_iterations": args.profile_iterations,
            "timing_repetitions": args.timing_repetitions,
            "timing_order": timing_order,
            "seed": args.seed,
            "collect_vtune": args.collect_vtune,
            "sudo": args.sudo,
        },
        "environment": {
            "MKL_THREADING_LAYER": "TBB",
            "inherited_MKL_NUM_THREADS": os.environ.get("MKL_NUM_THREADS"),
            "inherited_MKL_DYNAMIC": os.environ.get("MKL_DYNAMIC"),
        },
        "linkage": linkage,
        "host_preconditioning": host_preconditioning,
        "timing_runs": [],
    }
    profile_count = (
        len(args.profile_outer_tasks) * len(POLICIES) if vtune is not None else 0
    )
    artifact = JsonArtifact(
        output / "nested-inner-arena.json",
        scheduled_units=len(timing_jobs) + profile_count,
    )

    for index, job in enumerate(timing_jobs, start=1):
        policy = str(job["policy"])
        outer_tasks = int(job["outer_tasks"])
        repetition = int(job["repetition"])
        label = f"timing:r={repetition}:outer_tasks={outer_tasks}:policy={policy}"
        artifact.set_active(label)
        command = probe_command(probe, args, policy, outer_tasks)
        print(
            f"[{index}/{len(timing_jobs)}] {policy}, "
            f"outer_tasks={outer_tasks}, repetition={repetition}",
            flush=True,
        )
        with artifact.capture_failures(lambda: manifest):
            guard_host_condition(args, host_preconditioning, label=label)
            stdout = checked_output(command, environment)
            log_path = logs / f"{case_name(policy, outer_tasks, repetition)}.log"
            log_path.write_text(stdout)
            manifest["timing_runs"].append(
                {
                    **job,
                    "command": command,
                    "log": str(log_path),
                    "result": parse_key_value_marker(
                        stdout,
                        RESULT_PREFIX,
                        string_fields=STRING_FIELDS,
                        integer_fields=INTEGER_FIELDS,
                        float_fields=FLOAT_FIELDS,
                    ),
                }
            )
        artifact.checkpoint(manifest, completed_units=index)

    with artifact.capture_failures(lambda: manifest):
        validation = validate_results(manifest["timing_runs"], args)
        manifest["validation"] = validation
        manifest["timing_summary"] = summarize_timing(manifest["timing_runs"])
        manifest["comparisons"] = build_comparisons(manifest["timing_runs"])
        write_timing_csv(output / "timing.csv", manifest["timing_runs"])

    if vtune is not None:
        manifest["vtune"] = str(vtune)
        collect_vtune_profiles(
            probe,
            vtune,
            output,
            args,
            environment,
            manifest,
            artifact,
            len(timing_jobs),
        )

    artifact.complete(manifest)
    print(f"Results written to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
