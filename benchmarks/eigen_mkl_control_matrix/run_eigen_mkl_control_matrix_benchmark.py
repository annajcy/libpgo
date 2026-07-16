#!/usr/bin/env python3
"""Run the diagnostic 2x2 oneMKL-local-setting/TBB-arena experiment."""

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


POLICIES = (
    "DefaultArenaGlobal",
    "Local1ArenaGlobal",
    "DefaultArena1",
    "Local1Arena1",
)
WORKLOADS = ("EigenMklGemm", "NoBlas")
POLICY_FACTORS = {
    "DefaultArenaGlobal": (0, 0),
    "Local1ArenaGlobal": (1, 0),
    "DefaultArena1": (0, 1),
    "Local1Arena1": (1, 1),
}
TIME_SCALE = {"ns": 1e-9, "us": 1e-6, "ms": 1e-3, "s": 1.0}
CASE_PATTERN = re.compile(
    r"^EigenMklControlMatrix/"
    r"(EigenMklGemm|NoBlas)/"
    r"(DefaultArenaGlobal|Local1ArenaGlobal|DefaultArena1|Local1Arena1)"
    r"/c_(\d+)/tasks_(\d+)/n_(\d+)(?:/real_time)?$"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmark", type=Path)
    parser.add_argument(
        "--no-blas-benchmark",
        type=Path,
        help="Eigen-internal-GEMM executable (default: sibling no-BLAS target).",
    )
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=10)
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


def benchmark_environment() -> dict[str, str]:
    environment = os.environ.copy()
    environment["MKL_THREADING_LAYER"] = "TBB"
    return environment


def verify_linkage(executable: Path, require_dgemm: bool) -> dict[str, str]:
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
    if require_dgemm and not dgemm.search(undefined_symbols):
        raise RuntimeError("Benchmark does not expose a dynamic DGEMM reference.")

    return {
        "dependencies": dependencies,
        "undefined_symbols": undefined_symbols,
    }


def discover_cases(
    executable: Path, environment: dict[str, str], expected_workload: str
) -> dict[tuple[str, int, int, int], dict[str, str]]:
    output = checked_output([str(executable), "--benchmark_list_tests"], environment)
    discovered: dict[tuple[str, int, int, int], dict[str, str]] = {}
    for line in output.splitlines():
        name = line.strip()
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
        expected_local_one, expected_arena_one = POLICY_FACTORS[policy]
        expected_calls = int(row["iterations"]) * outer_tasks
        expected_inner_concurrency = 1 if expected_arena_one else concurrency
        expected = {
            "workload": WORKLOADS.index(workload),
            "uses_blas": 1 if workload == "EigenMklGemm" else 0,
            "configured_concurrency": concurrency,
            "effective_concurrency": concurrency,
            "outer_tasks": outer_tasks,
            "matrix_n": matrix_n,
            "uses_mkl_local_one": expected_local_one,
            "uses_arena_one": expected_arena_one,
            "outer_arena_concurrency": concurrency,
            "inner_arena_concurrency": expected_inner_concurrency,
            "body_calls": expected_calls,
            "restoration_mismatches": 0,
        }
        for counter_name, expected_value in expected.items():
            observed = integer_counter(row, counter_name)
            if observed != expected_value:
                raise RuntimeError(
                    f"{policy} reported {counter_name}={observed}; "
                    f"expected {expected_value}."
                )
        reported_min = integer_counter(row, "mkl_api_reported_min")
        reported_max = integer_counter(row, "mkl_api_reported_max")
        if reported_min <= 0 or reported_max < reported_min:
            raise RuntimeError(f"{policy} reported invalid MKL service-API state.")


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


def ratios(measurements: dict[str, dict[str, Any]]) -> dict[str, float]:
    times = {policy: float(measurements[policy]["wall_seconds"]) for policy in POLICIES}
    api_global = times["Local1ArenaGlobal"] / times["DefaultArenaGlobal"]
    arena_default = times["DefaultArena1"] / times["DefaultArenaGlobal"]
    api_arena_one = times["Local1Arena1"] / times["DefaultArena1"]
    arena_local_one = times["Local1Arena1"] / times["Local1ArenaGlobal"]
    return {
        "B_over_A_api_at_global_arena": api_global,
        "C_over_A_arena_at_default_api": arena_default,
        "D_over_C_api_at_arena_one": api_arena_one,
        "D_over_B_arena_at_local_one": arena_local_one,
        "D_over_A_combined": times["Local1Arena1"] / times["DefaultArenaGlobal"],
        "interaction_D_over_C_div_B_over_A": api_arena_one / api_global,
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
    for (workload, concurrency, outer_tasks, matrix_n), blocks in sorted(grouped.items()):
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
                "median_mkl_api_reported_range": {
                    policy: {
                        "min": statistics.median(
                            float(block["measurements"][policy]["mkl_api_reported_min"])
                            for block in blocks
                        ),
                        "max": statistics.median(
                            float(block["measurements"][policy]["mkl_api_reported_max"])
                            for block in blocks
                        ),
                    }
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
                raise RuntimeError(f"Probe markers are missing for {policy}, c={concurrency}.")
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
        "eigen_mkl_gemm": verify_linkage(executable, require_dgemm=True),
        "eigen_internal_gemm": verify_linkage(
            no_blas_executable, require_dgemm=False
        ),
    }
    cases = discover_cases(executable, environment, "EigenMklGemm")
    cases.update(discover_cases(no_blas_executable, environment, "NoBlas"))
    if args.case_limit:
        cases = dict(list(cases.items())[: args.case_limit])
    if not cases:
        raise SystemExit("No complete four-policy control-matrix cases were found.")

    probe: Path | None = None
    if not args.skip_mkl_verbose_probe:
        probe = (args.probe or executable.with_name("eigen_mkl_control_matrix_probe")).resolve()
        if not probe.exists():
            raise SystemExit(f"MKL_VERBOSE probe executable does not exist: {probe}")
        verify_linkage(probe, require_dgemm=True)

    print(f"Verified MKL-TBB linkage; matched {len(cases)} four-policy case(s).")
    for workload, concurrency, outer_tasks, matrix_n in cases:
        print(f"{workload}: c={concurrency} tasks={outer_tasks} n={matrix_n}")
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

    with tempfile.TemporaryDirectory(
        prefix="pgo-eigen-mkl-control-matrix-"
    ) as temp_dir:
        temporary = Path(temp_dir)
        for block_index, (repetition, key) in enumerate(schedule, start=1):
            policies = list(POLICIES)
            randomizer.shuffle(policies)
            workload, concurrency, outer_tasks, matrix_n = key
            print(
                f"[{block_index}/{len(schedule)}] repetition={repetition} "
                f"{workload}: c={concurrency} tasks={outer_tasks} n={matrix_n} "
                f"order={','.join(policies)}",
                flush=True,
            )
            measurements: dict[str, dict[str, Any]] = {}
            for policy in policies:
                measurements[policy] = run_one(
                    executable if workload == "EigenMklGemm" else no_blas_executable,
                    cases[key][policy],
                    temporary / f"{block_index}-{policy}.json",
                    args.min_time,
                    args.warmup_time,
                    environment,
                )

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

    verbose_probe = (
        [] if probe is None else run_mkl_verbose_probe(probe, cases, environment)
    )
    summary = summarize(records, args.seed, args.bootstrap_samples)
    payload = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "benchmark": str(executable),
        "no_blas_benchmark": str(no_blas_executable),
        "seed": args.seed,
        "repetitions": args.repetitions,
        "min_time": args.min_time,
        "warmup_time": args.warmup_time,
        "bootstrap_samples": args.bootstrap_samples,
        "thread_telemetry_source": "untimed_policy_warmup",
        "mkl_verbose_probe": verbose_probe,
        "environment": {"MKL_THREADING_LAYER": environment["MKL_THREADING_LAYER"]},
        "mkl_local_budget_note": (
            "Local1 isolates the per-OS-thread mkl_set_num_threads_local(1) "
            "budget from the independent one-participant task_arena factor."
        ),
        "factor_mapping": {
            "A": "DefaultArenaGlobal",
            "B": "Local1ArenaGlobal",
            "C": "DefaultArena1",
            "D": "Local1Arena1",
        },
        "linkage": linkage,
        "records": records,
        "summary": summary,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2) + "\n")

    print(
        "\nworkload       c tasks    n      A_ms      B_ms      C_ms      D_ms      B/A      C/A      D/C      D/B  interaction"
    )
    for row in summary:
        times = row["median_wall_seconds"]
        row_ratios = row["median_ratios"]
        print(
            f"{row['workload']:<12} {row['concurrency']:2d} {row['outer_tasks']:5d} {row['matrix_n']:4d}  "
            f"{times['DefaultArenaGlobal'] * 1e3:8.3f}  "
            f"{times['Local1ArenaGlobal'] * 1e3:8.3f}  "
            f"{times['DefaultArena1'] * 1e3:8.3f}  "
            f"{times['Local1Arena1'] * 1e3:8.3f}  "
            f"{row_ratios['B_over_A_api_at_global_arena']:7.3f}  "
            f"{row_ratios['C_over_A_arena_at_default_api']:7.3f}  "
            f"{row_ratios['D_over_C_api_at_arena_one']:7.3f}  "
            f"{row_ratios['D_over_B_arena_at_local_one']:7.3f}  "
            f"{row_ratios['interaction_D_over_C_div_B_over_A']:11.3f}"
        )
    print(f"\nWrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
