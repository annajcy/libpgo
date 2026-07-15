#!/usr/bin/env python3
"""Run randomized four-policy Eigen/oneMKL nested-parallel comparisons."""

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


POLICIES = ("PgoMulti", "PgoSingle", "TbbMulti", "TbbSingle")
TIME_SCALE = {"ns": 1e-9, "us": 1e-6, "ms": 1e-3, "s": 1.0}
CASE_PATTERN = re.compile(
    r"^NestedEigenMkl/(PgoMulti|PgoSingle|TbbMulti|TbbSingle)"
    r"/c_(\d+)/tasks_(\d+)/n_(\d+)(?:/real_time)?$"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmark", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=7)
    parser.add_argument("--min-time", default="0.05s")
    parser.add_argument("--warmup-time", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=20260714)
    parser.add_argument("--checksum-relative-tolerance", type=float, default=1e-10)
    parser.add_argument("--case-limit", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def checked_output(command: list[str], environment: dict[str, str] | None = None) -> str:
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

    if any(name in dependencies_lower for name in ("libiomp5", "libgomp", "libomp.so")):
        raise RuntimeError("Benchmark links an OpenMP runtime instead of a pure MKL-TBB stack.")

    dgemm = re.compile(r"(?:^|\s)_?(?:cblas_)?dgemm_?(?:@\S+)?(?:\s|$)", re.I | re.M)
    if not dgemm.search(undefined_symbols):
        raise RuntimeError("Benchmark does not expose a dynamic DGEMM reference.")

    return {
        "dependencies": dependencies,
        "undefined_symbols": undefined_symbols,
    }


def discover_cases(
    executable: Path, environment: dict[str, str]
) -> dict[tuple[int, int, int], dict[str, str]]:
    output = checked_output([str(executable), "--benchmark_list_tests"], environment)
    discovered: dict[tuple[int, int, int], dict[str, str]] = {}
    for line in output.splitlines():
        name = line.strip()
        match = CASE_PATTERN.fullmatch(name)
        if not match:
            continue
        policy, concurrency, outer_tasks, matrix_n = match.groups()
        key = (int(concurrency), int(outer_tasks), int(matrix_n))
        discovered.setdefault(key, {})[policy] = name

    return {
        key: cases
        for key, cases in sorted(discovered.items())
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


def counter(row: dict[str, Any], name: str) -> int:
    value = float(row.get(name, math.nan))
    if not math.isfinite(value):
        raise RuntimeError(f"Missing finite counter {name} in {row.get('name')}.")
    return round(value)


def validate_block(
    key: tuple[int, int, int],
    measurements: dict[str, dict[str, Any]],
    checksum_relative_tolerance: float,
) -> None:
    concurrency, outer_tasks, matrix_n = key
    checksums = [float(measurements[policy].get("checksum", math.nan)) for policy in POLICIES]
    if not all(math.isfinite(value) for value in checksums):
        raise RuntimeError(f"Missing finite checksum for c={concurrency}, tasks={outer_tasks}, n={matrix_n}.")
    tolerance = max(1e-12, checksum_relative_tolerance * max(abs(value) for value in checksums))
    if max(checksums) - min(checksums) > tolerance:
        raise RuntimeError(f"Checksum mismatch for c={concurrency}, tasks={outer_tasks}, n={matrix_n}.")

    for policy, row in measurements.items():
        if counter(row, "configured_concurrency") != concurrency:
            raise RuntimeError(f"{policy} did not receive concurrency={concurrency}.")
        if counter(row, "effective_concurrency") != concurrency:
            raise RuntimeError(f"{policy} did not establish global concurrency={concurrency}.")
        expected_calls = int(row["iterations"]) * outer_tasks
        if counter(row, "body_calls") != expected_calls:
            raise RuntimeError(f"{policy} executed an unexpected number of outer bodies.")

    if counter(measurements["PgoMulti"], "outer_arena_concurrency") != concurrency:
        raise RuntimeError("PgoMulti did not use the global-aligned arena.")
    if counter(measurements["PgoSingle"], "outer_arena_concurrency") != concurrency:
        raise RuntimeError("PgoSingle did not use the global-aligned arena.")
    if counter(measurements["PgoMulti"], "inner_arena_concurrency") != concurrency:
        raise RuntimeError("PgoMulti unexpectedly bounded the Eigen/MKL call.")
    if counter(measurements["TbbMulti"], "inner_arena_concurrency") != counter(
        measurements["TbbMulti"], "outer_arena_concurrency"
    ):
        raise RuntimeError("TbbMulti did not leave the Eigen/MKL call in the default arena.")
    for policy in ("PgoSingle", "TbbSingle"):
        if counter(measurements[policy], "inner_arena_concurrency") != 1:
            raise RuntimeError(f"{policy} did not put Eigen/MKL in a one-participant arena.")


def summarize(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[tuple[int, int, int], list[dict[str, Any]]] = {}
    for record in records:
        key = (record["concurrency"], record["outer_tasks"], record["matrix_n"])
        grouped.setdefault(key, []).append(record)

    summary: list[dict[str, Any]] = []
    for (concurrency, outer_tasks, matrix_n), blocks in sorted(grouped.items()):
        ratios = {
            name: [float(block["ratios"][name]) for block in blocks]
            for name in blocks[0]["ratios"]
        }
        summary.append(
            {
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
                    name: statistics.median(values) for name, values in ratios.items()
                },
                "median_extra_threads": {
                    policy: statistics.median(
                        float(block["cold_probes"][policy]["extra_threads"])
                        for block in blocks
                    )
                    for policy in POLICIES
                },
            }
        )
    return summary


def main() -> int:
    args = parse_args()
    if args.repetitions < 1 or args.warmup_time < 0:
        raise SystemExit("--repetitions must be positive and --warmup-time non-negative.")
    if args.case_limit < 0:
        raise SystemExit("--case-limit must be non-negative.")

    executable = args.benchmark.resolve()
    if not executable.exists():
        raise SystemExit(f"Benchmark executable does not exist: {executable}")

    environment = benchmark_environment()
    linkage = verify_linkage(executable)
    cases = discover_cases(executable, environment)
    if args.case_limit:
        cases = dict(list(cases.items())[: args.case_limit])
    if not cases:
        raise SystemExit("No complete four-policy benchmark blocks were found.")

    print(f"Verified MKL-TBB linkage; matched {len(cases)} four-policy block(s).")
    for concurrency, outer_tasks, matrix_n in cases:
        print(f"c={concurrency} tasks={outer_tasks} n={matrix_n}")
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

    with tempfile.TemporaryDirectory(prefix="pgo-eigen-mkl-nested-") as temp_dir:
        temporary = Path(temp_dir)
        for block_index, (repetition, key) in enumerate(schedule, start=1):
            policies = list(POLICIES)
            randomizer.shuffle(policies)
            concurrency, outer_tasks, matrix_n = key
            print(
                f"[{block_index}/{len(schedule)}] repetition={repetition} "
                f"c={concurrency} tasks={outer_tasks} n={matrix_n} "
                f"order={','.join(policies)}",
                flush=True,
            )
            measurements: dict[str, dict[str, Any]] = {}
            cold_probes: dict[str, dict[str, Any]] = {}
            for policy in policies:
                cold_probes[policy] = run_one(
                    executable,
                    cases[key][policy],
                    temporary / f"{block_index}-{policy}-cold.json",
                    "1x",
                    0.0,
                    environment,
                )
                measurements[policy] = run_one(
                    executable,
                    cases[key][policy],
                    temporary / f"{block_index}-{policy}-steady.json",
                    args.min_time,
                    args.warmup_time,
                    environment,
                )

            validate_block(key, cold_probes, args.checksum_relative_tolerance)
            validate_block(key, measurements, args.checksum_relative_tolerance)
            times = {policy: float(measurements[policy]["wall_seconds"]) for policy in POLICIES}
            records.append(
                {
                    "repetition": repetition,
                    "concurrency": concurrency,
                    "outer_tasks": outer_tasks,
                    "matrix_n": matrix_n,
                    "order": policies,
                    "measurements": measurements,
                    "cold_probes": cold_probes,
                    "ratios": {
                        "TbbMulti/PgoMulti": times["TbbMulti"] / times["PgoMulti"],
                        "TbbSingle/PgoSingle": times["TbbSingle"] / times["PgoSingle"],
                        "PgoMulti/PgoSingle": times["PgoMulti"] / times["PgoSingle"],
                        "TbbMulti/TbbSingle": times["TbbMulti"] / times["TbbSingle"],
                        "arena_interaction":
                            (times["TbbMulti"] / times["PgoMulti"])
                            / (times["TbbSingle"] / times["PgoSingle"]),
                    },
                }
            )

    summary = summarize(records)
    payload = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "benchmark": str(executable),
        "seed": args.seed,
        "repetitions": args.repetitions,
        "min_time": args.min_time,
        "warmup_time": args.warmup_time,
        "thread_telemetry_source": "cold_probes",
        "environment": {"MKL_THREADING_LAYER": environment["MKL_THREADING_LAYER"]},
        "linkage": linkage,
        "records": records,
        "summary": summary,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2) + "\n")

    print("\nc tasks    n  TbbMulti/PgoMulti  PgoMulti/PgoSingle  TbbMulti/TbbSingle")
    for row in summary:
        ratios = row["median_ratios"]
        print(
            f"{row['concurrency']:2d} {row['outer_tasks']:5d} {row['matrix_n']:4d}  "
            f"{ratios['TbbMulti/PgoMulti']:17.4f}  "
            f"{ratios['PgoMulti/PgoSingle']:20.4f}  "
            f"{ratios['TbbMulti/TbbSingle']:20.4f}"
        )
    print(f"\nWrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
