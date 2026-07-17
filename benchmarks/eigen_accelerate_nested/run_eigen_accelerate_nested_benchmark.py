#!/usr/bin/env python3
"""Compare SINGLE and MULTI Accelerate GEMM in identical executor arenas."""

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


BENCHMARKS_ROOT = Path(__file__).resolve().parents[1]
if str(BENCHMARKS_ROOT) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS_ROOT))

from host_preconditioning import (  # noqa: E402
    add_host_preconditioning_arguments,
    balanced_order,
    guard_host_condition,
    precondition_host,
)


POLICIES = ("ExecutorSingle", "ExecutorMulti")
TIME_SCALE = {"ns": 1e-9, "us": 1e-6, "ms": 1e-3, "s": 1.0}
CASE_PATTERN = re.compile(
    r"^NestedEigenAccelerate/(ExecutorSingle|ExecutorMulti)"
    r"/c_(\d+)/tasks_(\d+)/n_(\d+)(?:/real_time)?$"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmark", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=5)
    parser.add_argument("--min-time", default="0.05s")
    parser.add_argument("--warmup-time", type=float, default=0.02)
    parser.add_argument("--seed", type=int, default=20260714)
    parser.add_argument("--checksum-relative-tolerance", type=float, default=1e-10)
    parser.add_argument("--concurrency", type=int, action="append")
    parser.add_argument("--matrix-n", type=int, action="append")
    parser.add_argument("--outer-tasks", type=int, action="append")
    parser.add_argument("--case-limit", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true")
    add_host_preconditioning_arguments(parser)
    return parser.parse_args()


def checked_output(command: list[str]) -> str:
    result = subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(f"{' '.join(command)}\n{result.stdout}")
    return result.stdout


def verify_linkage(executable: Path) -> dict[str, str]:
    dependencies = checked_output(["otool", "-L", str(executable)])
    undefined_symbols = checked_output(["nm", "-u", str(executable)])
    if "/System/Library/Frameworks/Accelerate.framework" not in dependencies:
        raise RuntimeError("Benchmark does not link system Accelerate.")
    dgemm = re.compile(r"(?:^|\s)_?(?:cblas_)?dgemm_?(?:\s|$)", re.I | re.M)
    if not dgemm.search(undefined_symbols):
        raise RuntimeError("Benchmark does not expose a dynamic DGEMM reference.")
    return {
        "dependencies": dependencies,
        "undefined_symbols": undefined_symbols,
    }


def discover_cases(executable: Path) -> dict[tuple[int, int, int], dict[str, str]]:
    output = checked_output([str(executable), "--benchmark_list_tests"])
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
        key: variants
        for key, variants in sorted(discovered.items())
        if set(variants) == set(POLICIES)
    }


def filter_cases(
    cases: dict[tuple[int, int, int], dict[str, str]], args: argparse.Namespace
) -> dict[tuple[int, int, int], dict[str, str]]:
    concurrency = set(args.concurrency or [])
    matrix_n = set(args.matrix_n or [])
    outer_tasks = set(args.outer_tasks or [])
    selected = {
        key: variants
        for key, variants in cases.items()
        if (not concurrency or key[0] in concurrency)
        and (not outer_tasks or key[1] in outer_tasks)
        and (not matrix_n or key[2] in matrix_n)
    }
    if args.case_limit:
        selected = dict(list(selected.items())[: args.case_limit])
    return selected


def run_one(
    executable: Path,
    name: str,
    output: Path,
    min_time: str,
    warmup_time: float,
) -> dict[str, Any]:
    exact_filter = f"^{re.escape(name)}$"
    checked_output(
        [
            str(executable),
            f"--benchmark_filter={exact_filter}",
            f"--benchmark_min_time={min_time}",
            f"--benchmark_min_warmup_time={warmup_time:g}",
            "--benchmark_repetitions=1",
            f"--benchmark_out={output}",
            "--benchmark_out_format=json",
        ]
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
    checksums = [
        float(measurements[policy].get("checksum", math.nan)) for policy in POLICIES
    ]
    if not all(math.isfinite(value) for value in checksums):
        raise RuntimeError(
            f"Missing checksum for c={concurrency}, tasks={outer_tasks}, n={matrix_n}."
        )
    tolerance = max(
        1e-12, checksum_relative_tolerance * max(abs(value) for value in checksums)
    )
    if max(checksums) - min(checksums) > tolerance:
        raise RuntimeError(
            f"Checksum mismatch for c={concurrency}, tasks={outer_tasks}, n={matrix_n}."
        )

    for policy, row in measurements.items():
        if counter(row, "effective_concurrency") != concurrency:
            raise RuntimeError(f"{policy} did not establish concurrency={concurrency}.")
        if counter(row, "arena_concurrency") != concurrency:
            raise RuntimeError(
                f"{policy} did not execute in its configured executor arena."
            )
        expected_calls = int(row["iterations"]) * outer_tasks
        if counter(row, "body_calls") != expected_calls:
            raise RuntimeError(f"{policy} executed an unexpected number of bodies.")
        if counter(row, "other_mode_calls") != 0:
            raise RuntimeError(
                f"{policy} observed an unknown Accelerate threading mode."
            )

    single = measurements["ExecutorSingle"]
    multi = measurements["ExecutorMulti"]
    if counter(single, "single_mode_calls") != counter(single, "body_calls"):
        raise RuntimeError("ExecutorSingle did not observe SINGLE in every body.")
    if counter(single, "multi_mode_calls") != 0:
        raise RuntimeError("ExecutorSingle unexpectedly observed MULTI.")
    if counter(multi, "multi_mode_calls") != counter(multi, "body_calls"):
        raise RuntimeError("ExecutorMulti did not observe MULTI in every body.")
    if counter(multi, "single_mode_calls") != 0:
        raise RuntimeError("ExecutorMulti unexpectedly observed SINGLE.")


def summarize(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[tuple[int, int, int], list[dict[str, Any]]] = {}
    for record in records:
        key = (record["concurrency"], record["outer_tasks"], record["matrix_n"])
        grouped.setdefault(key, []).append(record)

    summary: list[dict[str, Any]] = []
    for (concurrency, outer_tasks, matrix_n), blocks in sorted(grouped.items()):
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
                "median_extra_threads": {
                    policy: statistics.median(
                        float(block["cold_probes"][policy]["extra_threads"])
                        for block in blocks
                    )
                    for policy in POLICIES
                },
                "median_multi_over_single": statistics.median(
                    float(block["multi_over_single"]) for block in blocks
                ),
                "median_extra_thread_delta": statistics.median(
                    float(block["extra_thread_delta"]) for block in blocks
                ),
                "median_cold_multi_over_single": statistics.median(
                    float(block["cold_multi_over_single"]) for block in blocks
                ),
            }
        )
    return summary


def main() -> int:
    args = parse_args()
    if args.repetitions < 1 or args.warmup_time < 0 or args.case_limit < 0:
        raise SystemExit("Invalid repetitions, warm-up, or case-limit value.")

    executable = args.benchmark.resolve()
    if not executable.exists():
        raise SystemExit(f"Benchmark executable does not exist: {executable}")

    linkage = verify_linkage(executable)
    cases = filter_cases(discover_cases(executable), args)
    if not cases:
        raise SystemExit("No complete two-policy benchmark blocks matched the filters.")

    print(f"Verified Accelerate linkage; matched {len(cases)} two-policy block(s).")
    for concurrency, outer_tasks, matrix_n in cases:
        print(f"c={concurrency} tasks={outer_tasks} n={matrix_n}")
    host_preconditioning = precondition_host(
        args,
        workers=max(key[0] for key in cases),
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

    with tempfile.TemporaryDirectory(prefix="pgo-eigen-accelerate-nested-") as temp_dir:
        temporary = Path(temp_dir)
        for block_index, (repetition, key) in enumerate(schedule, start=1):
            guard_host_condition(
                args,
                host_preconditioning,
                label=f"r={repetition}:key={key}",
            )
            concurrency, outer_tasks, matrix_n = key
            policies = balanced_order(
                POLICIES,
                repetition=repetition - 1,
                seed=args.seed,
                block_key=f"c={concurrency}:tasks={outer_tasks}:n={matrix_n}",
            )
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
                )
                measurements[policy] = run_one(
                    executable,
                    cases[key][policy],
                    temporary / f"{block_index}-{policy}-steady.json",
                    args.min_time,
                    args.warmup_time,
                )

            validate_block(key, cold_probes, args.checksum_relative_tolerance)
            validate_block(key, measurements, args.checksum_relative_tolerance)
            single_time = float(measurements["ExecutorSingle"]["wall_seconds"])
            multi_time = float(measurements["ExecutorMulti"]["wall_seconds"])
            cold_single_time = float(cold_probes["ExecutorSingle"]["wall_seconds"])
            cold_multi_time = float(cold_probes["ExecutorMulti"]["wall_seconds"])
            records.append(
                {
                    "repetition": repetition,
                    "concurrency": concurrency,
                    "outer_tasks": outer_tasks,
                    "matrix_n": matrix_n,
                    "order": policies,
                    "measurements": measurements,
                    "cold_probes": cold_probes,
                    "multi_over_single": multi_time / single_time,
                    "cold_multi_over_single": cold_multi_time / cold_single_time,
                    "extra_thread_delta": counter(
                        cold_probes["ExecutorMulti"], "extra_threads"
                    )
                    - counter(cold_probes["ExecutorSingle"], "extra_threads"),
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
        "host_preconditioning": host_preconditioning,
        "linkage": linkage,
        "records": records,
        "summary": summary,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2) + "\n")

    print("\nc tasks    n  steady multi/single  cold multi/single  extra-thread delta")
    for row in summary:
        print(
            f"{row['concurrency']:2d} {row['outer_tasks']:5d} {row['matrix_n']:4d}  "
            f"{row['median_multi_over_single']:19.4f}  "
            f"{row['median_cold_multi_over_single']:17.4f}  "
            f"{row['median_extra_thread_delta']:18.1f}"
        )
    print(f"\nWrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
