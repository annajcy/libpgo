#!/usr/bin/env python3
"""Run Google Benchmark cases in separate processes and merge CSV output."""

from __future__ import annotations

import argparse
import csv
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

from host_preconditioning import (
    add_host_preconditioning_arguments,
    balanced_order,
    guard_host_condition,
    precondition_host,
)
from benchmark_support.google_benchmark import (  # noqa: E402
    exact_filter,
    list_cases as list_google_benchmark_cases,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run each matching Google Benchmark case in a fresh process."
    )
    parser.add_argument(
        "benchmark", type=Path, help="Path to the benchmark executable."
    )
    parser.add_argument(
        "--filter", default=".*", help="Regex applied to benchmark case names."
    )
    parser.add_argument(
        "--out", type=Path, required=True, help="Merged CSV output path."
    )
    parser.add_argument(
        "--min-time", default="0.1s", help="Forwarded --benchmark_min_time value."
    )
    parser.add_argument(
        "--repetitions",
        type=int,
        default=1,
        help="Forwarded --benchmark_repetitions value.",
    )
    parser.add_argument(
        "--fresh-repetitions",
        type=int,
        default=0,
        help=(
            "Run each case this many times in separate fresh processes with "
            "--benchmark_min_time=1x and --benchmark_repetitions=1. "
            "0 keeps the normal single-process-per-case behavior."
        ),
    )
    parser.add_argument(
        "--case-limit",
        type=int,
        default=0,
        help="Run at most this many matching cases; 0 means no limit.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="List matching cases without running them.",
    )
    parser.add_argument(
        "--extra-arg",
        action="append",
        default=[],
        help="Additional argument forwarded to every benchmark invocation. Repeatable.",
    )
    parser.add_argument("--seed", type=int, default=20260717)
    add_host_preconditioning_arguments(parser)
    return parser.parse_args()


def run_command(
    command: list[str], *, cwd: Path | None = None
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def matching_cases(benchmark: Path, pattern: str) -> list[str]:
    regex = re.compile(pattern)
    return [
        name for name in list_google_benchmark_cases(benchmark) if regex.search(name)
    ]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        lines = f.readlines()

    header_index = None
    for index, line in enumerate(lines):
        if line.startswith("name,"):
            header_index = index
            break

    if header_index is None:
        raise ValueError(f"Could not find Google Benchmark CSV header in {path}")

    return list(csv.DictReader(lines[header_index:]))


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                fieldnames.append(key)

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    args = parse_args()
    if args.fresh_repetitions < 0:
        print("--fresh-repetitions must be non-negative.", file=sys.stderr)
        return 2

    benchmark = args.benchmark.resolve()
    if not benchmark.exists():
        print(f"Benchmark executable does not exist: {benchmark}", file=sys.stderr)
        return 2

    cases = matching_cases(benchmark, args.filter)
    if args.case_limit > 0:
        cases = cases[: args.case_limit]

    print(f"Matched {len(cases)} benchmark case(s).")
    for case in cases:
        print(case)
    if args.fresh_repetitions > 0:
        print(
            f"Fresh repetition mode: {args.fresh_repetitions} process(es) per case, "
            "benchmark_min_time=1x, benchmark_repetitions=1."
        )

    if args.dry_run:
        return 0
    if not cases:
        return 1

    host_preconditioning = precondition_host(args, workers=8)

    merged_rows: list[dict[str, str]] = []
    process_index = 0
    fresh_repetitions = args.fresh_repetitions if args.fresh_repetitions > 0 else 1
    benchmark_min_time = "1x" if args.fresh_repetitions > 0 else args.min_time
    benchmark_repetitions = 1 if args.fresh_repetitions > 0 else args.repetitions

    jobs: list[tuple[int, str]] = []
    for fresh_index in range(1, fresh_repetitions + 1):
        jobs.extend(
            (fresh_index, case)
            for case in balanced_order(
                cases,
                repetition=fresh_index - 1,
                seed=args.seed,
                block_key="isolated-cases",
            )
        )

    with tempfile.TemporaryDirectory(prefix="pgo-isolated-bench-") as tmp:
        tmpdir = Path(tmp)
        for job_index, (fresh_index, case) in enumerate(jobs, start=1):
            guard_host_condition(
                args,
                host_preconditioning,
                label=f"fresh={fresh_index}:case={case}",
            )
            process_index += 1
            case_index = cases.index(case) + 1
            case_csv = tmpdir / f"job-{job_index:04d}-fresh-{fresh_index:04d}.csv"
            command = [
                str(benchmark),
                f"--benchmark_filter={exact_filter(case)}",
                f"--benchmark_min_time={benchmark_min_time}",
                f"--benchmark_repetitions={benchmark_repetitions}",
                f"--benchmark_out={case_csv}",
                "--benchmark_out_format=csv",
                *args.extra_arg,
            ]

            print(
                f"[{job_index}/{len(jobs)} fresh {fresh_index}/{fresh_repetitions}] "
                f"{case}",
                flush=True,
            )
            result = run_command(command)
            if result.returncode != 0:
                print(result.stdout, file=sys.stderr)
                return result.returncode
            if not case_csv.exists():
                print(f"Benchmark did not produce CSV: {case_csv}", file=sys.stderr)
                print(result.stdout, file=sys.stderr)
                return 1

            rows = read_csv(case_csv)
            for row in rows:
                row["isolated_case_index"] = str(case_index)
                row["isolated_case_name"] = case
                row["isolated_process_index"] = str(process_index)
                if args.fresh_repetitions > 0:
                    row["fresh_repetition_index"] = str(fresh_index)
                    row["fresh_repetitions"] = str(fresh_repetitions)
            merged_rows.extend(rows)

    write_csv(args.out, merged_rows)
    host_path = args.out.with_suffix(args.out.suffix + ".host.json")
    host_path.write_text(json.dumps(host_preconditioning, indent=2) + "\n")
    print(f"Wrote {len(merged_rows)} row(s) to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
