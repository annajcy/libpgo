#!/usr/bin/env python3
"""Run Google Benchmark cases in separate processes and merge CSV output."""

from __future__ import annotations

import argparse
import csv
import re
import subprocess
import sys
import tempfile
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run each matching Google Benchmark case in a fresh process."
    )
    parser.add_argument("benchmark", type=Path, help="Path to the benchmark executable.")
    parser.add_argument("--filter", default=".*", help="Regex applied to benchmark case names.")
    parser.add_argument("--out", type=Path, required=True, help="Merged CSV output path.")
    parser.add_argument("--min-time", default="0.1s", help="Forwarded --benchmark_min_time value.")
    parser.add_argument(
        "--repetitions",
        type=int,
        default=1,
        help="Forwarded --benchmark_repetitions value.",
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
    return parser.parse_args()


def run_command(command: list[str], *, cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def list_cases(benchmark: Path, pattern: str) -> list[str]:
    result = run_command([str(benchmark), "--benchmark_list_tests"])
    if result.returncode != 0:
      print(result.stdout, file=sys.stderr)
      raise SystemExit(result.returncode)

    regex = re.compile(pattern)
    cases: list[str] = []
    for line in result.stdout.splitlines():
        name = line.strip()
        if not name:
            continue
        if regex.search(name):
            cases.append(name)
    return cases


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


def benchmark_filter_for_exact_case(case_name: str) -> str:
    return f"^{re.escape(case_name)}$"


def main() -> int:
    args = parse_args()
    benchmark = args.benchmark.resolve()
    if not benchmark.exists():
        print(f"Benchmark executable does not exist: {benchmark}", file=sys.stderr)
        return 2

    cases = list_cases(benchmark, args.filter)
    if args.case_limit > 0:
        cases = cases[: args.case_limit]

    print(f"Matched {len(cases)} benchmark case(s).")
    for case in cases:
        print(case)

    if args.dry_run:
        return 0
    if not cases:
        return 1

    merged_rows: list[dict[str, str]] = []
    with tempfile.TemporaryDirectory(prefix="pgo-isolated-bench-") as tmp:
        tmpdir = Path(tmp)
        for index, case in enumerate(cases, start=1):
            case_csv = tmpdir / f"case-{index:04d}.csv"
            command = [
                str(benchmark),
                f"--benchmark_filter={benchmark_filter_for_exact_case(case)}",
                f"--benchmark_min_time={args.min_time}",
                f"--benchmark_repetitions={args.repetitions}",
                f"--benchmark_out={case_csv}",
                "--benchmark_out_format=csv",
                *args.extra_arg,
            ]
            print(f"[{index}/{len(cases)}] {case}", flush=True)
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
                row["isolated_case_index"] = str(index)
                row["isolated_case_name"] = case
            merged_rows.extend(rows)

    write_csv(args.out, merged_rows)
    print(f"Wrote {len(merged_rows)} row(s) to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
