"""Execution fixture for JSON-producing Google Benchmark binaries."""

from __future__ import annotations

import json
import math
import re
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from benchmark_support.process import checked_output


TIME_SCALE = {"ns": 1e-9, "us": 1e-6, "ms": 1e-3, "s": 1.0}


def list_cases(
    executable: Path, environment: Mapping[str, str] | None = None
) -> list[str]:
    """Return the test names advertised by a Google Benchmark executable."""
    return [
        line.strip()
        for line in checked_output(
            [executable, "--benchmark_list_tests"], environment
        ).splitlines()
        if line.strip()
    ]


def exact_filter(name: str) -> str:
    return f"^{re.escape(name)}$"


def run_case(
    executable: Path,
    name: str,
    output: Path,
    min_time: str,
    warmup_time: float,
    environment: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """Run exactly one non-aggregate Google Benchmark case and normalize time."""
    checked_output(
        [
            executable,
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
    time_unit = row.get("time_unit")
    scale = TIME_SCALE.get(time_unit)
    if scale is None:
        raise RuntimeError(f"Unsupported time unit for {name}: {time_unit}")
    row["wall_seconds"] = float(row["real_time"]) * scale
    return row


def integer_counter(row: Mapping[str, Any], name: str) -> int:
    """Read a finite integer-valued benchmark counter."""
    value = float(row.get(name, math.nan))
    if not math.isfinite(value):
        raise RuntimeError(f"Missing finite counter {name} in {row.get('name')}.")
    return round(value)
