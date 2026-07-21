"""Run self-timed C++ benchmark probes under the shared Python harness."""

from __future__ import annotations

import math
import re
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any

from benchmark_support.process import checked_output, parse_key_value_marker


_DURATION_PATTERN = re.compile(
    r"^(?P<value>(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)"
    r"(?P<unit>ns|us|ms|s)?$"
)
_DURATION_SCALE = {"ns": 1e-9, "us": 1e-6, "ms": 1e-3, "s": 1.0}
COMMON_INTEGER_FIELDS = frozenset(
    {
        "configured_warmup_min_operations",
        "actual_warmup_operations",
        "measurement_operations",
    }
)
COMMON_FLOAT_FIELDS = frozenset(
    {
        "configured_warmup_seconds",
        "actual_warmup_seconds",
        "configured_measurement_min_seconds",
        "measurement_wall_seconds",
    }
)


def duration_seconds(value: str) -> float:
    """Parse the duration spelling accepted by benchmark runner ``--min-time``."""
    match = _DURATION_PATTERN.fullmatch(value.strip())
    if match is None:
        raise ValueError(f"invalid benchmark duration: {value}")
    number = float(match.group("value"))
    unit = match.group("unit") or "s"
    seconds = number * _DURATION_SCALE[unit]
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError(f"benchmark duration must be finite and nonnegative: {value}")
    return seconds


def integer_counter(row: Mapping[str, Any], name: str) -> int:
    """Read one finite integer-valued telemetry field."""
    value = float(row.get(name, math.nan))
    if not math.isfinite(value) or value != round(value):
        raise RuntimeError(f"Missing integer-valued probe field {name}.")
    return round(value)


def run_cpp_probe(
    executable: Path,
    arguments: Iterable[str],
    *,
    marker: str,
    min_time: str,
    warmup_seconds: float,
    warmup_min_operations: int,
    string_fields: frozenset[str] = frozenset(),
    integer_fields: frozenset[str] = frozenset(),
    float_fields: frozenset[str] = frozenset(),
    environment: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """Run one fresh C++ worker and normalize its self-timed per-operation result."""
    measurement_min_seconds = duration_seconds(min_time)
    command = [
        executable,
        *arguments,
        f"--warmup-seconds={warmup_seconds}",
        f"--warmup-min-operations={warmup_min_operations}",
        f"--measurement-min-seconds={measurement_min_seconds}",
        "--measurement-min-operations=1",
    ]
    output = checked_output(command, environment)
    row = parse_key_value_marker(
        output,
        marker,
        string_fields=string_fields,
        integer_fields=integer_fields | COMMON_INTEGER_FIELDS,
        float_fields=float_fields | COMMON_FLOAT_FIELDS,
    )
    if row["configured_warmup_seconds"] != warmup_seconds:
        raise RuntimeError("C++ probe reported the wrong warmup time bound.")
    if row["configured_warmup_min_operations"] != warmup_min_operations:
        raise RuntimeError("C++ probe reported the wrong warmup operation bound.")
    actual_warmup_operations = int(row["actual_warmup_operations"])
    if actual_warmup_operations < warmup_min_operations:
        raise RuntimeError("C++ probe completed too few warmup operations.")
    if warmup_seconds == 0.0 and actual_warmup_operations != warmup_min_operations:
        raise RuntimeError("Fixed-count C++ probe warmup did not stop exactly.")
    if row["configured_measurement_min_seconds"] != measurement_min_seconds:
        raise RuntimeError("C++ probe reported the wrong measurement time bound.")
    operations = int(row["measurement_operations"])
    elapsed = float(row["measurement_wall_seconds"])
    if operations < 1 or not math.isfinite(elapsed) or elapsed <= 0:
        raise RuntimeError("C++ probe reported an invalid timed measurement.")
    if elapsed < measurement_min_seconds:
        raise RuntimeError("C++ probe stopped before its measurement time bound.")
    row["wall_seconds"] = elapsed / operations
    return row
