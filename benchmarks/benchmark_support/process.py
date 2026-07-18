"""Portable subprocess and executable-discovery helpers for benchmarks."""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
from collections.abc import Iterable, Mapping
from pathlib import Path


MarkerValue = str | int | float


def command_text(command: Iterable[str | Path]) -> str:
    """Render a command for diagnostics without invoking a shell."""
    return shlex.join(str(part) for part in command)


def checked_output(
    command: Iterable[str | Path],
    environment: Mapping[str, str] | None = None,
    *,
    cwd: Path | None = None,
) -> str:
    """Run a command, merging stderr into stdout and raise on failure."""
    rendered = [str(part) for part in command]
    result = subprocess.run(
        rendered,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
        env=environment,
        cwd=cwd,
    )
    if result.returncode != 0:
        raise RuntimeError(f"{command_text(rendered)}\n{result.stdout}")
    return result.stdout


def report_output(
    command: Iterable[str | Path],
    environment: Mapping[str, str],
    *,
    empty_output_message: str = "Empty request output.",
) -> str:
    """Run a reporting command whose empty result is a valid outcome."""
    rendered = [str(part) for part in command]
    result = subprocess.run(
        rendered,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
        env=environment,
    )
    if result.returncode == 0:
        return result.stdout
    if empty_output_message in result.stdout:
        return ""
    raise RuntimeError(f"{command_text(rendered)}\n{result.stdout}")


def resolve_file(path: Path, label: str, *, executable: bool = False) -> Path:
    """Resolve an existing file and report a consistent diagnostic otherwise."""
    candidate = path.expanduser().resolve()
    if not candidate.is_file():
        raise FileNotFoundError(f"{label} does not exist: {candidate}")
    if executable and not os.access(candidate, os.X_OK):
        raise PermissionError(f"{label} is not executable: {candidate}")
    return candidate


def resolve_vtune(explicit: Path | None) -> Path:
    """Resolve an explicit VTune executable or discover it on PATH."""
    if explicit is not None:
        return resolve_file(explicit, "VTune executable")
    discovered = shutil.which("vtune")
    if discovered is None:
        raise FileNotFoundError(
            "VTune CLI was not found. Pass --vtune or omit --collect-vtune."
        )
    return Path(discovered).resolve()


def parse_key_value_marker(
    output: str,
    prefix: str,
    *,
    string_fields: frozenset[str] = frozenset(),
    integer_fields: frozenset[str] = frozenset(),
    float_fields: frozenset[str] = frozenset(),
) -> dict[str, MarkerValue]:
    """Parse one complete, typed ``PREFIX key=value ...`` result record."""
    marker_lines = [
        line.strip() for line in output.splitlines() if line.startswith(prefix)
    ]
    if len(marker_lines) != 1:
        raise RuntimeError(
            f"Expected exactly one {prefix} line, found {len(marker_lines)}."
        )

    expected = string_fields | integer_fields | float_fields
    values: dict[str, MarkerValue] = {}
    for item in marker_lines[0][len(prefix) :].strip().split():
        key, separator, raw_value = item.partition("=")
        if not separator:
            raise RuntimeError(f"Malformed result marker item: {item}")
        if key in string_fields:
            values[key] = raw_value
        elif key in integer_fields:
            values[key] = int(raw_value)
        elif key in float_fields:
            values[key] = float(raw_value)
        else:
            raise RuntimeError(f"Unknown result marker field: {key}")

    missing = expected - values.keys()
    if missing:
        raise RuntimeError(f"Result marker is missing fields: {sorted(missing)}")
    return values
