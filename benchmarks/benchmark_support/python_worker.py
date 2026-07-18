"""Fixture for benchmarks that measure one fresh Python worker per sample."""

from __future__ import annotations

import json
import os
import subprocess
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from benchmark_support.process import command_text


def worker_environment(
    root: Path, overrides: Mapping[str, str] | None = None
) -> dict[str, str]:
    """Create a worker environment that imports the checkout under test."""
    environment = os.environ.copy()
    if overrides:
        environment.update(overrides)
    environment["PYTHONPATH"] = os.pathsep.join(
        [str(root), environment.get("PYTHONPATH", "")]
    ).rstrip(os.pathsep)
    return environment


def parse_json_result_marker(
    stdout: str, marker: str, command: Sequence[str | Path]
) -> dict[str, Any]:
    payloads = [
        line[len(marker) :] for line in stdout.splitlines() if line.startswith(marker)
    ]
    if len(payloads) != 1:
        raise RuntimeError(
            f"worker emitted {len(payloads)} result payloads: "
            f"{command_text(command)}\n{stdout}"
        )
    return json.loads(payloads[0])


def run_json_worker(
    command: Sequence[str | Path],
    *,
    marker: str,
    cwd: Path,
    environment: Mapping[str, str],
) -> dict[str, Any]:
    """Run one isolated worker process and return its sole JSON result marker."""
    rendered = [str(part) for part in command]
    completed = subprocess.run(
        rendered,
        cwd=cwd,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"worker failed with exit code {completed.returncode}: "
            f"{command_text(rendered)}\n{completed.stdout}"
        )
    return parse_json_result_marker(completed.stdout, marker, rendered)
