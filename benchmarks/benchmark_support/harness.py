"""Shared benchmark CLI and immutable host-environment provenance."""

from __future__ import annotations

import argparse
import platform
from typing import Any

from benchmark_support.host import (
    add_host_arguments,
    configure_cpu_placement,
    host_state_snapshot,
)
from benchmark_support.schedule import add_schedule_arguments


def add_benchmark_harness_arguments(parser: argparse.ArgumentParser) -> None:
    """Add placement and counterbalanced-order arguments shared by runners."""
    group = parser.add_argument_group("benchmark harness")
    add_host_arguments(group)
    add_schedule_arguments(group)


def prepare_benchmark_host(
    args: argparse.Namespace,
    *,
    workers: int,
    dry_run: bool = False,
) -> dict[str, Any]:
    """Apply strict placement and record the host without running a proxy workload."""
    placement = configure_cpu_placement(
        args,
        expected_cpu_count=workers,
        dry_run=dry_run,
    )
    cpu_ids = placement["actual_cpu_list"]
    return {
        "protocol": "workload_specific_warmup",
        "dry_run": dry_run,
        "platform": platform.platform(),
        "cpu_placement": placement,
        "state_before": host_state_snapshot(cpu_ids),
    }


def finish_benchmark_host(host: dict[str, Any]) -> None:
    """Append a read-only end snapshot without changing the workload state."""
    placement = host.get("cpu_placement", {})
    host["state_after"] = host_state_snapshot(placement.get("actual_cpu_list"))
