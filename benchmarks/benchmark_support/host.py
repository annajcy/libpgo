"""CPU placement, topology, frequency, and host-state provenance."""

from __future__ import annotations

import argparse
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence


def parse_cpu_list(value: str) -> list[int]:
    """Parse a Linux CPU-list such as ``21-28,31`` into sorted CPU IDs."""
    cpu_ids: set[int] = set()
    for item in value.split(","):
        item = item.strip()
        if not item:
            raise ValueError("CPU list contains an empty item")
        if "-" not in item:
            if not item.isdecimal():
                raise ValueError(f"invalid CPU ID: {item}")
            cpu_ids.add(int(item))
            continue
        if item.count("-") != 1:
            raise ValueError(f"invalid CPU range: {item}")
        first_text, last_text = item.split("-")
        if not first_text.isdecimal() or not last_text.isdecimal():
            raise ValueError(f"invalid CPU range: {item}")
        first = int(first_text)
        last = int(last_text)
        if first > last:
            raise ValueError(f"descending CPU range: {item}")
        cpu_ids.update(range(first, last + 1))
    if not cpu_ids:
        raise ValueError("CPU list must not be empty")
    return sorted(cpu_ids)


def add_host_arguments(group: argparse._ArgumentGroup) -> None:
    group.add_argument(
        "--cpu-list",
        type=parse_cpu_list,
        help=(
            "CPU affinity for the controller and all inherited benchmark work, "
            "using Linux CPU-list syntax such as 21-28. On Linux, an inherited "
            "affinity is accepted only when its size matches benchmark concurrency."
        ),
    )
    group.add_argument(
        "--numa-node",
        type=int,
        help="Require every selected CPU to belong to this NUMA node.",
    )
    group.add_argument(
        "--allow-cross-numa",
        action="store_true",
        help="Explicitly permit a selected CPU list that spans NUMA nodes.",
    )


def validate_host_arguments(args: argparse.Namespace) -> None:
    numa_node = getattr(args, "numa_node", None)
    if numa_node is not None and numa_node < 0:
        raise ValueError("--numa-node must be nonnegative")
    if numa_node is not None and getattr(args, "allow_cross_numa", False):
        raise ValueError("--numa-node and --allow-cross-numa are mutually exclusive")


def available_cpu_ids() -> list[int] | None:
    if hasattr(os, "sched_getaffinity"):
        return sorted(os.sched_getaffinity(0))
    return None


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text().strip()
    except OSError:
        return None


def _read_int(path: Path) -> int | None:
    value = _read_text(path)
    if value is None:
        return None
    try:
        return int(value)
    except ValueError:
        return None


def cpu_numa_node(cpu: int) -> int | None:
    cpu_root = Path(f"/sys/devices/system/cpu/cpu{cpu}")
    for node_path in sorted(cpu_root.glob("node[0-9]*")):
        suffix = node_path.name.removeprefix("node")
        if suffix.isdecimal():
            return int(suffix)
    return None


def cpu_topology(cpu: int) -> dict[str, int | None]:
    root = Path(f"/sys/devices/system/cpu/cpu{cpu}/topology")
    return {
        "cpu": cpu,
        "core": _read_int(root / "core_id"),
        "socket": _read_int(root / "physical_package_id"),
        "numa_node": cpu_numa_node(cpu),
    }


def configure_cpu_placement(
    args: argparse.Namespace,
    *,
    expected_cpu_count: int,
    dry_run: bool,
) -> dict[str, Any]:
    validate_host_arguments(args)
    requested_value = getattr(args, "cpu_list", None)
    requested = (
        parse_cpu_list(requested_value)
        if isinstance(requested_value, str)
        else requested_value
    )
    if requested is not None and len(requested) != expected_cpu_count:
        raise ValueError(
            f"--cpu-list selects {len(requested)} CPUs, but benchmark concurrency "
            f"requires exactly {expected_cpu_count}: {requested}"
        )

    if requested is not None:
        if not hasattr(os, "sched_setaffinity"):
            raise RuntimeError("--cpu-list requires OS CPU-affinity support")
        try:
            os.sched_setaffinity(0, set(requested))
        except OSError as error:
            raise RuntimeError(
                f"failed to apply --cpu-list {requested}: {error}"
            ) from error

    actual = available_cpu_ids()
    if actual is None:
        if getattr(args, "numa_node", None) is not None:
            raise RuntimeError("--numa-node requires OS CPU-affinity support")
        return {
            "requested_cpu_list": requested,
            "actual_cpu_list": None,
            "expected_cpu_count": expected_cpu_count,
            "affinity_enforced": False,
            "topology": None,
            "numa_nodes": None,
            "expected_numa_node": getattr(args, "numa_node", None),
            "cross_numa_allowed": bool(getattr(args, "allow_cross_numa", False)),
        }

    if requested is not None and actual != requested:
        raise RuntimeError(
            f"kernel applied CPU affinity {actual}, which does not match "
            f"--cpu-list {requested}"
        )
    if len(actual) != expected_cpu_count and not dry_run:
        raise RuntimeError(
            f"benchmark requires exactly {expected_cpu_count} affinity CPUs, but "
            f"the controller can run on {len(actual)} CPUs: {actual}. Pass an "
            "explicit --cpu-list with the required size."
        )

    topology = [cpu_topology(cpu) for cpu in actual]
    known_nodes = sorted(
        {
            int(record["numa_node"])
            for record in topology
            if record["numa_node"] is not None
        }
    )
    expected_node = getattr(args, "numa_node", None)
    unknown_nodes = any(record["numa_node"] is None for record in topology)
    if expected_node is not None and (unknown_nodes or known_nodes != [expected_node]):
        raise RuntimeError(
            f"selected CPUs do not all belong to NUMA node {expected_node}: "
            f"{topology}"
        )
    if (
        len(known_nodes) > 1
        and not getattr(args, "allow_cross_numa", False)
        and not dry_run
    ):
        raise RuntimeError(
            f"selected CPUs span NUMA nodes {known_nodes}; choose one node or pass "
            "--allow-cross-numa explicitly"
        )

    return {
        "requested_cpu_list": requested,
        "actual_cpu_list": actual,
        "expected_cpu_count": expected_cpu_count,
        "affinity_enforced": requested is not None or len(actual) == expected_cpu_count,
        "topology": topology,
        "numa_nodes": known_nodes,
        "expected_numa_node": expected_node,
        "cross_numa_allowed": bool(getattr(args, "allow_cross_numa", False)),
    }


def frequency_metadata(cpu_ids: Sequence[int] | None) -> dict[str, Any] | None:
    if not cpu_ids:
        return None
    records: list[dict[str, Any]] = []
    for cpu in cpu_ids:
        root = Path(f"/sys/devices/system/cpu/cpu{cpu}/cpufreq")
        if not root.is_dir():
            continue
        records.append(
            {
                "cpu": cpu,
                "driver": _read_text(root / "scaling_driver"),
                "governor": _read_text(root / "scaling_governor"),
                "minimum_khz": _read_text(root / "scaling_min_freq"),
                "maximum_khz": _read_text(root / "scaling_max_freq"),
                "current_khz": _read_text(root / "scaling_cur_freq"),
            }
        )
    return {"cpus": records} if records else None


def cgroup_metadata() -> dict[str, str | None]:
    """Record the effective cgroup-v2 CPU placement and quota when available."""
    root = Path("/sys/fs/cgroup")
    membership = _read_text(Path("/proc/self/cgroup"))
    unified_path = None
    if membership is not None:
        for line in membership.splitlines():
            if line.startswith("0::"):
                unified_path = line.removeprefix("0::") or "/"
                break
    effective_root = (
        root / unified_path.lstrip("/") if unified_path is not None else root
    )
    return {
        "membership": membership,
        "unified_path": unified_path,
        "cpuset_cpus_effective": _read_text(
            effective_root / "cpuset.cpus.effective"
        ),
        "cpuset_mems_effective": _read_text(
            effective_root / "cpuset.mems.effective"
        ),
        "cpu_max": _read_text(effective_root / "cpu.max"),
        "cpu_weight": _read_text(effective_root / "cpu.weight"),
    }


def host_state_snapshot(cpu_ids: Sequence[int] | None = None) -> dict[str, Any]:
    if cpu_ids is None:
        cpu_ids = available_cpu_ids()
    load_average = None
    try:
        load_average = list(os.getloadavg())
    except OSError:
        pass
    return {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "load_average": load_average,
        "cpu_pressure": _read_text(Path("/proc/pressure/cpu")),
        "cgroup": cgroup_metadata(),
        "frequency": frequency_metadata(cpu_ids),
    }
