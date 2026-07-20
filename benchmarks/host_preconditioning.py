#!/usr/bin/env python3
"""Shared host preconditioning and balanced scheduling for benchmarks."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import multiprocessing
import os
import platform
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence, TypeVar


DEFAULT_PREHEAT_SECONDS = 60.0
DEFAULT_PROBE_SECONDS = 1.0
DEFAULT_REQUIRED_STABLE_PROBES = 3
DEFAULT_MAX_PROBES = 6
DEFAULT_STABILITY_TOLERANCE = 0.02
DEFAULT_DRIFT_TOLERANCE = 0.05
DEFAULT_ABORT_DRIFT_TOLERANCE = 0.50
_BURN_BATCH_SIZE = 4096
_T = TypeVar("_T")
ORDER_DESIGN = "williams_first_order_carryover"


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


def add_host_preconditioning_arguments(parser: argparse.ArgumentParser) -> None:
    group = parser.add_argument_group("host preconditioning")
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
    group.add_argument(
        "--allow-incomplete-order-cycle",
        action="store_true",
        help=(
            "Permit a diagnostic run whose repetitions or truncation do not form "
            "complete counterbalanced order cycles."
        ),
    )
    group.add_argument(
        "--host-preheat-seconds",
        type=float,
        default=DEFAULT_PREHEAT_SECONDS,
        help="Policy-neutral all-core preheat before any measured case.",
    )
    group.add_argument(
        "--host-preheat-workers",
        type=int,
        help="Override the default benchmark concurrency used for preheating.",
    )
    group.add_argument(
        "--host-probe-seconds",
        type=float,
        default=DEFAULT_PROBE_SECONDS,
        help="Duration of each policy-neutral stability probe.",
    )
    group.add_argument(
        "--host-required-stable-probes",
        type=int,
        default=DEFAULT_REQUIRED_STABLE_PROBES,
        help="Number of trailing probes used by the stability check.",
    )
    group.add_argument(
        "--host-max-probes",
        type=int,
        default=DEFAULT_MAX_PROBES,
        help="Maximum stability probes before aborting an unstable run.",
    )
    group.add_argument(
        "--host-stability-tolerance",
        type=float,
        default=DEFAULT_STABILITY_TOLERANCE,
        help="Maximum relative spread of trailing probe throughput.",
    )
    group.add_argument(
        "--host-drift-tolerance",
        type=float,
        default=DEFAULT_DRIFT_TOLERANCE,
        help="Maximum case-guard deviation from the initial stable baseline.",
    )
    group.add_argument(
        "--host-abort-drift-tolerance",
        type=float,
        default=DEFAULT_ABORT_DRIFT_TOLERANCE,
        help="Always abort when a case guard drifts this far from the baseline.",
    )
    group.add_argument(
        "--allow-host-drift",
        action="store_true",
        help=(
            "Record and continue through host drift above --host-drift-tolerance. "
            "This weakens timing comparability and is disabled by default."
        ),
    )
    group.add_argument(
        "--skip-host-preconditioning",
        action="store_true",
        help="Explicitly skip host preconditioning and record that choice.",
    )


def validate_host_preconditioning_arguments(args: argparse.Namespace) -> None:
    numa_node = getattr(args, "numa_node", None)
    if numa_node is not None and numa_node < 0:
        raise ValueError("--numa-node must be nonnegative")
    if numa_node is not None and getattr(args, "allow_cross_numa", False):
        raise ValueError("--numa-node and --allow-cross-numa are mutually exclusive")
    if args.host_preheat_seconds < 0:
        raise ValueError("--host-preheat-seconds must be nonnegative")
    if args.host_preheat_workers is not None and args.host_preheat_workers <= 0:
        raise ValueError("--host-preheat-workers must be positive")
    if args.host_probe_seconds <= 0:
        raise ValueError("--host-probe-seconds must be positive")
    if args.host_required_stable_probes < 2:
        raise ValueError("--host-required-stable-probes must be at least two")
    if args.host_max_probes < args.host_required_stable_probes:
        raise ValueError(
            "--host-max-probes must be at least --host-required-stable-probes"
        )
    if not 0 < args.host_stability_tolerance < 1:
        raise ValueError("--host-stability-tolerance must be between zero and one")
    if not args.host_stability_tolerance <= args.host_drift_tolerance < 1:
        raise ValueError(
            "--host-drift-tolerance must be at least the stability tolerance "
            "and less than one"
        )
    if not args.host_drift_tolerance <= args.host_abort_drift_tolerance < 1:
        raise ValueError(
            "--host-abort-drift-tolerance must be at least the drift tolerance "
            "and less than one"
        )


def _available_cpu_ids() -> list[int] | None:
    if hasattr(os, "sched_getaffinity"):
        return sorted(os.sched_getaffinity(0))
    return None


def _read_int(path: Path) -> int | None:
    value = _read_text(path)
    if value is None:
        return None
    try:
        return int(value)
    except ValueError:
        return None


def _cpu_numa_node(cpu: int) -> int | None:
    cpu_root = Path(f"/sys/devices/system/cpu/cpu{cpu}")
    for node_path in sorted(cpu_root.glob("node[0-9]*")):
        suffix = node_path.name.removeprefix("node")
        if suffix.isdecimal():
            return int(suffix)
    return None


def _cpu_topology(cpu: int) -> dict[str, int | None]:
    root = Path(f"/sys/devices/system/cpu/cpu{cpu}/topology")
    return {
        "cpu": cpu,
        "core": _read_int(root / "core_id"),
        "socket": _read_int(root / "physical_package_id"),
        "numa_node": _cpu_numa_node(cpu),
    }


def _configure_cpu_placement(
    args: argparse.Namespace,
    *,
    expected_cpu_count: int,
    dry_run: bool,
) -> dict[str, Any]:
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

    actual = _available_cpu_ids()
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

    topology = [_cpu_topology(cpu) for cpu in actual]
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


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text().strip()
    except OSError:
        return None


def _frequency_metadata(cpu_ids: Sequence[int] | None) -> dict[str, Any] | None:
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


def host_state_snapshot(cpu_ids: Sequence[int] | None = None) -> dict[str, Any]:
    if cpu_ids is None:
        cpu_ids = _available_cpu_ids()
    load_average = None
    try:
        load_average = list(os.getloadavg())
    except OSError:
        pass
    cpu_pressure = _read_text(Path("/proc/pressure/cpu"))
    return {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "load_average": load_average,
        "cpu_pressure": cpu_pressure,
        "frequency": _frequency_metadata(cpu_ids),
    }


def _burn_worker(
    start_event: Any,
    ready_queue: Any,
    result_queue: Any,
    cpu: int | None,
    duration_seconds: float,
) -> None:
    if cpu is not None and hasattr(os, "sched_setaffinity"):
        os.sched_setaffinity(0, {cpu})
    ready_queue.put(True)
    start_event.wait()
    deadline = time.monotonic() + duration_seconds
    iterations = 0
    value = 1.0000001
    while time.monotonic() < deadline:
        for _ in range(_BURN_BATCH_SIZE):
            value = value * 1.00000001 + 0.00000001
        iterations += _BURN_BATCH_SIZE
    result_queue.put((iterations, value))


def _run_burn_round(duration_seconds: float, workers: int) -> dict[str, Any]:
    context = multiprocessing.get_context("spawn")
    cpu_ids = _available_cpu_ids()
    selected_cpu_ids = cpu_ids[:workers] if cpu_ids else [None] * workers
    actual_workers = len(selected_cpu_ids)
    start_event = context.Event()
    ready_queue = context.Queue()
    result_queue = context.Queue()
    processes = [
        context.Process(
            target=_burn_worker,
            args=(start_event, ready_queue, result_queue, cpu, duration_seconds),
        )
        for cpu in selected_cpu_ids
    ]
    for process in processes:
        process.start()
    for _ in processes:
        ready_queue.get(timeout=30.0)
    started_at = time.monotonic()
    start_event.set()
    results = [result_queue.get(timeout=duration_seconds + 30.0) for _ in processes]
    for process in processes:
        process.join(timeout=10.0)
        if process.exitcode != 0:
            raise RuntimeError(
                f"host preconditioning worker exited with code {process.exitcode}"
            )
    elapsed = time.monotonic() - started_at
    iterations = sum(result[0] for result in results)
    return {
        "duration_seconds": duration_seconds,
        "elapsed_seconds": elapsed,
        "workers": actual_workers,
        "iterations": iterations,
        "iterations_per_second": iterations / elapsed,
    }


def precondition_host(
    args: argparse.Namespace,
    *,
    workers: int,
    dry_run: bool = False,
) -> dict[str, Any]:
    validate_host_preconditioning_arguments(args)
    cpu_placement = _configure_cpu_placement(
        args,
        expected_cpu_count=workers,
        dry_run=dry_run,
    )
    cpu_ids = cpu_placement["actual_cpu_list"]
    requested_workers = args.host_preheat_workers or workers
    actual_workers = max(
        1, min(requested_workers, len(cpu_ids) if cpu_ids else requested_workers)
    )
    result: dict[str, Any] = {
        "enabled": not args.skip_host_preconditioning,
        "dry_run": dry_run,
        "strict_timing_comparability": bool(
            not args.skip_host_preconditioning and not dry_run
        ),
        "platform": platform.platform(),
        "cpu_placement": cpu_placement,
        "requested_workers": requested_workers,
        "workers": actual_workers,
        "cpu_affinity": cpu_ids,
        "preheat_seconds": args.host_preheat_seconds,
        "probe_seconds": args.host_probe_seconds,
        "required_stable_probes": args.host_required_stable_probes,
        "max_probes": args.host_max_probes,
        "stability_tolerance": args.host_stability_tolerance,
        "drift_tolerance": args.host_drift_tolerance,
        "abort_drift_tolerance": args.host_abort_drift_tolerance,
        "allow_host_drift": bool(getattr(args, "allow_host_drift", False)),
        "state_before": host_state_snapshot(cpu_ids),
    }
    if dry_run or args.skip_host_preconditioning:
        result["status"] = "skipped"
        result["state_after"] = host_state_snapshot(cpu_ids)
        return result

    if args.host_preheat_seconds > 0:
        result["preheat"] = _run_burn_round(args.host_preheat_seconds, actual_workers)

    probes: list[dict[str, Any]] = []
    stable = False
    for _ in range(args.host_max_probes):
        probes.append(_run_burn_round(args.host_probe_seconds, actual_workers))
        if len(probes) < args.host_required_stable_probes:
            continue
        trailing = [
            probe["iterations_per_second"]
            for probe in probes[-args.host_required_stable_probes :]
        ]
        center = statistics.median(trailing)
        relative_spread = (max(trailing) - min(trailing)) / center
        relative_mad = (
            statistics.median(abs(value - center) for value in trailing) / center
        )
        probes[-1]["trailing_relative_spread"] = relative_spread
        probes[-1]["trailing_relative_mad"] = relative_mad
        if (
            relative_mad <= args.host_stability_tolerance
            and relative_spread <= 5.0 * args.host_stability_tolerance
        ):
            stable = True
            break
    result["probes"] = probes
    result["stable"] = stable
    result["status"] = "stable" if stable else "unstable"
    result["state_after"] = host_state_snapshot(cpu_ids)
    if not stable:
        raise RuntimeError(
            "host performance did not stabilize: trailing probe throughput spread "
            f"or MAD exceeded the allowed bounds after {args.host_max_probes} probes; "
            f"last probes={trailing}"
        )
    return result


def guard_host_condition(
    args: argparse.Namespace,
    preconditioning: dict[str, Any],
    *,
    label: str,
) -> dict[str, Any]:
    """Verify the policy-neutral throughput baseline before a measured case."""
    if not preconditioning.get("enabled") or preconditioning.get("dry_run"):
        check = {
            "label": label,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "status": "skipped",
        }
        preconditioning["strict_timing_comparability"] = False
        preconditioning.setdefault("block_checks", []).append(check)
        return check

    baseline_values = [
        probe["iterations_per_second"]
        for probe in preconditioning["probes"][-args.host_required_stable_probes :]
    ]
    baseline = statistics.median(baseline_values)
    probes = [_run_burn_round(args.host_probe_seconds, preconditioning["workers"])]

    def deviation() -> float:
        current = statistics.median(probe["iterations_per_second"] for probe in probes)
        return abs(current / baseline - 1.0)

    if deviation() > args.host_stability_tolerance:
        for _ in range(args.host_required_stable_probes - 1):
            probes.append(
                _run_burn_round(
                    args.host_probe_seconds,
                    preconditioning["workers"],
                )
            )

    check = {
        "label": label,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "baseline_iterations_per_second": baseline,
        "probes": probes,
        "relative_deviation": deviation(),
    }
    if check["relative_deviation"] <= args.host_drift_tolerance:
        check["status"] = "stable"
    elif check["relative_deviation"] <= args.host_abort_drift_tolerance:
        check["status"] = "drifted"
    else:
        check["status"] = "unstable"
    if check["status"] != "stable":
        preconditioning["strict_timing_comparability"] = False
    preconditioning.setdefault("block_checks", []).append(check)
    allow_host_drift = bool(getattr(args, "allow_host_drift", False))
    if check["status"] == "unstable" or (
        check["status"] == "drifted" and not allow_host_drift
    ):
        allowed_limit = (
            args.host_abort_drift_tolerance
            if allow_host_drift
            else args.host_drift_tolerance
        )
        raise RuntimeError(
            f"host performance changed before {label}: policy-neutral probe "
            f"deviated {check['relative_deviation']:.2%} from the initial baseline "
            f"(allowed limit {allowed_limit:.2%})"
        )
    return check


def balanced_order(
    values: Sequence[_T],
    *,
    repetition: int,
    seed: int,
    block_key: str,
) -> list[_T]:
    """Return a deterministic first-order carry-over-balanced policy order.

    A plain cyclic Latin square balances positions but repeats the same policy
    adjacencies in every row. That leaves paired benchmarks vulnerable when a
    long-running case changes the host condition for the case that follows it.
    This uses a Williams design: even-sized policy sets balance every directed
    first-order transition in one cycle, while odd-sized sets add reversed rows
    to complete the balance in two cycles.
    """
    if not values:
        return []
    digest = hashlib.sha256(f"{seed}:{block_key}".encode()).digest()
    keyed = sorted(
        values,
        key=lambda value: hashlib.sha256(digest + repr(value).encode()).digest(),
    )
    count = len(keyed)
    if count == 1:
        return keyed

    base_indices = [0]
    for position in range(1, count):
        if position % 2:
            base_indices.append((position + 1) // 2)
        else:
            base_indices.append(count - position // 2)

    if count % 2:
        cycle_length = 2 * count
        within_cycle = repetition % cycle_length
        row = within_cycle % count
        reverse = within_cycle >= count
    else:
        row = repetition % count
        reverse = False

    order = [keyed[(index + row) % count] for index in base_indices]
    if reverse:
        order.reverse()
    return order


def order_cycle_length(value_count: int) -> int:
    """Return the number of repetitions in one complete Williams cycle."""
    if value_count <= 0:
        raise ValueError("counterbalanced order requires at least one value")
    if value_count == 1:
        return 1
    return value_count if value_count % 2 == 0 else 2 * value_count


def order_configuration(
    repetitions: int,
    groups: Sequence[tuple[str, Sequence[Any]]],
    *,
    allow_incomplete: bool = False,
    schedule_truncated: bool = False,
) -> dict[str, Any]:
    """Validate and describe the complete counterbalanced scheduling cycle."""
    if repetitions <= 0:
        raise ValueError("counterbalanced repetitions must be positive")
    group_records = []
    cycle_lengths = []
    for name, values in groups:
        cycle_length = order_cycle_length(len(values))
        cycle_lengths.append(cycle_length)
        group_records.append(
            {
                "name": name,
                "value_count": len(values),
                "order_cycle_length": cycle_length,
            }
        )
    if not cycle_lengths:
        raise ValueError("counterbalanced order requires at least one value group")

    combined_cycle_length = math.lcm(*cycle_lengths)
    complete = repetitions % combined_cycle_length == 0 and not schedule_truncated
    if not complete and not allow_incomplete:
        reasons = []
        if repetitions % combined_cycle_length:
            reasons.append(
                f"{repetitions} repetitions is not a multiple of "
                f"order_cycle_length={combined_cycle_length}"
            )
        if schedule_truncated:
            reasons.append("the requested case limit truncates a repetition block")
        raise ValueError(
            "; ".join(reasons)
            + ". Use a complete cycle or pass --allow-incomplete-order-cycle "
            "for a diagnostic run."
        )

    return {
        "design": ORDER_DESIGN,
        "repetitions": repetitions,
        "order_cycle_length": combined_cycle_length,
        "completed_order_cycles": repetitions // combined_cycle_length,
        "strictly_balanced": complete,
        "schedule_truncated": schedule_truncated,
        "groups": group_records,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workers", type=int, default=8)
    add_host_preconditioning_arguments(parser)
    args = parser.parse_args()
    if args.workers <= 0:
        raise SystemExit("--workers must be positive")
    print(json.dumps(precondition_host(args, workers=args.workers), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
