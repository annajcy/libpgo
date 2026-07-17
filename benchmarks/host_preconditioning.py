#!/usr/bin/env python3
"""Shared host preconditioning and balanced scheduling for benchmarks."""

from __future__ import annotations

import argparse
import hashlib
import json
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


def add_host_preconditioning_arguments(parser: argparse.ArgumentParser) -> None:
    group = parser.add_argument_group("host preconditioning")
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
        help="Maximum block-probe deviation from the initial stable baseline.",
    )
    group.add_argument(
        "--host-abort-drift-tolerance",
        type=float,
        default=DEFAULT_ABORT_DRIFT_TOLERANCE,
        help="Abort when a block probe drifts this far from the initial baseline.",
    )
    group.add_argument(
        "--skip-host-preconditioning",
        action="store_true",
        help="Explicitly skip host preconditioning and record that choice.",
    )


def validate_host_preconditioning_arguments(args: argparse.Namespace) -> None:
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
    cpu_ids = _available_cpu_ids()
    requested_workers = args.host_preheat_workers or workers
    actual_workers = max(
        1, min(requested_workers, len(cpu_ids) if cpu_ids else requested_workers)
    )
    result: dict[str, Any] = {
        "enabled": not args.skip_host_preconditioning,
        "dry_run": dry_run,
        "platform": platform.platform(),
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
    """Verify the policy-neutral throughput baseline before a temporal block."""
    if not preconditioning.get("enabled") or preconditioning.get("dry_run"):
        check = {
            "label": label,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "status": "skipped",
        }
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
    preconditioning.setdefault("block_checks", []).append(check)
    if check["status"] == "unstable":
        raise RuntimeError(
            f"host performance changed before {label}: policy-neutral probe "
            f"deviated {check['relative_deviation']:.2%} from the initial baseline "
            f"(abort limit {args.host_abort_drift_tolerance:.2%})"
        )
    return check


def balanced_order(
    values: Sequence[_T],
    *,
    repetition: int,
    seed: int,
    block_key: str,
) -> list[_T]:
    """Return a deterministic Latin-style rotation for one repetition block."""
    if not values:
        return []
    digest = hashlib.sha256(f"{seed}:{block_key}".encode()).digest()
    keyed = sorted(
        values,
        key=lambda value: hashlib.sha256(digest + repr(value).encode()).digest(),
    )
    offset = repetition % len(keyed)
    rotated = keyed[offset:] + keyed[:offset]
    # Reverse alternate complete Latin cycles to reduce persistent carryover.
    if (repetition // len(keyed)) % 2:
        rotated.reverse()
    return rotated


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
