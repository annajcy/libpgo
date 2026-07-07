"""Process-wide CPU concurrency controls for libpgo."""

from __future__ import annotations

from collections.abc import Iterator

import pypgo._core as _core


def _normalize_num_workers(num_workers: int | None) -> int | None:
    if num_workers is None:
        return None
    value = int(num_workers)
    if value <= 0:
        raise ValueError("num_workers must be a positive integer or None")
    return value


def _normalize_num_cpus(num_cpus: int | None) -> int | None:
    if num_cpus is None:
        return None
    value = int(num_cpus)
    if value <= 0:
        raise ValueError("num_cpus must be a positive integer or None")
    return value


def set_worker_limit(num_workers: int | None) -> None:
    """Set the process-wide pgo worker limit.

    This limits pgo's outer worker scheduler. Nested native kernels are handled
    by the per-call nested-kernel policy in native code, not by this numeric
    limit. ``None`` restores automatic scheduling.

    Changing the limit is process-wide and must not race with native work or
    another thread changing the limit.
    """

    value = _normalize_num_workers(num_workers)
    if value is None:
        _core._parallel_reset_worker_limit()
    else:
        _core._parallel_set_worker_limit(value)


def get_worker_limit() -> int | None:
    """Return the requested process-wide limit, or ``None`` for automatic."""

    value = _core._parallel_get_worker_limit()
    if value is None:
        return None
    return int(value)


def runtime_info() -> dict[str, int | None]:
    """Return native CPU runtime limits observed by libpgo."""

    return dict(_core._parallel_runtime_info())


def supports_cpu_affinity_limit() -> bool:
    """Return whether OS-level CPU affinity limits are available."""

    return bool(_core._parallel_supports_cpu_affinity_limit())


def set_cpu_affinity_limit(num_cpus: int | None) -> None:
    """Set an OS-level CPU affinity limit for the current process.

    This is a hard safety guard: it restricts libpgo and native library worker
    threads to at most ``num_cpus`` CPUs from the process's initial affinity
    mask. It limits where threads may run; it does not prevent a backend from
    creating more OS threads.
    """

    value = _normalize_num_cpus(num_cpus)
    if value is None:
        _core._parallel_reset_cpu_affinity_limit()
    else:
        _core._parallel_set_cpu_affinity_limit(value)


def get_cpu_affinity_limit() -> int | None:
    """Return the requested CPU affinity limit, or ``None`` if unset."""

    value = _core._parallel_get_cpu_affinity_limit()
    if value is None:
        return None
    return int(value)


class _WorkerLimit:
    def __init__(self, num_workers: int | None):
        self._num_workers = _normalize_num_workers(num_workers)
        self._previous: int | None = None

    def __enter__(self) -> None:
        self._previous = get_worker_limit()
        set_worker_limit(self._num_workers)
        return None

    def __exit__(self, exc_type, exc, tb) -> bool:
        set_worker_limit(self._previous)
        return False


def worker_limit(num_workers: int | None) -> Iterator[None]:
    """Temporarily override the process-wide pgo worker limit."""

    return _WorkerLimit(num_workers)


class _CpuAffinityLimit:
    def __init__(self, num_cpus: int | None):
        self._num_cpus = _normalize_num_cpus(num_cpus)
        self._previous: int | None = None

    def __enter__(self) -> None:
        self._previous = get_cpu_affinity_limit()
        set_cpu_affinity_limit(self._num_cpus)
        return None

    def __exit__(self, exc_type, exc, tb) -> bool:
        set_cpu_affinity_limit(self._previous)
        return False


def cpu_affinity_limit(num_cpus: int | None) -> Iterator[None]:
    """Temporarily apply an OS-level CPU affinity safety guard."""

    return _CpuAffinityLimit(num_cpus)


__all__ = [
    "cpu_affinity_limit",
    "get_cpu_affinity_limit",
    "get_worker_limit",
    "runtime_info",
    "set_cpu_affinity_limit",
    "set_worker_limit",
    "supports_cpu_affinity_limit",
    "worker_limit",
]
