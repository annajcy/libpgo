"""Process-wide CPU concurrency controls for libpgo."""

from __future__ import annotations

from collections.abc import Iterator

import pypgo._core as _core


def _normalize_num_threads(num_threads: int | None) -> int | None:
    if num_threads is None:
        return None
    value = int(num_threads)
    if value <= 0:
        raise ValueError("num_threads must be a positive integer or None")
    return value


def _normalize_num_cpus(num_cpus: int | None) -> int | None:
    if num_cpus is None:
        return None
    value = int(num_cpus)
    if value <= 0:
        raise ValueError("num_cpus must be a positive integer or None")
    return value


def set_num_threads(num_threads: int | None) -> None:
    """Set the process-wide CPU concurrency limit for libpgo.

    The limit covers libpgo's TBB, Eigen, OpenMP, and supported MKL/OpenBLAS
    runtimes. Because NumPy and SciPy can share the same BLAS runtime, they may
    inherit this limit too. ``None`` restores the runtime defaults observed
    before the first limit was set.

    Changing the limit is process-wide and must not race with native work or
    another thread changing the limit.
    """

    value = _normalize_num_threads(num_threads)
    if value is None:
        _core._parallel_reset_num_threads()
    else:
        _core._parallel_set_num_threads(value)


def get_num_threads() -> int | None:
    """Return the requested process-wide limit, or ``None`` for automatic."""

    value = _core._parallel_get_num_threads()
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


class _ThreadLimit:
    def __init__(self, num_threads: int | None):
        self._num_threads = _normalize_num_threads(num_threads)
        self._previous: int | None = None

    def __enter__(self) -> None:
        self._previous = get_num_threads()
        set_num_threads(self._num_threads)
        return None

    def __exit__(self, exc_type, exc, tb) -> bool:
        set_num_threads(self._previous)
        return False


def thread_limit(num_threads: int | None) -> Iterator[None]:
    """Temporarily override the process-wide libpgo concurrency limit."""

    return _ThreadLimit(num_threads)


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
    "get_num_threads",
    "runtime_info",
    "set_cpu_affinity_limit",
    "set_num_threads",
    "supports_cpu_affinity_limit",
    "thread_limit",
]
