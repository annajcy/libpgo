"""Runtime controls for libpgo's internal parallel loops."""

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


def set_num_threads(num_threads: int | None) -> None:
    """Set the default worker limit for libpgo's ``pgo::parallel`` loops.

    ``None`` resets to automatic backend defaults. This does not promise to
    control third-party thread pools such as BLAS, Eigen, libigl, or OpenVDB.
    """

    value = _normalize_num_threads(num_threads)
    if value is None:
        _core._parallel_reset_num_threads()
    else:
        _core._parallel_set_num_threads(value)


def get_num_threads() -> int | None:
    """Return the current libpgo default worker limit, or ``None`` for auto."""

    value = _core._parallel_get_num_threads()
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
    """Temporarily override the libpgo default worker limit."""

    return _ThreadLimit(num_threads)


__all__ = ["get_num_threads", "set_num_threads", "thread_limit"]
