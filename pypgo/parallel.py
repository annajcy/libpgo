"""Process-wide TBB concurrency configuration.

The configured ceiling applies to pgo's TBB scheduling facade and oneMKL only when oneMKL uses
the TBB threading layer. Configure at an application quiescence boundary: changing the value does
not preempt work that is already running.
"""

from __future__ import annotations

import operator

import pypgo._core as _core


def _normalize_max_concurrency(max_concurrency: int | None) -> int | None:
    if max_concurrency is None:
        return None
    if isinstance(max_concurrency, bool):
        raise TypeError("max_concurrency must be an integer or None")
    try:
        value = operator.index(max_concurrency)
    except TypeError as error:
        raise TypeError("max_concurrency must be an integer or None") from error
    if value <= 0:
        raise ValueError("max_concurrency must be a positive integer or None")
    return value


def initialize(*, max_concurrency: int | None = None) -> int:
    """Set the process TBB ceiling and return the effective active concurrency.

    Passing ``None`` restores oneTBB's default concurrency. This function is intentionally
    repeatable; concurrent external TBB controls may make the returned effective value smaller
    than the requested value.
    """

    return int(_core._parallel_initialize(_normalize_max_concurrency(max_concurrency)))


__all__ = ["initialize"]
