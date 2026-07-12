"""One-time process parallel runtime configuration and diagnostics.

Participant counters cover pgo TBB arenas, not CPU utilization or work from
OpenMP and other native runtimes. Avoid nesting OpenMP-backed APIs inside pgo
parallel work because the runtimes do not coordinate their worker limits.
"""

from __future__ import annotations

from dataclasses import dataclass
import operator
from typing import Any

import pypgo._core as _core


@dataclass(frozen=True, slots=True)
class RuntimeInfo:
    """Immutable snapshot of pgo runtime configuration and arena participation."""

    initialized: bool
    using_default_concurrency: bool
    max_concurrency: int | None
    default_concurrency: int
    effective_tbb_max_allowed_parallelism: int
    tbb_worker_ceiling: int
    current_worker_participants: int
    current_external_participants: int
    current_total_participants: int
    peak_total_participants: int
    participant_pressure_observed: bool


def _runtime_info(raw: dict[str, Any]) -> RuntimeInfo:
    return RuntimeInfo(**raw)


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


def default_concurrency() -> int:
    """Return the current oneTBB default without initializing the pgo runtime."""

    return int(_core._parallel_default_concurrency())


def initialize(*, max_concurrency: int | None = None) -> RuntimeInfo:
    """Initialize the process runtime once and return its current snapshot."""

    value = _normalize_max_concurrency(max_concurrency)
    return _runtime_info(dict(_core._parallel_initialize(value)))


def runtime_info() -> RuntimeInfo:
    """Return a snapshot without initializing the pgo runtime."""

    return _runtime_info(dict(_core._parallel_runtime_info()))


__all__ = [
    "RuntimeInfo",
    "default_concurrency",
    "initialize",
    "runtime_info",
]
