"""Process-wide profiling controls for libpgo."""

from __future__ import annotations

import pypgo._core as _core


def set_enabled(enabled: bool) -> None:
    """Enable or disable collection for native profiling sections."""

    _core._profiling_set_enabled(bool(enabled))


def is_enabled() -> bool:
    """Return whether native profiling collection is enabled."""

    return bool(_core._profiling_is_enabled())


def reset() -> None:
    """Clear collected native profiling statistics and counters."""

    _core._profiling_reset()


def snapshot() -> dict:
    """Return collected section timing statistics as a nested tree."""

    return dict(_core._profiling_stats())


def snapshot_counters() -> list[dict]:
    """Return collected profiling counters."""

    return list(_core._profiling_counter_stats())
