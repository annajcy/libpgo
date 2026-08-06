"""Logging controls for the C++ backend.

The C++ solver/mesh backend logs through spdlog to stdout. These helpers make
the log level controllable from Python, e.g. to keep notebook outputs tidy:

    pypgo.set_log_level("off")        # silence the C++ backend
    with pypgo.quiet_cpp_logs():      # or temporarily, and restore after
        ...solve...
"""

from __future__ import annotations

from contextlib import contextmanager

import pypgo._core as _core


LEVELS = ("trace", "debug", "info", "warn", "error", "critical", "off")


def set_log_level(level: str) -> None:
    """Set the C++ backend log level (``trace``..``off``)."""
    if level not in LEVELS:
        raise ValueError(
            f"invalid log level {level!r}; expected one of {', '.join(LEVELS)}")
    _core.set_log_level(level)


def get_log_level() -> str:
    """Return the current C++ backend log level."""
    return str(_core.get_log_level())


@contextmanager
def quiet_cpp_logs():
    """Temporarily silence the C++ backend, restoring the previous level."""
    previous = get_log_level()
    set_log_level("off")
    try:
        yield
    finally:
        set_log_level(previous)


__all__ = ["LEVELS", "get_log_level", "quiet_cpp_logs", "set_log_level"]
