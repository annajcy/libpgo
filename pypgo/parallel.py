"""Explicit oneTBB global control and persistent current-thread BLAS policy."""

from __future__ import annotations

import operator

import pypgo._core as _core


AccelerateThreading = _core._AccelerateThreading


def _normalize_max_concurrency(max_concurrency: int) -> int:
    if isinstance(max_concurrency, bool):
        raise TypeError("max_concurrency must be an integer")
    try:
        value = operator.index(max_concurrency)
    except TypeError as error:
        raise TypeError("max_concurrency must be an integer") from error
    if value <= 0:
        raise ValueError("max_concurrency must be a positive integer")
    return value


class GlobalTbbControl:
    """RAII owner of oneTBB's process-wide maximum-parallelism control."""

    __slots__ = ("_control",)

    def __init__(self, max_allowed_parallelism: int) -> None:
        value = _normalize_max_concurrency(max_allowed_parallelism)
        self._control = _core._GlobalTbbControl(value)

    def close(self) -> None:
        """Release this control. Calling ``close`` repeatedly is harmless."""

        self._control = None

    def __enter__(self) -> GlobalTbbControl:
        if self._control is None:
            raise RuntimeError("GlobalTbbControl is already closed")
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


class ArenaThreadingExecutor:
    """Private oneTBB arena with a scoped BLAS TLS policy per participant."""

    __slots__ = ("_executor",)

    def __init__(
        self,
        max_concurrency: int,
        *,
        reserved_slots: int = 1,
        mkl_local_thread_budget: int = 1,
        accelerate: AccelerateThreading = AccelerateThreading.SINGLE,
    ) -> None:
        max_concurrency = _normalize_max_concurrency(max_concurrency)
        if isinstance(reserved_slots, bool):
            raise TypeError("reserved_slots must be an integer")
        try:
            reserved_slots = operator.index(reserved_slots)
        except TypeError as error:
            raise TypeError("reserved_slots must be an integer") from error
        if not 0 <= reserved_slots <= max_concurrency:
            raise ValueError(
                "reserved_slots must be between zero and max_concurrency"
            )
        if isinstance(mkl_local_thread_budget, bool):
            raise TypeError("mkl_local_thread_budget must be an integer")
        try:
            mkl_local_thread_budget = operator.index(
                mkl_local_thread_budget
            )
        except TypeError as error:
            raise TypeError(
                "mkl_local_thread_budget must be an integer"
            ) from error
        if mkl_local_thread_budget < 0:
            raise ValueError("mkl_local_thread_budget must be non-negative")

        self._executor = _core._ArenaThreadingExecutor(
            max_concurrency,
            reserved_slots,
            mkl_local_thread_budget,
            accelerate,
        )

    def execute(self, fn):
        """Synchronously invoke ``fn`` inside the controlled arena."""

        if not callable(fn):
            raise TypeError("fn must be callable")
        return self._executor.execute(fn)


def set_threading_policy(
    *,
    mkl_local_thread_budget: int | None = None,
    accelerate: AccelerateThreading | None = None,
) -> None:
    """Persistently overwrite active BLAS backend TLS on the calling OS thread.

    ``mkl_local_thread_budget=0`` clears the oneMKL thread-local override and follows
    the MKL global setting. No previous state is restored automatically.
    """

    if mkl_local_thread_budget is not None:
        if isinstance(mkl_local_thread_budget, bool):
            raise TypeError("mkl_local_thread_budget must be an integer or None")
        try:
            mkl_local_thread_budget = operator.index(mkl_local_thread_budget)
        except TypeError as error:
            raise TypeError(
                "mkl_local_thread_budget must be an integer or None"
            ) from error
        if mkl_local_thread_budget < 0:
            raise ValueError("mkl_local_thread_budget must be non-negative")

    _core._parallel_set_threading_policy(
        mkl_local_thread_budget, accelerate
    )


__all__ = [
    "AccelerateThreading",
    "ArenaThreadingExecutor",
    "GlobalTbbControl",
    "set_threading_policy",
]
