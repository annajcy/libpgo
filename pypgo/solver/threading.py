"""Semantic threading policy for Newton solver phases."""

from __future__ import annotations

from pypgo.parallel import ArenaThreadingExecutor


class NewtonThreadingPolicy:
    """Map Newton evaluation and sparse-linear-solver phases to executors.

    The policy declares how phases execute; the Newton state machine and phase
    ordering remain implemented in C++. Both executors are retained for the
    lifetime of this policy and may be reused by repeated ``solve()`` calls.
    """

    __slots__ = ("_evaluation", "_linear_solver")

    def __init__(
        self,
        *,
        evaluation: ArenaThreadingExecutor,
        linear_solver: ArenaThreadingExecutor,
    ) -> None:
        if not isinstance(evaluation, ArenaThreadingExecutor):
            raise TypeError("evaluation must be an ArenaThreadingExecutor")
        if not isinstance(linear_solver, ArenaThreadingExecutor):
            raise TypeError("linear_solver must be an ArenaThreadingExecutor")
        self._evaluation = evaluation
        self._linear_solver = linear_solver

    @property
    def evaluation(self) -> ArenaThreadingExecutor:
        return self._evaluation

    @property
    def linear_solver(self) -> ArenaThreadingExecutor:
        return self._linear_solver


__all__ = ["NewtonThreadingPolicy"]
