"""Abstract optimizer base class and the shared solve flow."""

from __future__ import annotations

from typing import Sequence

import numpy as np

import pypgo._core as _core
from pypgo._utils import float_vector
from pypgo.solver.problem import OptimizationProblem
from pypgo.solver.result import SolverResult, _result_from_core


class Optimizer:
    """Abstract base for all pypgo optimizers.

    Each concrete optimizer holds a C++ peer via ``_handle``.  The
    :meth:`solve` method accepts an :class:`OptimizationProblem` and an
    initial guess and returns a :class:`SolverResult`.
    """

    def __init__(self, handle) -> None:
        if not isinstance(handle, _core.PyOptimizer):
            raise TypeError(f"handle must be a _core.PyOptimizer, got {type(handle).__name__}")
        self._handle = handle

    def solve(self, problem: OptimizationProblem, x0: np.ndarray | Sequence[float]) -> SolverResult:
        if not isinstance(problem, OptimizationProblem):
            raise TypeError("problem must be an OptimizationProblem")
        problem._sync_variable_bounds_to_handle()
        x0_arr = float_vector("x0", x0)
        return _result_from_core(self._handle.solve(problem._handle, x0_arr))
