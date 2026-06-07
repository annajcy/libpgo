"""Optimization problem and optimizer entry points for pypgo energies."""

from __future__ import annotations

from enum import IntEnum
from typing import Sequence

import numpy as np

import pypgo._core as _core
from pypgo._arrays import float_vector
from pypgo.energy import PotentialEnergy


class SolveStatus(IntEnum):
    CONVERGED = 0
    MAX_ITERATIONS = 1
    LINE_SEARCH_FAILED = 2
    STEP_TOO_SMALL = 3
    NON_FINITE = 4
    LINEAR_SOLVE_FAILED = 5
    EXTERNAL_SOLVER_FAILURE = 100
    UNSUPPORTED_BACKEND = 101


class SolveDiagnostics:
    """Diagnostics from the most recent solve step.

    Constructed from the C++ solve result dictionary; intentionally not
    a frozen dataclass so the solver peer layer controls the shape.
    """

    __slots__ = (
        "min_feasible_alpha",
        "min_line_search_alpha",
        "min_effective_alpha",
        "material_clamp_count",
        "contact_clamp_count",
        "final_gradient_norm",
        "final_gradient_max_norm",
    )

    def __init__(self, **kwargs):
        for k in self.__slots__:
            object.__setattr__(self, k, kwargs.get(k))


class SolverResult:
    """Result of a single call to :meth:`Optimizer.solve`."""

    __slots__ = (
        "x",
        "status",
        "converged",
        "iterations",
        "raw_status_code",
        "final_objective",
        "final_gradient_norm",
        "final_gradient_max_norm",
        "diagnostics",
    )

    def __init__(self, **kwargs):
        for k in self.__slots__:
            object.__setattr__(self, k, kwargs.get(k))


_SPARSE_SOLVERS = {
    "auto": 0,
    "eigen_ldlt": 1,
    "pardiso": 2,
    "orig_pardiso": 3,
}

_LINE_SEARCH_METHODS = {"golden", "brents", "backtrack", "simple"}


def _result_from_core(data: dict) -> SolverResult:
    diagnostics = SolveDiagnostics(**data["diagnostics"])
    return SolverResult(
        x=np.asarray(data["x"], dtype=np.float64),
        status=SolveStatus(int(data["status"])),
        converged=bool(data["converged"]),
        iterations=int(data["iterations"]),
        raw_status_code=int(data["raw_status_code"]),
        final_objective=data["final_objective"],
        final_gradient_norm=data["final_gradient_norm"],
        final_gradient_max_norm=data["final_gradient_max_norm"],
        diagnostics=diagnostics,
    )


class Bounds:
    """Lower/upper variable bounds for :class:`OptimizationProblem`."""

    __slots__ = ("lower", "upper")

    def __init__(self, lower=None, upper=None):
        self.lower = lower
        self.upper = upper


class OptimizationProblem:
    """Mathematical nonlinear optimization problem.

    The problem describes the objective and mathematical bounds.  Solver
    choices live on :class:`Optimizer` objects.  Variable bounds are
    editable on the Python side and synced to C++ before each solve.
    """

    def __init__(self, *, objective) -> None:
        if not isinstance(objective, PotentialEnergy):
            raise TypeError("objective must be a pypgo.energy.PotentialEnergy")
        self.objective = objective
        self.variable_bounds = Bounds()
        self._handle = _core._create_optimization_problem(objective._handle)

    def _sync_variable_bounds_to_handle(self) -> None:
        lower = (
            np.empty(0, dtype=np.float64)
            if self.variable_bounds.lower is None
            else float_vector("variable_bounds.lower", self.variable_bounds.lower)
        )
        upper = (
            np.empty(0, dtype=np.float64)
            if self.variable_bounds.upper is None
            else float_vector("variable_bounds.upper", self.variable_bounds.upper)
        )
        self._handle.set_variable_bounds(
            lower,
            self.variable_bounds.lower is not None,
            upper,
            self.variable_bounds.upper is not None,
        )

    def fix_variables(
        self,
        dofs: Sequence[int],
        values: np.ndarray | Sequence[float],
        *,
        num_dofs: int | None = None,
    ) -> None:
        dof_list = [int(dof) for dof in dofs]
        value_arr = float_vector("values", values)
        if value_arr.size != len(dof_list):
            raise ValueError("values size must match dofs size")
        if len(set(dof_list)) != len(dof_list):
            raise ValueError("fixed dofs must be unique")
        if num_dofs is None:
            num_dofs = int(self.objective.num_dofs)

        lower = (
            np.full(num_dofs, -np.inf, dtype=np.float64)
            if self.variable_bounds.lower is None
            else float_vector("variable_bounds.lower", self.variable_bounds.lower).copy()
        )
        upper = (
            np.full(num_dofs, np.inf, dtype=np.float64)
            if self.variable_bounds.upper is None
            else float_vector("variable_bounds.upper", self.variable_bounds.upper).copy()
        )
        if lower.size != num_dofs or upper.size != num_dofs:
            raise ValueError("variable bounds size must match num_dofs")

        for dof, value in zip(dof_list, value_arr):
            if dof < 0 or dof >= num_dofs:
                raise ValueError("fixed dof out of range")
            lower[dof] = value
            upper[dof] = value

        self.variable_bounds = Bounds(lower=lower, upper=upper)


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


class NewtonOptimizer(Optimizer):
    """Newton optimizer with backend-specific options.

    Parameters are validated at construction time and used to build the
    C++ peer once.  The peer is immutable after construction; to change
    options, create a new ``NewtonOptimizer``.
    """

    def __init__(self, *, max_iterations=50, gradient_tolerance=1e-6,
                 damping=True, line_search="backtrack", verbose=0,
                 sparse_solver="auto"):
        if sparse_solver not in _SPARSE_SOLVERS:
            raise ValueError(f"sparse_solver must be one of {list(_SPARSE_SOLVERS)}, got {sparse_solver!r}")
        if line_search not in _LINE_SEARCH_METHODS:
            raise ValueError(f"line_search must be one of {sorted(_LINE_SEARCH_METHODS)}, got {line_search!r}")

        options = _core.PyNewtonOptimizerOptions()
        options.max_iterations = int(max_iterations)
        options.gradient_tolerance = float(gradient_tolerance)
        options.damping = bool(damping)
        options.line_search = str(line_search)
        options.verbose = int(verbose)
        options.sparse_solver_kind = _SPARSE_SOLVERS[sparse_solver]
        super().__init__(_core.PyNewtonOptimizer(options))


__all__ = [
    "Bounds",
    "NewtonOptimizer",
    "OptimizationProblem",
    "Optimizer",
    "SolveDiagnostics",
    "SolveStatus",
    "SolverResult",
]
