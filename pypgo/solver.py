"""Optimization problem and optimizer entry points for pypgo energies."""

from __future__ import annotations

from dataclasses import dataclass, replace
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


@dataclass(frozen=True)
class SolveDiagnostics:
    min_feasible_alpha: float = 1.0
    min_line_search_alpha: float = 1.0
    min_effective_alpha: float = 1.0
    material_clamp_count: int = 0
    contact_clamp_count: int = 0
    final_gradient_norm: float | None = None
    final_gradient_max_norm: float | None = None


@dataclass(frozen=True)
class SolverResult:
    x: np.ndarray
    status: SolveStatus
    converged: bool
    iterations: int
    raw_status_code: int
    final_objective: float | None
    final_gradient_norm: float | None
    final_gradient_max_norm: float | None
    diagnostics: SolveDiagnostics


_SPARSE_SOLVERS = {
    "auto": 0,
    "eigen_ldlt": 1,
    "pardiso": 2,
    "orig_pardiso": 3,
}


_LINE_SEARCH_METHODS = {"golden", "brents", "backtrack", "simple"}


def _as_fixed_dofs(fixed_dofs: Sequence[int] | None):
    if fixed_dofs is None:
        return None
    return [int(dof) for dof in fixed_dofs]


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


@dataclass(frozen=True)
class Bounds:
    lower: np.ndarray | None = None
    upper: np.ndarray | None = None


class OptimizationProblem:
    """Mathematical nonlinear optimization problem.

    The problem describes only the objective and mathematical bounds. Solver
    choices and Newton/Ipopt/Knitro options live on optimizer objects.
    """

    def __init__(self, *, objective) -> None:
        if not isinstance(objective, PotentialEnergy):
            raise TypeError("objective must be a pypgo.energy.PotentialEnergy")
        self.objective = objective
        self.variable_bounds = Bounds()

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


@dataclass(kw_only=True)
class NewtonOptimizer:
    """Newton optimizer with backend-specific options."""

    max_iterations: int = 50
    gradient_tolerance: float = 1e-6
    damping: bool = True
    line_search: str = "backtrack"
    verbose: int = 0
    sparse_solver: str = "auto"

    def __post_init__(self) -> None:
        if self.sparse_solver not in _SPARSE_SOLVERS:
            raise ValueError(f"sparse_solver must be one of {list(_SPARSE_SOLVERS)}, got {self.sparse_solver!r}")
        if self.line_search not in _LINE_SEARCH_METHODS:
            raise ValueError(f"line_search must be one of {sorted(_LINE_SEARCH_METHODS)}, got {self.line_search!r}")

        self.max_iterations = int(self.max_iterations)
        self.gradient_tolerance = float(self.gradient_tolerance)
        self.damping = bool(self.damping)
        self.line_search = str(self.line_search)
        self.verbose = int(self.verbose)
        self.sparse_solver = str(self.sparse_solver)

    def solve(self, problem: OptimizationProblem, x0: np.ndarray | Sequence[float]) -> SolverResult:
        if not isinstance(problem, OptimizationProblem):
            raise TypeError("problem must be an OptimizationProblem")

        x0_arr = float_vector("x0", x0)
        lower = (
            np.empty(0, dtype=np.float64)
            if problem.variable_bounds.lower is None
            else float_vector("variable_bounds.lower", problem.variable_bounds.lower)
        )
        upper = (
            np.empty(0, dtype=np.float64)
            if problem.variable_bounds.upper is None
            else float_vector("variable_bounds.upper", problem.variable_bounds.upper)
        )

        data = _core._newton_optimizer_solve(
            problem.objective._handle,
            x0_arr,
            lower,
            problem.variable_bounds.lower is not None,
            upper,
            problem.variable_bounds.upper is not None,
            self.max_iterations,
            self.gradient_tolerance,
            self.damping,
            self.line_search,
            self.verbose,
            _SPARSE_SOLVERS[self.sparse_solver],
        )
        return _result_from_core(data)


def solve_newton(
    energy,
    x0: np.ndarray,
    *,
    options: NewtonOptimizer | None = None,
    max_iter: int | None = None,
    tol: float | None = None,
    fixed_dofs: Sequence[int] | None = None,
    fixed_values: np.ndarray | Sequence[float] | None = None,
    damping: bool | None = None,
    line_search: str | None = None,
    verbose: int | None = None,
    sparse_solver: str | None = None,
) -> SolverResult:
    """Compatibility shim for the object-style Newton API."""
    if not isinstance(energy, PotentialEnergy):
        raise TypeError("energy must be a pypgo.energy.PotentialEnergy")
    if options is not None and not isinstance(options, NewtonOptimizer):
        raise TypeError("options must be a NewtonOptimizer instance")

    x0_arr = float_vector("x0", x0)
    overrides = {
        "max_iterations": max_iter,
        "gradient_tolerance": tol,
        "damping": damping,
        "line_search": line_search,
        "verbose": verbose,
        "sparse_solver": sparse_solver,
    }
    optimizer = replace(
        options or NewtonOptimizer(),
        **{key: value for key, value in overrides.items() if value is not None},
    )

    problem = OptimizationProblem(objective=energy)
    fixed = _as_fixed_dofs(fixed_dofs) or []
    if fixed:
        values = float_vector("fixed_values", fixed_values) if fixed_values is not None else x0_arr[fixed]
        problem.fix_variables(fixed, values, num_dofs=x0_arr.size)

    return optimizer.solve(problem, x0_arr)


__all__ = [
    "SolveStatus",
    "SolveDiagnostics",
    "SolverResult",
    "Bounds",
    "OptimizationProblem",
    "NewtonOptimizer",
    "solve_newton",
]
