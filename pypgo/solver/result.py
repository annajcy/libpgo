"""Solve status, diagnostics, and result types for pypgo optimizers."""

from __future__ import annotations

from dataclasses import dataclass, fields
from enum import IntEnum

import numpy as np


class SolveStatus(IntEnum):
    CONVERGED = 0
    MAX_ITERATIONS = 1
    LINE_SEARCH_FAILED = 2
    STEP_TOO_SMALL = 3
    NON_FINITE = 4
    LINEAR_SOLVE_FAILED = 5
    EXTERNAL_SOLVER_FAILURE = 100
    UNSUPPORTED_BACKEND = 101


class NewtonConvergenceReason(IntEnum):
    NONE = 0
    ABSOLUTE_GRADIENT = 1
    RELATIVE_GRADIENT = 2
    TERMINATION_POLICY = 3
    ABSOLUTE_GRADIENT_FP_LIMIT = 4
    LOOSE_RELATIVE_GRADIENT_FP_LIMIT = 5


@dataclass(frozen=True, slots=True)
class SolveDiagnostics:
    """Diagnostics from the most recent solve step.

    Built from the C++ solve-result dictionary via :meth:`from_dict`, which
    keeps only recognised keys so the solver peer layer stays free to add or
    omit fields without breaking construction.
    """

    min_feasible_alpha: float | None = None
    min_line_search_alpha: float | None = None
    min_effective_alpha: float | None = None
    material_clamp_count: int | None = None
    contact_clamp_count: int | None = None
    final_gradient_norm: float | None = None
    final_gradient_max_norm: float | None = None
    last_grad_dot_dx: float | None = None
    last_raw_step_max_norm: float | None = None
    last_raw_step_norm: float | None = None
    last_accepted_step_max_norm: float | None = None
    last_line_search_iterations: int | None = None
    max_line_search_iterations: int | None = None
    total_line_search_iterations: int | None = None
    last_current_energy: float | None = None
    last_accepted_energy: float | None = None
    last_energy_delta: float | None = None
    last_damping_value: float | None = None
    last_active_system_nnz: int | None = None
    last_active_system_rows: int | None = None
    last_active_system_cols: int | None = None
    linear_solver_symbolic_rebuild_count: int | None = None
    linear_solver_symbolic_reuse_count: int | None = None
    newton_convergence_reason: NewtonConvergenceReason | None = None
    newton_convergence_reason_name: str | None = None
    newton_convergence_threshold: float | None = None
    newton_low_value_iteration_count: int | None = None
    newton_tiny_step_count: int | None = None
    newton_small_alpha_count: int | None = None
    newton_symbolic_rebuild_count: int | None = None
    newton_worst_progress_iteration: int | None = None
    newton_worst_progress_ratio: float | None = None
    optimizer_preparation_seconds: float | None = None
    newton_solver_setup_seconds: float | None = None
    initial_hessian_seconds: float | None = None
    initial_reduced_system_seconds: float | None = None
    initial_symbolic_analyze_seconds: float | None = None
    newton_solve_seconds: float | None = None
    newton_total_iteration_seconds: float | None = None
    newton_total_evaluate_current_state_seconds: float | None = None
    newton_total_func_grad_hessian_seconds: float | None = None
    newton_total_prepare_reduced_system_seconds: float | None = None
    newton_total_ensure_linear_solver_seconds: float | None = None
    newton_total_symbolic_analyze_seconds: float | None = None
    newton_total_factorize_seconds: float | None = None
    newton_total_solve_seconds: float | None = None
    newton_total_expand_reduced_step_seconds: float | None = None
    newton_total_line_search_seconds: float | None = None
    final_objective_seconds: float | None = None
    linear_solver_cleanup_seconds: float | None = None
    optimizer_total_seconds: float | None = None
    newton_iterations: list[dict] | None = None

    @classmethod
    def from_dict(cls, data: dict) -> "SolveDiagnostics":
        known = {f.name for f in fields(cls)}
        values = {k: v for k, v in data.items() if k in known}
        if values.get("newton_convergence_reason") is not None:
            values["newton_convergence_reason"] = NewtonConvergenceReason(
                int(values["newton_convergence_reason"])
            )
        return cls(**values)


@dataclass(frozen=True, slots=True)
class SolverResult:
    """Result of a single call to :meth:`Optimizer.solve`."""

    x: np.ndarray
    status: SolveStatus
    converged: bool
    iterations: int
    raw_status_code: int
    final_objective: float | None
    final_gradient_norm: float | None
    final_gradient_max_norm: float | None
    diagnostics: SolveDiagnostics


def _result_from_core(data: dict) -> SolverResult:
    return SolverResult(
        x=np.asarray(data["x"], dtype=np.float64),
        status=SolveStatus(int(data["status"])),
        converged=bool(data["converged"]),
        iterations=int(data["iterations"]),
        raw_status_code=int(data["raw_status_code"]),
        final_objective=data["final_objective"],
        final_gradient_norm=data["final_gradient_norm"],
        final_gradient_max_norm=data["final_gradient_max_norm"],
        diagnostics=SolveDiagnostics.from_dict(data["diagnostics"]),
    )
