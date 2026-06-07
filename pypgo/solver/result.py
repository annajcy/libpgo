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

    @classmethod
    def from_dict(cls, data: dict) -> "SolveDiagnostics":
        known = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in data.items() if k in known})


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
