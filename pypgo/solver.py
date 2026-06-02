"""Newton solver entry points for pypgo energies."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import IntEnum
from typing import Sequence

import numpy as np

import pypgo._core as _core


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


@dataclass(frozen=True)
class NewtonOptions:
    max_iter: int = 50
    tol: float = 1e-6
    damping: bool = True
    line_search: str = "backtrack"
    verbose: int = 0


def _merge_newton_options(
    options: NewtonOptions | None,
    *,
    max_iter: int | None,
    tol: float | None,
    damping: bool | None,
    line_search: str | None,
    verbose: int | None,
) -> NewtonOptions:
    if options is None:
        options = NewtonOptions()
    elif not isinstance(options, NewtonOptions):
        raise TypeError("options must be a NewtonOptions instance")

    overrides = {}
    if max_iter is not None:
        overrides["max_iter"] = max_iter
    if tol is not None:
        overrides["tol"] = tol
    if damping is not None:
        overrides["damping"] = damping
    if line_search is not None:
        overrides["line_search"] = line_search
    if verbose is not None:
        overrides["verbose"] = verbose

    return replace(options, **overrides) if overrides else options


def _as_float_vector(name: str, arr) -> np.ndarray:
    out = np.asarray(arr, dtype=np.float64, order="C")
    if out.ndim != 1:
        raise ValueError(f"{name} must be 1-D, got shape {out.shape}")
    return out


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


def solve_newton(
    energy,
    x0: np.ndarray,
    *,
    options: NewtonOptions | None = None,
    max_iter: int | None = None,
    tol: float | None = None,
    fixed_dofs: Sequence[int] | None = None,
    fixed_values: np.ndarray | Sequence[float] | None = None,
    damping: bool | None = None,
    line_search: str | None = None,
    verbose: int | None = None,
) -> SolverResult:
    """Minimize a PotentialEnergy with the Newton backend.

    ``x0`` is copied by the C++ service before solving. If ``fixed_values`` is
    ``None``, fixed DOFs are fixed to their corresponding values in ``x0``. Pass
    ``options`` to reuse a ``NewtonOptions`` object; explicit keyword arguments
    override fields from ``options``.
    """
    if not hasattr(energy, "_handle"):
        raise TypeError("energy must be a pypgo.energy PotentialEnergy-compatible object")

    opts = _merge_newton_options(
        options,
        max_iter=max_iter,
        tol=tol,
        damping=damping,
        line_search=line_search,
        verbose=verbose,
    )

    x0_arr = _as_float_vector("x0", x0)
    has_fixed_values = fixed_values is not None
    fixed_values_arr = (
        np.empty(0, dtype=np.float64)
        if fixed_values is None
        else _as_float_vector("fixed_values", fixed_values)
    )

    data = _core._solve_newton(
        energy._handle,
        x0_arr,
        _as_fixed_dofs(fixed_dofs) or [],
        fixed_values_arr,
        has_fixed_values,
        int(opts.max_iter),
        float(opts.tol),
        bool(opts.damping),
        str(opts.line_search),
        int(opts.verbose),
    )
    return _result_from_core(data)


__all__ = [
    "SolveStatus",
    "SolveDiagnostics",
    "SolverResult",
    "NewtonOptions",
    "solve_newton",
]
