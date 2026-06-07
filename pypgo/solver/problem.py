"""Optimization problem definition and variable bounds."""

from __future__ import annotations

from typing import Sequence

import numpy as np

import pypgo._core as _core
from pypgo._arrays import float_vector
from pypgo.energy import PotentialEnergy


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
