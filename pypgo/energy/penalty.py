"""Soft penalty energies derived from hard constraint functions."""

from __future__ import annotations

import pypgo._core as _core
from pypgo.constraints import Bounded, ConstraintFunction
from pypgo.energy.base import PotentialEnergy


class ConstraintPenalty(PotentialEnergy):
    """Soft zero-residual penalty ``0.5 * weight * ||C(x)||^2``."""

    def __init__(self, constraints, weight=1.0):
        if not isinstance(constraints, ConstraintFunction):
            raise TypeError("constraints must be a pypgo.constraints.ConstraintFunction")
        object.__setattr__(self, "_constraints", constraints)
        object.__setattr__(self, "weight", float(weight))
        super().__init__(_core._create_constraint_penalty(constraints._handle, self.weight))

    @property
    def constraints(self):
        return self._constraints

    def __repr__(self) -> str:
        return f"ConstraintPenalty({self.num_dofs} DOFs, weight={self.weight:g})"


class ConstraintViolationPenalty(PotentialEnergy):
    """Soft penalty for bound violations in ``lower <= C(x) <= upper``."""

    def __init__(self, bounded, weight=1.0):
        if not isinstance(bounded, Bounded):
            raise TypeError("bounded must be a pypgo.constraints.Bounded")
        object.__setattr__(self, "_bounded", bounded)
        object.__setattr__(self, "weight", float(weight))
        super().__init__(
            _core._create_constraint_violation_penalty(
                bounded.functions._handle,
                bounded.lower,
                bounded.upper,
                self.weight,
            )
        )

    @property
    def bounded(self):
        return self._bounded

    def __repr__(self) -> str:
        return f"ConstraintViolationPenalty({self.num_dofs} DOFs, weight={self.weight:g})"
