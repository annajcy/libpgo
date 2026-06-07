"""Bounds wrapper for constraint functions: ``lower <= functions(x) <= upper``."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from pypgo._arrays import float_vector
from pypgo.constraints.base import ConstraintFunction


def _bounds_vector(name, value, size):
    if np.isscalar(value):
        return np.full(size, float(value), dtype=np.float64)
    arr = float_vector(name, value)
    if arr.shape != (size,):
        raise ValueError(f"{name} must be scalar or shape ({size},), got shape {arr.shape}")
    return arr.copy()


@dataclass(frozen=True, repr=False)
class Bounded:
    """Bounds for ``lower <= functions(x) <= upper``."""

    functions: ConstraintFunction
    lower: np.ndarray = field(kw_only=True)
    upper: np.ndarray = field(kw_only=True)

    def __post_init__(self):
        if not isinstance(self.functions, ConstraintFunction):
            raise TypeError("functions must be a pypgo.constraints.ConstraintFunction")
        object.__setattr__(
            self, "lower", _bounds_vector("lower", self.lower, self.functions.num_constraints))
        object.__setattr__(
            self, "upper", _bounds_vector("upper", self.upper, self.functions.num_constraints))

    def __repr__(self) -> str:
        return f"Bounded({self.functions.num_constraints} constraints)"
