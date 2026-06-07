"""Implicit field base class, grid-sampled fields, and CSG operations.

``ImplicitField`` is the stable contract for all fields (eval / bounds /
sampling / CSG).  ``GridField`` is the grid-sampled result; it is coupled to
``ImplicitField.sample_to_grid`` and so lives here rather than with the shape
primitives in ``fields.py``.
"""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo._utils import vec3
from pypgo.implicit.grid import GridSpec


class ImplicitField:
    def __init__(self, core_obj):
        self._handle = core_obj

    @classmethod
    def _from_core(cls, core_obj):
        obj = object.__new__(cls)
        obj._handle = core_obj
        return obj

    def eval(self, p) -> float:
        return float(self._handle.eval(vec3("p", p)))

    def bounds(self):
        result = self._handle.bounds()
        if result is None:
            return None
        return tuple(np.asarray(v, dtype=np.float64) for v in result)

    def sample_to_grid(self, grid_spec: GridSpec, *, num_threads: int | None = None) -> "GridField":
        if not isinstance(grid_spec, GridSpec):
            raise TypeError(f"grid_spec must be GridSpec, got {type(grid_spec).__name__}")
        if num_threads is None:
            core_num_threads = 0
        else:
            core_num_threads = int(num_threads)
            if core_num_threads <= 0:
                raise ValueError("num_threads must be a positive integer or None")
        return GridField(self._handle.sample_to_grid(grid_spec._handle, core_num_threads))

    def __or__(self, other: "ImplicitField") -> "ImplicitField":
        return ImplicitField(_core.implicit_union(self._handle, _field_core(other)))

    def __and__(self, other: "ImplicitField") -> "ImplicitField":
        return ImplicitField(_core.implicit_intersection(self._handle, _field_core(other)))

    def __sub__(self, other: "ImplicitField") -> "ImplicitField":
        return ImplicitField(_core.implicit_difference(self._handle, _field_core(other)))

    def offset(self, value: float) -> "ImplicitField":
        return ImplicitField(_core.implicit_offset(self._handle, float(value)))


def _field_core(value: ImplicitField):
    if not isinstance(value, ImplicitField):
        raise TypeError(f"value must be ImplicitField, got {type(value).__name__}")
    return value._handle


class GridField(ImplicitField):
    @property
    def values(self) -> np.ndarray:
        return np.asarray(self._handle)

    @property
    def grid_spec(self) -> GridSpec:
        return GridSpec._from_core(self._handle.grid_spec())
