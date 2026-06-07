"""Grid specification for sampling implicit fields."""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo._utils import vec3
from pypgo.mesh import TriMeshData


class GridSpec:
    def __init__(self, bmin, bmax, resolution: int):
        self._handle = _core.PyGridSpec(vec3("bmin", bmin), vec3("bmax", bmax), int(resolution))

    @classmethod
    def _from_core(cls, core_obj) -> "GridSpec":
        obj = object.__new__(cls)
        obj._handle = core_obj
        return obj

    @classmethod
    def from_mesh(cls, mesh: TriMeshData, resolution: int, padding: float = 0.1) -> "GridSpec":
        if not isinstance(mesh, TriMeshData):
            raise TypeError(f"mesh must be TriMeshData, got {type(mesh).__name__}")
        bmin, bmax = mesh.bbox
        extent = bmax - bmin
        fallback = max(float(np.max(extent)) * float(padding), float(padding), 1e-6)
        pad = np.where(extent > 0.0, extent * float(padding), fallback)
        return cls(bmin - pad, bmax + pad, resolution)

    @property
    def resolution(self) -> int:
        return int(self._handle.resolution)

    @property
    def bmin(self) -> np.ndarray:
        return np.asarray(self._handle.bmin(), dtype=np.float64)

    @property
    def bmax(self) -> np.ndarray:
        return np.asarray(self._handle.bmax(), dtype=np.float64)

    def __repr__(self) -> str:
        return f"GridSpec(bmin={self.bmin.tolist()}, bmax={self.bmax.tolist()}, resolution={self.resolution})"
