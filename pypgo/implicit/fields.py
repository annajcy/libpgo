"""Implicit field primitives and the mesh-thickening pipeline.

This is the side of the implicit package expected to grow: each shape primitive
is a concrete ``ImplicitField`` subclass and lives here.  When this file gets
heavy, promote it to a ``fields/`` subpackage (one module per primitive).
"""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo._utils import vec3
from pypgo.mesh import TriMeshData
from pypgo.implicit.base import ImplicitField
from pypgo.implicit.grid import GridSpec
from pypgo.implicit.extract import extract_marching_cubes


class SphereField(ImplicitField):
    def __init__(self, center, radius: float):
        super().__init__(_core.PySphereField(vec3("center", center), float(radius)))

    @classmethod
    def from_mesh_bbox(cls, mesh: TriMeshData) -> "SphereField":
        if not isinstance(mesh, TriMeshData):
            raise TypeError(f"mesh must be TriMeshData, got {type(mesh).__name__}")
        return cls._from_core(_core.PySphereField.from_mesh_bbox(mesh._handle))

    @property
    def center(self) -> np.ndarray:
        return np.asarray(self._handle.center(), dtype=np.float64)

    @property
    def radius(self) -> float:
        return float(self._handle.radius())


class MeshUnsignedDistanceField(ImplicitField):
    def __init__(self, mesh: TriMeshData):
        if not isinstance(mesh, TriMeshData):
            raise TypeError(f"mesh must be TriMeshData, got {type(mesh).__name__}")
        super().__init__(_core.PyMeshUnsignedDistanceField(mesh._handle))


class BoxField(ImplicitField):
    def __init__(self, center, half_extent):
        super().__init__(_core.PyBoxField(vec3("center", center), vec3("half_extent", half_extent)))

    @classmethod
    def from_bbox(cls, bmin, bmax) -> "BoxField":
        return cls._from_core(_core.PyBoxField.from_bbox(vec3("bmin", bmin), vec3("bmax", bmax)))


def thicken_mesh_surface(
    mesh: TriMeshData,
    *,
    thickness: float,
    resolution: int,
    padding: float = 0.1,
    iso_offset: float = 0.0,
) -> TriMeshData:
    if not isinstance(mesh, TriMeshData):
        raise TypeError(f"mesh must be TriMeshData, got {type(mesh).__name__}")
    grid_spec = GridSpec.from_mesh(mesh, resolution, padding=padding)
    field = MeshUnsignedDistanceField(mesh).offset(0.5 * float(thickness))
    grid = field.sample_to_grid(grid_spec)
    return extract_marching_cubes(grid, iso_offset=iso_offset)
