"""Surface extraction: marching cubes and the optional OpenVDB level-set backend."""

from __future__ import annotations

import pypgo._core as _core
from pypgo.mesh import TriMeshData
from pypgo.implicit.base import GridField


def extract_marching_cubes(field: GridField, *, iso_offset: float = 0.0) -> TriMeshData:
    if not isinstance(field, GridField):
        raise TypeError("field must be a GridField; call .sample_to_grid() first")
    return TriMeshData(_core.extract_marching_cubes(field._handle, float(iso_offset)))


# -- OpenVDB level-set backend (optional dependency) --------------------------

def has_openvdb() -> bool:
    return bool(_core.has_openvdb())


class OpenVDBOptions:
    def __init__(self, voxel_size: float, half_width: float = 3.0, adaptivity: float = 0.0, smooth_steps: int = 0):
        self._handle = _core.PyOpenVDBOptions(float(voxel_size))
        self._handle.half_width = float(half_width)
        self._handle.adaptivity = float(adaptivity)
        self._handle.smooth_steps = int(smooth_steps)


def build_openvdb_shell_from_mesh(mesh: TriMeshData, shell_thickness: float, options: OpenVDBOptions):
    if not has_openvdb():
        raise RuntimeError("OpenVDB is not available in this build")
    if not isinstance(mesh, TriMeshData):
        raise TypeError(f"mesh must be TriMeshData, got {type(mesh).__name__}")
    return _core.build_openvdb_shell_from_mesh(mesh._handle, float(shell_thickness), options._handle)


def build_openvdb_from_grid_field(field: GridField, options: OpenVDBOptions):
    if not has_openvdb():
        raise RuntimeError("OpenVDB is not available in this build")
    if not isinstance(field, GridField):
        raise TypeError("field must be a GridField; call .sample_to_grid() first")
    return _core.build_openvdb_from_grid_field(field._handle, options._handle)


def extract_openvdb(levelset, options: OpenVDBOptions) -> TriMeshData:
    if not has_openvdb():
        raise RuntimeError("OpenVDB is not available in this build")
    return TriMeshData(_core.extract_openvdb(levelset, options._handle))
