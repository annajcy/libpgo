"""Mesh data containers, OBJ I/O, and mesh factories."""

from __future__ import annotations

from importlib import import_module

import numpy as np

import pypgo._core as _core
from pypgo._utils import vec3_list
from pypgo.mesh.data import (
    CubicMeshData,
    MeshDataType,
    TetMeshData,
    TriMeshData,
)
from pypgo.mesh.geometry import SurfaceEmbedding

_VISUALIZE_EXPORTS = {
    "get_backend",
    "plot_points_on_mesh",
    "plot_surface",
    "plot_volume_surface",
    "reset_backend",
    "set_backend",
    "to_pyvista_surface",
    "to_pyvista_volume",
    "write_points_obj",
}

__all__ = [
    "MeshDataType",
    "TriMeshData",
    "TetMeshData",
    "CubicMeshData",
    "SurfaceEmbedding",
    "read_obj",
    "write_obj",
    "write_points_obj",
    "create_box",
    "create_sphere",
    "create_cylinder",
    "create_torus",
    "VolumetricMeshInfo",
    "QualityReport",
    "MergeCloseVerticesResult",
    "RawSurfaceCleanupStats",
    "RawSurfaceCleanupReport",
    "RawSurfaceCleanupResult",
    "volume_mesh_info",
    "cubic_mesher",
    "tet_mesher",
    "has_tetwild",
    "has_cgal_remesher",
    "remove_isolated_vertices",
    "merge_close_vertices",
    "raw_surface_cleanup",
    "plot_points_on_mesh",
    "cgal_smooth",
    "cgal_isotropic_remesh",
    "cgal_repair_self_intersections",
    "cgal_simplify",
    "check_surface_quality",
    "geometry",
    "volume",
]


def read_obj(path: str) -> TriMeshData:
    """Read an OBJ surface mesh."""
    return TriMeshData(_core.read_obj(str(path)))


def write_obj(path: str, surface_data: TriMeshData) -> None:
    """Write a triangle surface mesh to OBJ."""
    if not isinstance(surface_data, TriMeshData):
        raise TypeError(f"surface_data must be a TriMeshData, got {type(surface_data).__name__}")
    _core.write_obj(str(path), surface_data._handle)


def create_box(*, bmin, bmax) -> TriMeshData:
    """Create an axis-aligned box surface mesh."""
    return TriMeshData(_core.create_box_mesh(vec3_list("bmin", bmin), vec3_list("bmax", bmax)))


def create_sphere(*, radius: float, axis_subdiv: int, height_subdiv: int) -> TriMeshData:
    """Create a sphere surface mesh."""
    return TriMeshData(_core.create_sphere_mesh(float(radius), int(axis_subdiv), int(height_subdiv)))


def create_cylinder(*, radius: float, height: float, axis_subdiv: int, height_subdiv: int) -> TriMeshData:
    """Create a capped cylinder surface mesh."""
    return TriMeshData(
        _core.create_cylinder_mesh(float(radius), float(height), int(axis_subdiv), int(height_subdiv))
    )


def create_torus(*, radial_res: int, tubular_res: int, radius: float, thickness: float) -> TriMeshData:
    """Create a torus surface mesh."""
    return TriMeshData(
        _core.create_torus_mesh(int(radial_res), int(tubular_res), float(radius), float(thickness))
    )


def __getattr__(name: str):
    if name in {"geometry", "volume"}:
        module = import_module(f"{__name__}.{name}")
        globals()[name] = module
        return module
    if name == "visualize":
        module = import_module(f"{__name__}.visualize")
        globals()[name] = module
        return module
    if name in _VISUALIZE_EXPORTS:
        module = import_module(f"{__name__}.visualize")
        value = getattr(module, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


from pypgo.mesh.processing import (  # noqa: E402
    MergeCloseVerticesResult,
    QualityReport,
    RawSurfaceCleanupReport,
    RawSurfaceCleanupResult,
    RawSurfaceCleanupStats,
    VolumetricMeshInfo,
    cgal_isotropic_remesh,
    cgal_repair_self_intersections,
    cgal_simplify,
    cgal_smooth,
    check_surface_quality,
    cubic_mesher,
    has_cgal_remesher,
    has_tetwild,
    merge_close_vertices,
    raw_surface_cleanup,
    remove_isolated_vertices,
    tet_mesher,
    volume_mesh_info,
)
