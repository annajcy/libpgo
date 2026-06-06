"""Mesh generation, surface cleanup, and quality diagnostics."""

from pypgo.mesh.processing.volume import (
    VolumetricMeshInfo,
    cubic_mesher,
    has_cgal_remesher,
    has_tetwild,
    tet_mesher,
    volume_mesh_info,
)
from pypgo.mesh.processing.surface import (
    MergeCloseVerticesResult,
    QualityReport,
    RawSurfaceCleanupReport,
    RawSurfaceCleanupResult,
    RawSurfaceCleanupStats,
    cgal_isotropic_remesh,
    cgal_repair_self_intersections,
    cgal_simplify,
    cgal_smooth,
    check_surface_quality,
    merge_close_vertices,
    raw_surface_cleanup,
    remove_isolated_vertices,
)

__all__ = [
    "MergeCloseVerticesResult",
    "QualityReport",
    "RawSurfaceCleanupReport",
    "RawSurfaceCleanupResult",
    "RawSurfaceCleanupStats",
    "VolumetricMeshInfo",
    "cgal_isotropic_remesh",
    "cgal_repair_self_intersections",
    "cgal_simplify",
    "cgal_smooth",
    "check_surface_quality",
    "cubic_mesher",
    "has_cgal_remesher",
    "has_tetwild",
    "merge_close_vertices",
    "raw_surface_cleanup",
    "remove_isolated_vertices",
    "tet_mesher",
    "volume_mesh_info",
]
