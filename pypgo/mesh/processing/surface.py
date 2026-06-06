"""Surface mesh cleanup, repair, and quality diagnostics."""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import dataclass

import numpy as np

import pypgo._core as _core
from pypgo.mesh.data import TriMeshData
from pypgo.mesh.processing.volume import has_cgal_remesher


# ---------------------------------------------------------------------------
# Surface quality check
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class QualityReport:
    is_clean: bool
    degenerate_tris: list[int]
    short_edges: list[tuple[int, int]]
    non_manifold_edges: list[tuple[int, int]]
    flipped_tris: list[int]
    has_self_intersections: bool


def check_surface_quality(
    tri_data: TriMeshData,
    short_edge_threshold: float = 0.0,
    degenerate_area_threshold: float = 1e-12,
) -> QualityReport:
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")

    vertices = tri_data.vertices
    triangles = tri_data.elements
    tri_vertices = vertices[triangles]
    cross = np.cross(tri_vertices[:, 1] - tri_vertices[:, 0], tri_vertices[:, 2] - tri_vertices[:, 0])
    areas = np.linalg.norm(cross, axis=1) / 2.0

    degenerate_tris = np.flatnonzero(areas <= float(degenerate_area_threshold)).astype(int).tolist()
    edge_records = _edge_records(triangles)
    edge_counts = Counter(record[0] for record in edge_records)
    short_edges = _short_edges(vertices, edge_counts.keys(), float(short_edge_threshold))
    non_manifold_edges = sorted(edge for edge, count in edge_counts.items() if count > 2)
    flipped_tris = _flipped_tris(edge_records)
    has_self_intersections = _core.check_self_intersections(tri_data._core_obj)

    is_clean = not (
        degenerate_tris
        or short_edges
        or non_manifold_edges
        or flipped_tris
        or has_self_intersections
    )
    return QualityReport(
        is_clean=is_clean,
        degenerate_tris=degenerate_tris,
        short_edges=short_edges,
        non_manifold_edges=non_manifold_edges,
        flipped_tris=flipped_tris,
        has_self_intersections=has_self_intersections,
    )


def _edge_records(triangles: np.ndarray):
    records = []
    for face_id, (a, b, c) in enumerate(triangles.tolist()):
        for u, v in ((a, b), (b, c), (c, a)):
            key = (u, v) if u < v else (v, u)
            records.append((key, (u, v), face_id))
    return records


def _short_edges(vertices: np.ndarray, edges, threshold: float) -> list[tuple[int, int]]:
    if threshold <= 0.0:
        return []
    short = []
    for edge in edges:
        a, b = edge
        if np.linalg.norm(vertices[a] - vertices[b]) < threshold:
            short.append(edge)
    return sorted(short)


def _flipped_tris(edge_records) -> list[int]:
    by_edge = defaultdict(list)
    for key, directed_edge, face_id in edge_records:
        by_edge[key].append((directed_edge, face_id))

    flipped = set()
    for records in by_edge.values():
        seen_directions = {}
        for directed_edge, face_id in records:
            if directed_edge in seen_directions:
                flipped.add(face_id)
            seen_directions[directed_edge] = face_id
    return sorted(flipped)


# ---------------------------------------------------------------------------
# Surface cleanup / repair
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MergeCloseVerticesResult:
    surface: TriMeshData
    merged_vertices: int
    eps: float


@dataclass(frozen=True)
class RawSurfaceCleanupStats:
    vertices: int
    triangles: int
    invalid_triangles: int
    components: int
    boundary_or_nonmanifold_edges: int
    is_manifold: bool


@dataclass(frozen=True)
class RawSurfaceCleanupReport:
    expected_components: int
    short_edge_threshold: float
    max_passes: int
    max_collapses: int
    before: RawSurfaceCleanupStats
    after: RawSurfaceCleanupStats
    topology_preserved: bool
    cleanup_complete: bool
    attempted_deletions: int
    accepted_deletions: int
    attempted_collapses: int
    accepted_collapses: int
    rejected_by_topology: int
    rejected_by_invalid_count: int

    def to_dict(self) -> dict:
        before = self.before.__dict__.copy()
        after = self.after.__dict__.copy()
        return {
            "type": "raw_surface_cleanup",
            "expected_components": self.expected_components,
            "short_edge_threshold": self.short_edge_threshold,
            "max_passes": self.max_passes,
            "max_collapses": self.max_collapses,
            "before": before,
            "after": after,
            "vertices_before": self.before.vertices,
            "vertices_after": self.after.vertices,
            "triangles_before": self.before.triangles,
            "triangles_after": self.after.triangles,
            "invalid_triangles_before": self.before.invalid_triangles,
            "invalid_triangles_after": self.after.invalid_triangles,
            "components_before": self.before.components,
            "components_after": self.after.components,
            "boundary_or_nonmanifold_edges_before": self.before.boundary_or_nonmanifold_edges,
            "boundary_or_nonmanifold_edges_after": self.after.boundary_or_nonmanifold_edges,
            "is_manifold_before": self.before.is_manifold,
            "is_manifold_after": self.after.is_manifold,
            "topology_preserved": self.topology_preserved,
            "cleanup_complete": self.cleanup_complete,
            "attempted_deletions": self.attempted_deletions,
            "accepted_deletions": self.accepted_deletions,
            "attempted_collapses": self.attempted_collapses,
            "accepted_collapses": self.accepted_collapses,
            "rejected_by_topology": self.rejected_by_topology,
            "rejected_by_invalid_count": self.rejected_by_invalid_count,
        }


@dataclass(frozen=True)
class RawSurfaceCleanupResult:
    surface: TriMeshData
    report: RawSurfaceCleanupReport


def remove_isolated_vertices(tri_data: TriMeshData) -> TriMeshData:
    """Remove vertices not referenced by any triangle."""
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    return TriMeshData(_core.surface_remove_isolated_vertices(tri_data._core_obj))


def merge_close_vertices(tri_data: TriMeshData, *, eps: float | None = None) -> MergeCloseVerticesResult:
    """Merge vertices closer than eps and clean the resulting triangle surface."""
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    if not has_cgal_remesher():
        raise RuntimeError("CGAL surface cleanup is not available in this build")
    eps_value = -1.0 if eps is None else float(eps)
    surface_core, merged_vertices, used_eps = _core.surface_merge_close_vertices(tri_data._core_obj, eps_value)
    return MergeCloseVerticesResult(
        surface=TriMeshData(surface_core),
        merged_vertices=int(merged_vertices),
        eps=float(used_eps),
    )


def raw_surface_cleanup(
    tri_data: TriMeshData,
    *,
    expected_components: int | None = None,
    short_edge_threshold: float = 1e-5,
    max_passes: int = 3,
    max_collapses: int = 10000,
) -> RawSurfaceCleanupResult:
    """Conservatively remove degenerate triangles while preserving surface topology."""
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    if not has_cgal_remesher():
        raise RuntimeError("CGAL surface cleanup is not available in this build")
    expected = -1 if expected_components is None else int(expected_components)
    surface_core, report_payload = _core.raw_surface_cleanup(
        tri_data._core_obj,
        expected,
        float(short_edge_threshold),
        int(max_passes),
        int(max_collapses),
    )
    return RawSurfaceCleanupResult(
        surface=TriMeshData(surface_core),
        report=_raw_cleanup_report_from_dict(report_payload),
    )


def _raw_cleanup_stats_from_dict(payload: dict) -> RawSurfaceCleanupStats:
    return RawSurfaceCleanupStats(
        vertices=int(payload["vertices"]),
        triangles=int(payload["triangles"]),
        invalid_triangles=int(payload["invalid_triangles"]),
        components=int(payload["components"]),
        boundary_or_nonmanifold_edges=int(payload["boundary_or_nonmanifold_edges"]),
        is_manifold=bool(payload["is_manifold"]),
    )


def _raw_cleanup_report_from_dict(payload: dict) -> RawSurfaceCleanupReport:
    return RawSurfaceCleanupReport(
        expected_components=int(payload["expected_components"]),
        short_edge_threshold=float(payload["short_edge_threshold"]),
        max_passes=int(payload["max_passes"]),
        max_collapses=int(payload["max_collapses"]),
        before=_raw_cleanup_stats_from_dict(payload["before"]),
        after=_raw_cleanup_stats_from_dict(payload["after"]),
        topology_preserved=bool(payload["topology_preserved"]),
        cleanup_complete=bool(payload["cleanup_complete"]),
        attempted_deletions=int(payload["attempted_deletions"]),
        accepted_deletions=int(payload["accepted_deletions"]),
        attempted_collapses=int(payload["attempted_collapses"]),
        accepted_collapses=int(payload["accepted_collapses"]),
        rejected_by_topology=int(payload["rejected_by_topology"]),
        rejected_by_invalid_count=int(payload["rejected_by_invalid_count"]),
    )


# ---------------------------------------------------------------------------
# CGAL surface operations
# ---------------------------------------------------------------------------


def cgal_smooth(tri_data: TriMeshData, *, num_iter: int = 10, sharp_angle: float = 180.0) -> TriMeshData:
    """Smooth a surface mesh with CGAL angle-and-area smoothing.

    Edges whose dihedral angle exceeds sharp_angle (degrees) are treated as
    feature edges and left unmodified.
    """
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    if not has_cgal_remesher():
        raise RuntimeError("CGAL remesher is not available in this build")
    return TriMeshData(_core.cgal_smooth_surface(tri_data._core_obj, int(num_iter), float(sharp_angle)))


def cgal_isotropic_remesh(
    tri_data: TriMeshData,
    *,
    target_edge_length: float,
    num_iter: int = 10,
    sharp_angle: float = 180.0,
) -> TriMeshData:
    """Isotropically remesh a surface to a target edge length with CGAL."""
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    if not has_cgal_remesher():
        raise RuntimeError("CGAL remesher is not available in this build")
    return TriMeshData(
        _core.cgal_isotropic_remesh(
            tri_data._core_obj, float(target_edge_length), int(num_iter), float(sharp_angle)
        )
    )


def cgal_repair_self_intersections(
    tri_data: TriMeshData, *, method: str = "autorefine"
) -> tuple[TriMeshData, bool]:
    """Repair self-intersections with CGAL.

    method: "autorefine" (default), "autorefine-only", or "remove".
    Returns (repaired_mesh, all_fixed).
    """
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    if method not in ("autorefine", "autorefine-only", "remove"):
        raise ValueError(f"method must be 'autorefine', 'autorefine-only', or 'remove', got {method!r}")
    if not has_cgal_remesher():
        raise RuntimeError("CGAL remesher is not available in this build")
    mesh_core, all_fixed = _core.cgal_repair_self_intersections(tri_data._core_obj, method)
    return TriMeshData(mesh_core), bool(all_fixed)


def cgal_simplify(tri_data: TriMeshData, *, target_ratio: float) -> TriMeshData:
    """Simplify a surface mesh to target_ratio of its original edge count with CGAL."""
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    if not has_cgal_remesher():
        raise RuntimeError("CGAL remesher is not available in this build")
    return TriMeshData(_core.cgal_simplify_surface(tri_data._core_obj, float(target_ratio)))
