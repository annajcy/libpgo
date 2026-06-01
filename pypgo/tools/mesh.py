"""Mesh quality checks and diagnostics."""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import dataclass

import numpy as np

import pypgo._core as _core
from pypgo.mesh import CubicMeshData, TetMeshData, TriMeshData


@dataclass(frozen=True)
class VolumetricMeshInfo:
    num_vertices: int
    num_elements: int
    num_element_vertices: int
    total_volume: float
    center_of_mass: np.ndarray

    def __str__(self) -> str:
        cx, cy, cz = self.center_of_mass
        return (
            f"#vtx:              {self.num_vertices}\n"
            f"#elements:         {self.num_elements}\n"
            f"#element vertices: {self.num_element_vertices}\n"
            f"total volume:      {self.total_volume:.17g}\n"
            f"center of mass:    {cx:.17g} {cy:.17g} {cz:.17g}"
        )


def volume_mesh_info(mesh) -> VolumetricMeshInfo:
    """Geometric summary of a volume mesh — equivalent to the volumetricMeshInfo CLI tool.

    Accepts TetMeshData, CubicMeshData, or VolumeMesh.
    """
    from pypgo.mesh.veg import VegFile, VolumeMesh

    if isinstance(mesh, (VegFile, VolumeMesh)):
        mesh = mesh.mesh_data
    if not isinstance(mesh, (TetMeshData, CubicMeshData)):
        raise TypeError(
            f"mesh must be a TetMeshData, CubicMeshData, or VolumeMesh, got {type(mesh).__name__}"
        )
    return VolumetricMeshInfo(
        num_vertices=mesh.num_vertices,
        num_elements=mesh.num_elements,
        num_element_vertices=int(mesh.elements.shape[1]),
        total_volume=mesh.volume,
        center_of_mass=mesh.center_of_mass,
    )


@dataclass(frozen=True)
class QualityReport:
    is_clean: bool
    degenerate_tris: list[int]
    short_edges: list[tuple[int, int]]
    non_manifold_edges: list[tuple[int, int]]
    flipped_tris: list[int]
    has_self_intersections: bool


def cubic_mesher(
    tri_data: TriMeshData,
    *,
    resolution: int,
    E: float = 1e6,
    nu: float = 0.45,
    density: float = 1000.0,
) -> CubicMeshData:
    """Voxelize a closed triangle surface into a cubic volume mesh."""
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    return CubicMeshData(
        _core.cubic_mesher(
            tri_data._core_obj,
            int(resolution),
            float(E),
            float(nu),
            float(density),
        )
    )


def tet_mesher(tri_data: TriMeshData, *, backend: str = "tetgen", config: dict | None = None) -> TetMeshData:
    """Tetrahedralize a closed triangle surface with tetgen or tetwild."""
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")

    config = {} if config is None else dict(config)
    backend = str(backend)
    if backend == "tetwild" and not has_tetwild():
        raise RuntimeError("tetwild backend is not available")

    tetgen_command = str(config.get("command", "pq1.414"))
    tetwild_la = float(config.get("la", 0.0))
    return TetMeshData(
        _core.tet_mesher(
            tri_data._core_obj,
            backend,
            tetgen_command,
            float(config.get("lr", 0.05)),
            tetwild_la,
            "la" in config,
            float(config.get("epsr", 0.001)),
            float(config.get("stop_energy", 10.0)),
            int(config.get("max_threads", 0)),
        )
    )


def has_tetwild() -> bool:
    return bool(_core.has_tetwild())


def has_cgal_remesher() -> bool:
    return bool(_core.has_cgal_remesher())


def has_geogram_remesher() -> bool:
    return bool(_core.has_geogram_remesher())


def remove_isolated_vertices(tri_data: TriMeshData) -> TriMeshData:
    """Remove vertices not referenced by any triangle."""
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    return TriMeshData(_core.surface_remove_isolated_vertices(tri_data._core_obj))


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


def geogram_remesh(
    tri_data: TriMeshData,
    *,
    target_num_vertices: int,
    size_factor: float = 1.0,
    anisotropy: float = 1.0,
) -> TriMeshData:
    """Remesh a surface to a target vertex count with Geogram."""
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    if not has_geogram_remesher():
        raise RuntimeError("Geogram remesher is not available in this build")
    return TriMeshData(
        _core.geogram_remesh_surface(
            tri_data._core_obj, int(target_num_vertices), float(size_factor), float(anisotropy)
        )
    )


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


def _has_self_intersections(vertices: np.ndarray, triangles: np.ndarray) -> bool:
    tri_vertices = vertices[triangles]
    tri_bmin = tri_vertices.min(axis=1)
    tri_bmax = tri_vertices.max(axis=1)

    for i in range(len(triangles)):
        for j in range(i + 1, len(triangles)):
            if np.intersect1d(triangles[i], triangles[j]).size > 0:
                continue
            if np.any(tri_bmax[i] < tri_bmin[j]) or np.any(tri_bmax[j] < tri_bmin[i]):
                continue
            if _triangles_intersect(tri_vertices[i], tri_vertices[j]):
                return True
    return False


def _triangles_intersect(a: np.ndarray, b: np.ndarray) -> bool:
    for p0, p1 in ((a[0], a[1]), (a[1], a[2]), (a[2], a[0])):
        if _segment_intersects_triangle(p0, p1, b):
            return True
    for p0, p1 in ((b[0], b[1]), (b[1], b[2]), (b[2], b[0])):
        if _segment_intersects_triangle(p0, p1, a):
            return True
    return False


def _segment_intersects_triangle(p0: np.ndarray, p1: np.ndarray, tri: np.ndarray, eps: float = 1e-12) -> bool:
    edge0 = tri[1] - tri[0]
    edge1 = tri[2] - tri[0]
    direction = p1 - p0
    normal = np.cross(edge0, edge1)
    denom = float(np.dot(normal, direction))
    if abs(denom) <= eps:
        return False

    t = float(np.dot(normal, tri[0] - p0) / denom)
    if t < -eps or t > 1.0 + eps:
        return False

    point = p0 + t * direction
    return _point_in_triangle(point, tri, eps)


def _point_in_triangle(point: np.ndarray, tri: np.ndarray, eps: float) -> bool:
    v0 = tri[2] - tri[0]
    v1 = tri[1] - tri[0]
    v2 = point - tri[0]

    dot00 = float(np.dot(v0, v0))
    dot01 = float(np.dot(v0, v1))
    dot02 = float(np.dot(v0, v2))
    dot11 = float(np.dot(v1, v1))
    dot12 = float(np.dot(v1, v2))
    denom = dot00 * dot11 - dot01 * dot01
    if abs(denom) <= eps:
        return False

    inv_denom = 1.0 / denom
    u = (dot11 * dot02 - dot01 * dot12) * inv_denom
    v = (dot00 * dot12 - dot01 * dot02) * inv_denom
    return u >= -eps and v >= -eps and (u + v) <= 1.0 + eps
