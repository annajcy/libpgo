"""Pure-Python geometry algorithms.

These operate on mesh data types (TriMeshData, TetMeshData, CubicMeshData)
and have no dependency on the C++ geometry facades in core.py.
"""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo.mesh.data import CubicMeshData, TetMeshData, TriMeshData


# ---------------------------------------------------------------------------
# Connected components
# ---------------------------------------------------------------------------

def triangle_component_ids(tri_data: TriMeshData) -> tuple[np.ndarray, np.ndarray]:
    """Edge-connected component labels for each triangle.

    Returns (component_ids, component_sizes):
      component_ids  — int64 array of shape (n_triangles,), triID → componentID
      component_sizes — int64 array, componentID → triangle count (unsorted)
    """
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    ids, sizes = _core.triangle_component_ids(tri_data._handle)
    return np.asarray(ids, dtype=np.int64), np.asarray(sizes, dtype=np.int64)


def connected_components_by_edge(tri_data: TriMeshData) -> list[np.ndarray]:
    """Split a surface mesh into edge-connected components.

    Returns a list of int64 arrays, each containing the triangle indices of one
    component.  Components are returned in the order produced by the C++ library
    (typically descending size).
    """
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    groups = _core.connected_components_by_edge(tri_data._handle)
    return [np.asarray(g, dtype=np.int64) for g in groups]


def connected_components_by_vertex(tri_data: TriMeshData) -> list[np.ndarray]:
    """Split a surface mesh into vertex-connected components.

    Vertex connectivity is weaker than edge connectivity and can merge
    components that only share a single vertex (pinch-point).
    Returns a list of int64 triangle-index arrays, one per component.
    """
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    groups = _core.connected_components_by_vertex(tri_data._handle)
    return [np.asarray(g, dtype=np.int64) for g in groups]


def filter_mesh_components(
    mesh: TriMeshData | TetMeshData | CubicMeshData,
    *,
    min_elements: int = 0,
    keep_largest: int = -1,
) -> TriMeshData | TetMeshData | CubicMeshData:
    """Filter face-connected components and compact unused vertices.

    keep_largest > 0  — after the threshold pass, keep only the N largest.
    keep_largest = -1 — keep all components above the threshold (default).
    """
    mesh_types = (TriMeshData, TetMeshData, CubicMeshData)
    if not isinstance(mesh, mesh_types):
        raise TypeError(f"mesh must be a TriMeshData, TetMeshData, or CubicMeshData, got {type(mesh).__name__}")
    return type(mesh)(_core.filter_mesh_components(mesh._handle, int(min_elements), int(keep_largest)))


def get_outer_component(tri_data: TriMeshData) -> TriMeshData:
    """Extract the outermost vertex-connected component.

    Finds the topmost triangle by y-coordinate, assumes it belongs to the outer
    shell, and returns the entire vertex-connected component containing it.
    Useful for isolating the outer surface of nested shell/cavity meshes.
    """
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    return TriMeshData(_core.get_outer_component(tri_data._handle))


def split_components(tri_data: TriMeshData) -> list[TriMeshData]:
    """Split a surface mesh into edge-connected component submeshes.

    Returns one TriMeshData per component (compacted vertex set), ordered
    by descending triangle count.
    """
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    return [tri_data.take_elements(ids) for ids in connected_components_by_edge(tri_data)]


# ---------------------------------------------------------------------------
# Minimum bounding sphere (Welzl's algorithm, seed=0)
# ---------------------------------------------------------------------------

def _mbs_contains(center: np.ndarray, radius: float, p: np.ndarray, tol: float = 1e-12) -> bool:
    return float(np.linalg.norm(p - center)) <= radius + tol * max(1.0, radius)


def _mbs_from_2(a: np.ndarray, b: np.ndarray) -> tuple[np.ndarray, float]:
    c = (a + b) * 0.5
    return c, float(np.linalg.norm(b - a)) * 0.5


def _mbs_from_3(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> tuple[np.ndarray, float] | None:
    ab, ac = b - a, c - a
    n = np.cross(ab, ac)
    denom = 2.0 * float(n @ n)
    if denom < 1e-30:
        return None
    offset = (np.cross(n, ab) * float(ac @ ac) + np.cross(ac, n) * float(ab @ ab)) / denom
    center = a + offset
    return center, float(np.linalg.norm(center - a))


def _mbs_from_4(a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray) -> tuple[np.ndarray, float] | None:
    M = np.stack([b - a, c - a, d - a]) * 2.0
    rhs = np.array([b @ b - a @ a, c @ c - a @ a, d @ d - a @ a])
    try:
        center = np.linalg.solve(M, rhs)
    except np.linalg.LinAlgError:
        return None
    return center, float(np.linalg.norm(center - a))


def _mbs_from_boundary(boundary: list[np.ndarray]) -> tuple[np.ndarray, float]:
    if not boundary:
        return np.zeros(3), 0.0
    if len(boundary) == 1:
        return boundary[0].copy(), 0.0
    from itertools import combinations
    best = None
    for pair in combinations(boundary, 2):
        s = _mbs_from_2(*pair)
        if all(_mbs_contains(*s, p) for p in boundary):
            if best is None or s[1] < best[1]:
                best = s
    if len(boundary) >= 3:
        for triple in combinations(boundary, 3):
            s = _mbs_from_3(*triple)
            if s and all(_mbs_contains(*s, p) for p in boundary):
                if best is None or s[1] < best[1]:
                    best = s
    if len(boundary) == 4:
        s = _mbs_from_4(*boundary)
        if s and all(_mbs_contains(*s, p) for p in boundary):
            if best is None or s[1] < best[1]:
                best = s
    if best is None:
        raise ValueError("could not construct minimal sphere from boundary points")
    return best


def minimum_bounding_sphere(
    mesh_data,
    *,
    padding: float = 0.0,
) -> tuple[np.ndarray, float]:
    """Minimum enclosing sphere for all vertices of a mesh (Welzl's algorithm).

    Accepts TriMeshData, TetMeshData, or CubicMeshData.

    Returns (center, radius):
      center  — float64 ndarray of shape (3,)
      radius  — float, multiplied by (1 + padding)

    The RNG is seeded at 0 for reproducibility.
    """
    import random as _random
    if not hasattr(mesh_data, "vertices"):
        raise TypeError(f"mesh_data must have a .vertices property, got {type(mesh_data).__name__}")
    pts = [mesh_data.vertices[i] for i in range(mesh_data.num_vertices)]
    _random.Random(0).shuffle(pts)

    center = np.zeros(3)
    radius = -1.0

    for i, p in enumerate(pts):
        if radius >= 0.0 and _mbs_contains(center, radius, p):
            continue
        center, radius = p.copy(), 0.0
        for j in range(i):
            if _mbs_contains(center, radius, pts[j]):
                continue
            center, radius = _mbs_from_2(p, pts[j])
            for k in range(j):
                if _mbs_contains(center, radius, pts[k]):
                    continue
                s = _mbs_from_3(p, pts[j], pts[k])
                center, radius = s if s else _mbs_from_2(p, pts[j])
                for l in range(k):
                    if _mbs_contains(center, radius, pts[l]):
                        continue
                    s = _mbs_from_4(p, pts[j], pts[k], pts[l])
                    if s:
                        center, radius = s
                    else:
                        s = _mbs_from_boundary([p, pts[j], pts[k], pts[l]])
                        center, radius = s

    return np.asarray(center, dtype=np.float64), radius * (1.0 + padding)
