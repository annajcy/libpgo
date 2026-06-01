"""Geometry-only mesh facades and geometry algorithms."""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo._arrays import float_matrix, index_matrix
from pypgo.mesh import CubicMeshData, TetMeshData, TriMeshData, _array_from_core
from pypgo.sparse import SparseMatrix


class TriMeshGeo:
    """Triangle surface mesh geometry facade."""

    def __init__(self, vertices, triangles=None):
        if isinstance(vertices, _core.PyTriMeshGeo) and triangles is None:
            self._core_obj = vertices
            return

        v_arr = float_matrix("vertices", vertices, 3)
        t_arr = index_matrix("triangles", triangles, 3, num_vertices=v_arr.shape[0])
        self._core_obj = _core.create_tri_mesh_geo(v_arr.ravel().tolist(), t_arr.ravel().tolist())

    @classmethod
    def from_mesh_data(cls, mesh_data: TriMeshData) -> "TriMeshGeo":
        if not isinstance(mesh_data, TriMeshData):
            raise TypeError(f"mesh_data must be a TriMeshData, got {type(mesh_data).__name__}")
        return cls(_core.PyTriMeshGeo(mesh_data._core_obj))

    @property
    def vertices(self) -> np.ndarray:
        return _array_from_core(self._core_obj, "vertices", 3, np.float64)

    @property
    def triangles(self) -> np.ndarray:
        return _array_from_core(self._core_obj, "triangles", 3, np.int64)

    @property
    def num_vertices(self) -> int:
        return self._core_obj.num_vertices()

    @property
    def num_triangles(self) -> int:
        return self._core_obj.num_triangles()

    @property
    def face_areas(self) -> np.ndarray:
        vertices = self.vertices[self.triangles]
        cross = np.cross(vertices[:, 1] - vertices[:, 0], vertices[:, 2] - vertices[:, 0])
        return np.linalg.norm(cross, axis=1) / 2.0

    @property
    def face_normals(self) -> np.ndarray:
        vertices = self.vertices[self.triangles]
        normals = np.cross(vertices[:, 1] - vertices[:, 0], vertices[:, 2] - vertices[:, 0])
        lengths = np.linalg.norm(normals, axis=1, keepdims=True)
        return normals / np.where(lengths == 0.0, 1.0, lengths)

    @property
    def vertex_normals(self) -> np.ndarray:
        weighted_normals = self.face_normals * self.face_areas[:, None]
        normals = np.zeros((self.num_vertices, 3), dtype=np.float64)
        triangles = self.triangles
        np.add.at(normals, triangles[:, 0], weighted_normals)
        np.add.at(normals, triangles[:, 1], weighted_normals)
        np.add.at(normals, triangles[:, 2], weighted_normals)
        lengths = np.linalg.norm(normals, axis=1, keepdims=True)
        return normals / np.where(lengths == 0.0, 1.0, lengths)

    def tri_vtx_id(self, tri_id: int, local_vertex_id: int) -> int:
        return self._core_obj.tri_vtx_id(int(tri_id), int(local_vertex_id))

    def to_mesh_data(self) -> TriMeshData:
        return TriMeshData(self._core_obj.to_mesh_data())


class BarycentricEmbedding:
    """Barycentric interpolation from a volume mesh to target locations."""

    def __init__(self, target_locations, volume_mesh):
        from pypgo.mesh.veg import VolumeMesh

        if not isinstance(volume_mesh, VolumeMesh):
            raise TypeError(f"volume_mesh must be a VolumeMesh, got {type(volume_mesh).__name__}")
        locations = float_matrix("target_locations", target_locations, 3)
        self._num_target_locations = int(locations.shape[0])
        self._core_obj = _core.PyBarycentricEmbedding(
            locations.ravel().tolist(), volume_mesh._core_obj)

    @property
    def num_target_locations(self) -> int:
        return self._num_target_locations

    @property
    def interpolation_matrix(self) -> SparseMatrix:
        return SparseMatrix(self._core_obj.interpolation_matrix())

    def interpolation_matrix_coo(self):
        return self.interpolation_matrix.to_coo()

    def deform(self, volume_disp) -> np.ndarray:
        disp = np.ascontiguousarray(volume_disp, dtype=np.float64)
        if disp.ndim != 1:
            disp = disp.reshape(-1)
        return np.asarray(self._core_obj.deform(disp.tolist()), dtype=np.float64)


def triangle_component_ids(tri_data: TriMeshData) -> tuple[np.ndarray, np.ndarray]:
    """Edge-connected component labels for each triangle.

    Returns (component_ids, component_sizes):
      component_ids  — int64 array of shape (n_triangles,), triID → componentID
      component_sizes — int64 array, componentID → triangle count (unsorted)
    """
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    ids, sizes = _core.triangle_component_ids(tri_data._core_obj)
    return np.asarray(ids, dtype=np.int64), np.asarray(sizes, dtype=np.int64)


def connected_components_by_edge(tri_data: TriMeshData) -> list[np.ndarray]:
    """Split a surface mesh into edge-connected components.

    Returns a list of int64 arrays, each containing the triangle indices of one
    component.  Components are returned in the order produced by the C++ library
    (typically descending size).
    """
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    groups = _core.connected_components_by_edge(tri_data._core_obj)
    return [np.asarray(g, dtype=np.int64) for g in groups]


def connected_components_by_vertex(tri_data: TriMeshData) -> list[np.ndarray]:
    """Split a surface mesh into vertex-connected components.

    Vertex connectivity is weaker than edge connectivity and can merge
    components that only share a single vertex (pinch-point).
    Returns a list of int64 triangle-index arrays, one per component.
    """
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    groups = _core.connected_components_by_vertex(tri_data._core_obj)
    return [np.asarray(g, dtype=np.int64) for g in groups]


def filter_small_components(
    tri_data: TriMeshData,
    *,
    min_triangles: int,
    keep_largest: int = -1,
) -> TriMeshData:
    """Remove edge-connected components with fewer than min_triangles triangles.

    keep_largest > 0  — after the threshold pass, keep only the N largest.
    keep_largest = -1 — keep all components above the threshold (default).
    Isolated vertices are removed from the returned mesh.
    """
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    return TriMeshData(_core.filter_small_components(tri_data._core_obj, int(min_triangles), int(keep_largest)))


def get_outer_component(tri_data: TriMeshData) -> TriMeshData:
    """Extract the outermost vertex-connected component.

    Finds the topmost triangle by y-coordinate, assumes it belongs to the outer
    shell, and returns the entire vertex-connected component containing it.
    Useful for isolating the outer surface of nested shell/cavity meshes.
    """
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    return TriMeshData(_core.get_outer_component(tri_data._core_obj))


def surface_to_volume_interpolation_matrix(surface_mesh: TriMeshData, volume_mesh) -> SparseMatrix:
    """Barycentric interpolation matrix from a volume mesh to a surface mesh.

    Returns W with shape (3 * n_surf_verts, 3 * n_vol_verts) such that
    W @ vol_disp.ravel() gives the interpolated surface displacements.
    """
    if not isinstance(surface_mesh, TriMeshData):
        raise TypeError(f"surface_mesh must be a TriMeshData, got {type(surface_mesh).__name__}")
    return BarycentricEmbedding(surface_mesh.vertices, volume_mesh).interpolation_matrix


class TetMeshGeo:
    """Tetrahedral mesh geometry facade."""

    def __init__(self, vertices, tets=None):
        if isinstance(vertices, _core.PyTetMeshGeo) and tets is None:
            self._core_obj = vertices
            return

        v_arr = float_matrix("vertices", vertices, 3)
        t_arr = index_matrix("tets", tets, 4, num_vertices=v_arr.shape[0])
        self._core_obj = _core.create_tet_mesh_geo(v_arr.ravel().tolist(), t_arr.ravel().tolist())

    @classmethod
    def from_mesh_data(cls, mesh_data: TetMeshData) -> "TetMeshGeo":
        if not isinstance(mesh_data, TetMeshData):
            raise TypeError(f"mesh_data must be a TetMeshData, got {type(mesh_data).__name__}")
        return cls(_core.PyTetMeshGeo(mesh_data._core_obj))

    @property
    def vertices(self) -> np.ndarray:
        return _array_from_core(self._core_obj, "vertices", 3, np.float64)

    @property
    def tets(self) -> np.ndarray:
        return _array_from_core(self._core_obj, "tets", 4, np.int64)

    @property
    def num_vertices(self) -> int:
        return self._core_obj.num_vertices()

    @property
    def num_tets(self) -> int:
        return self._core_obj.num_tets()

    def tet_vtx_id(self, tet_id: int, local_vertex_id: int) -> int:
        return self._core_obj.tet_vtx_id(int(tet_id), int(local_vertex_id))

    def to_mesh_data(self) -> TetMeshData:
        return TetMeshData(self._core_obj.to_mesh_data())


class CubicMeshGeo:
    """Cubic/hexahedral mesh geometry facade."""

    def __init__(self, vertices, cubes=None):
        if isinstance(vertices, _core.PyCubicMeshGeo) and cubes is None:
            self._core_obj = vertices
            return

        v_arr = float_matrix("vertices", vertices, 3)
        c_arr = index_matrix("cubes", cubes, 8, num_vertices=v_arr.shape[0])
        self._core_obj = _core.create_cubic_mesh_geo(v_arr.ravel().tolist(), c_arr.ravel().tolist())

    @classmethod
    def from_mesh_data(cls, mesh_data: CubicMeshData) -> "CubicMeshGeo":
        if not isinstance(mesh_data, CubicMeshData):
            raise TypeError(f"mesh_data must be a CubicMeshData, got {type(mesh_data).__name__}")
        return cls(_core.PyCubicMeshGeo(mesh_data._core_obj))

    @property
    def vertices(self) -> np.ndarray:
        return _array_from_core(self._core_obj, "vertices", 3, np.float64)

    @property
    def cubes(self) -> np.ndarray:
        return _array_from_core(self._core_obj, "cubes", 8, np.int64)

    @property
    def num_vertices(self) -> int:
        return self._core_obj.num_vertices()

    @property
    def num_cubes(self) -> int:
        return self._core_obj.num_cubes()

    def cube_vtx_id(self, cube_id: int, local_vertex_id: int) -> int:
        return self._core_obj.cube_vtx_id(int(cube_id), int(local_vertex_id))

    def to_mesh_data(self) -> CubicMeshData:
        return CubicMeshData(self._core_obj.to_mesh_data())
