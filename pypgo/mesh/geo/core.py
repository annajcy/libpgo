"""Geometry facades — thin wrappers around C++ geometry objects.

These classes delegate storage and heavy computation to the C++ layer.
They are stable glue code that changes infrequently.
"""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo._arrays import float_matrix, index_matrix
from pypgo.mesh.data import CubicMeshData, TetMeshData, TriMeshData, _array_from_core
from pypgo.sparse import SparseMatrix


# ---------------------------------------------------------------------------
# Triangle surface geometry
# ---------------------------------------------------------------------------


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

# ---------------------------------------------------------------------------
# Tet / Cubic geometry facades
# ---------------------------------------------------------------------------


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


# ---------------------------------------------------------------------------
# Barycentric embedding
# ---------------------------------------------------------------------------


class BarycentricEmbedding:
    """Barycentric interpolation from a volume mesh to target locations."""

    def __init__(self, target_locations, volume_mesh):
        from pypgo.mesh.volume import VolumeMesh

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
    def num_element_vertices(self) -> int:
        return int(self._core_obj.num_element_vertices())

    @property
    def embedding_indices(self) -> np.ndarray:
        return np.asarray(self._core_obj.embedding_indices_flat(), dtype=np.int64).reshape(
            self.num_target_locations, self.num_element_vertices
        )

    @property
    def embedding_weights(self) -> np.ndarray:
        return np.asarray(self._core_obj.embedding_weights_flat(), dtype=np.float64).reshape(
            self.num_target_locations, self.num_element_vertices
        )

    @property
    def embedding_elements(self) -> np.ndarray:
        return np.asarray(self._core_obj.embedding_elements(), dtype=np.int64)

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


def surface_to_volume_interpolation_matrix(surface_mesh: TriMeshData, volume_mesh) -> SparseMatrix:
    """Barycentric interpolation matrix from a volume mesh to a surface mesh.

    Returns W with shape (3 * n_surf_verts, 3 * n_vol_verts) such that
    W @ vol_disp.ravel() gives the interpolated surface displacements.
    """
    if not isinstance(surface_mesh, TriMeshData):
        raise TypeError(f"surface_mesh must be a TriMeshData, got {type(surface_mesh).__name__}")
    return BarycentricEmbedding(surface_mesh.vertices, volume_mesh).interpolation_matrix

# ---------------------------------------------------------------------------
# Surface embedding — maps volume displacements onto a surface mesh
# ---------------------------------------------------------------------------

class SurfaceEmbedding:
    """Map volume displacements onto an embedded triangle surface mesh."""

    def __init__(self, surface_mesh: TriMeshData, volume_mesh):
        from pypgo.mesh.volume import VolumeMesh

        if not isinstance(surface_mesh, TriMeshData):
            raise TypeError(f"surface_mesh must be a TriMeshData, got {type(surface_mesh).__name__}")
        if not isinstance(volume_mesh, VolumeMesh):
            raise TypeError(f"volume_mesh must be a VolumeMesh, got {type(volume_mesh).__name__}")

        self._rest_surface = surface_mesh
        self._interpolation_matrix = surface_to_volume_interpolation_matrix(surface_mesh, volume_mesh)

    @property
    def rest_surface(self) -> TriMeshData:
        return self._rest_surface

    @property
    def interpolation_matrix(self):
        return self._interpolation_matrix

    def _flat_volume_displacement(self, volume_displacement) -> np.ndarray:
        disp = np.asarray(volume_displacement, dtype=np.float64)
        if disp.ndim == 2:
            if disp.shape[1] != 3:
                raise ValueError(f"volume_displacement must have shape (n, 3), got {disp.shape}")
            disp = disp.reshape(-1)
        elif disp.ndim != 1:
            raise ValueError(f"volume_displacement must be a flat vector or an (n, 3) array, got {disp.shape}")

        expected = self._interpolation_matrix.shape[1]
        if disp.size != expected:
            raise ValueError(f"volume_displacement has {disp.size} entries, expected {expected}")
        return np.ascontiguousarray(disp, dtype=np.float64)

    def displacement(self, volume_displacement) -> np.ndarray:
        """Interpolate a volume displacement field onto the embedded surface."""

        disp = self._flat_volume_displacement(volume_displacement)
        return (self._interpolation_matrix @ disp).reshape((-1, 3))

    def deform(self, volume_displacement) -> TriMeshData:
        """Return a deformed surface mesh with the rest topology preserved."""

        return TriMeshData(
            self._rest_surface.vertices + self.displacement(volume_displacement),
            self._rest_surface.elements,
        )
