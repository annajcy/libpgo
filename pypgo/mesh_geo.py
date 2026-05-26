"""Geometry-only mesh types for pypgo."""

import numpy as np

import pypgo._core as _core
from pypgo._arrays import float_matrix, index_matrix

MeshDataType = _core.MeshDataType


def _flat_vertices(vertices):
    return float_matrix("vertices", vertices, 3).ravel().tolist()


def _flat_indices(name, value, columns, *, num_vertices):
    return index_matrix(name, value, columns, num_vertices=num_vertices).ravel().tolist()


def _array_from_core(core_obj, method_name, columns, dtype):
    values = getattr(core_obj, method_name)()
    return np.array(values, dtype=dtype).reshape(-1, columns)


class _MeshDataBase:
    _core_type = None
    _create = None
    _element_width = None

    def __init__(self, vertices, elements=None):
        if isinstance(vertices, self._core_type) and elements is None:
            self._core_obj = vertices
            return

        v_arr = float_matrix("vertices", vertices, 3)
        e_arr = index_matrix("elements", elements, self._element_width, num_vertices=v_arr.shape[0])
        self._core_obj = self._create(v_arr.ravel().tolist(), e_arr.ravel().tolist())

    @property
    def vertices(self) -> np.ndarray:
        return _array_from_core(self._core_obj, "vertices", 3, np.float64)

    @property
    def elements(self) -> np.ndarray:
        return _array_from_core(self._core_obj, "elements", self._element_width, np.int64)

    @property
    def mesh_type(self):
        return self._core_obj.mesh_type()

    @property
    def num_vertices(self) -> int:
        return self._core_obj.num_vertices()

    @property
    def num_elements(self) -> int:
        return self._core_obj.num_elements()

    def element_vtx_id(self, element_id: int, local_vertex_id: int) -> int:
        return self._core_obj.element_vtx_id(int(element_id), int(local_vertex_id))


class TriMeshData(_MeshDataBase):
    """Triangle surface mesh data (MeshData<3>)."""

    _core_type = _core.TriMeshDataCore
    _create = staticmethod(_core.create_tri_mesh_data)
    _element_width = 3


class TetMeshData(_MeshDataBase):
    """Tetrahedral volume mesh data (MeshData<4>)."""

    _core_type = _core.TetMeshDataCore
    _create = staticmethod(_core.create_tet_mesh_data)
    _element_width = 4


class CubicMeshData(_MeshDataBase):
    """Cubic/hexahedral volume mesh data (MeshData<8>)."""

    _core_type = _core.CubicMeshDataCore
    _create = staticmethod(_core.create_cubic_mesh_data)
    _element_width = 8


class TriMeshGeo:
    """Triangle surface mesh geometry facade."""

    def __init__(self, vertices, triangles=None):
        if isinstance(vertices, _core.TriMeshGeoCore) and triangles is None:
            self._core_obj = vertices
            return

        v_arr = float_matrix("vertices", vertices, 3)
        t_arr = index_matrix("triangles", triangles, 3, num_vertices=v_arr.shape[0])
        self._core_obj = _core.create_tri_mesh_geo(v_arr.ravel().tolist(), t_arr.ravel().tolist())

    @classmethod
    def from_mesh_data(cls, mesh_data: TriMeshData) -> "TriMeshGeo":
        if not isinstance(mesh_data, TriMeshData):
            raise TypeError(f"mesh_data must be a TriMeshData, got {type(mesh_data).__name__}")
        return cls(_core.TriMeshGeoCore(mesh_data._core_obj))

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

    def tri_vtx_id(self, tri_id: int, local_vertex_id: int) -> int:
        return self._core_obj.tri_vtx_id(int(tri_id), int(local_vertex_id))

    def to_mesh_data(self) -> TriMeshData:
        return TriMeshData(self._core_obj.to_mesh_data())


class TetMeshGeo:
    """Tetrahedral mesh geometry facade."""

    def __init__(self, vertices, tets=None):
        if isinstance(vertices, _core.TetMeshGeoCore) and tets is None:
            self._core_obj = vertices
            return

        v_arr = float_matrix("vertices", vertices, 3)
        t_arr = index_matrix("tets", tets, 4, num_vertices=v_arr.shape[0])
        self._core_obj = _core.create_tet_mesh_geo(v_arr.ravel().tolist(), t_arr.ravel().tolist())

    @classmethod
    def from_mesh_data(cls, mesh_data: TetMeshData) -> "TetMeshGeo":
        if not isinstance(mesh_data, TetMeshData):
            raise TypeError(f"mesh_data must be a TetMeshData, got {type(mesh_data).__name__}")
        return cls(_core.TetMeshGeoCore(mesh_data._core_obj))

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
        if isinstance(vertices, _core.CubicMeshGeoCore) and cubes is None:
            self._core_obj = vertices
            return

        v_arr = float_matrix("vertices", vertices, 3)
        c_arr = index_matrix("cubes", cubes, 8, num_vertices=v_arr.shape[0])
        self._core_obj = _core.create_cubic_mesh_geo(v_arr.ravel().tolist(), c_arr.ravel().tolist())

    @classmethod
    def from_mesh_data(cls, mesh_data: CubicMeshData) -> "CubicMeshGeo":
        if not isinstance(mesh_data, CubicMeshData):
            raise TypeError(f"mesh_data must be a CubicMeshData, got {type(mesh_data).__name__}")
        return cls(_core.CubicMeshGeoCore(mesh_data._core_obj))

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
