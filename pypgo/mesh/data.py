"""Mesh data containers — vertex/element storage with no geometric interpretation.

This module owns the pure-data mesh types and their helper functions.
It has zero internal dependencies on the rest of pypgo.mesh, so other
modules (geo, veg, processing) can safely import from here without
triggering circular imports.
"""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo._utils import float_matrix, index_matrix

MeshDataType = _core.MeshDataType


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _flat_vertices(vertices):
    return float_matrix("vertices", vertices, 3).ravel().tolist()


def _flat_indices(name, value, columns, *, num_vertices):
    return index_matrix(name, value, columns, num_vertices=num_vertices).ravel().tolist()


def _array_from_core(core_obj, method_name, columns, dtype):
    values = getattr(core_obj, method_name)()
    return np.array(values, dtype=dtype).reshape(-1, columns)


def _wrap_mesh_data_core(core_obj):
    if isinstance(core_obj, _core.PyTriMeshData):
        return TriMeshData(core_obj)
    if isinstance(core_obj, _core.PyTetMeshData):
        return TetMeshData(core_obj)
    if isinstance(core_obj, _core.PyCubicMeshData):
        return CubicMeshData(core_obj)
    raise RuntimeError(f"Unexpected core mesh data type: {type(core_obj).__name__}")


def _tet_volumes(points: np.ndarray) -> np.ndarray:
    edges = points[:, 1:] - points[:, 0:1]
    return np.abs(np.linalg.det(edges)) / 6.0


# ---------------------------------------------------------------------------
# Mesh data containers
# ---------------------------------------------------------------------------


class _MeshDataBase:
    _core_type = None
    _create = None
    _element_width = None

    def __init__(self, vertices, elements=None):
        if isinstance(vertices, self._core_type) and elements is None:
            self._handle = vertices
            return

        v_arr = float_matrix("vertices", vertices, 3)
        e_arr = index_matrix("elements", elements, self._element_width, num_vertices=v_arr.shape[0])
        self._handle = self._create(v_arr.ravel().tolist(), e_arr.ravel().tolist())

    @property
    def vertices(self) -> np.ndarray:
        return _array_from_core(self._handle, "vertices", 3, np.float64)

    @property
    def elements(self) -> np.ndarray:
        return _array_from_core(self._handle, "elements", self._element_width, np.int64)

    @property
    def mesh_type(self):
        return self._handle.mesh_type()

    @property
    def num_vertices(self) -> int:
        return self._handle.num_vertices()

    @property
    def num_elements(self) -> int:
        return self._handle.num_elements()

    @property
    def bbox(self) -> tuple[np.ndarray, np.ndarray]:
        vertices = self.vertices
        if vertices.shape[0] == 0:
            raise ValueError("Cannot compute bbox for an empty vertex array")
        return vertices.min(axis=0), vertices.max(axis=0)

    def element_vtx_id(self, element_id: int, local_vertex_id: int) -> int:
        return self._handle.element_vtx_id(int(element_id), int(local_vertex_id))

    def take_elements(self, indices):
        idx = np.asarray(indices, dtype=np.int64)
        return self.__class__(self.vertices, self.elements[idx])

    @classmethod
    def concatenate(cls, meshes):
        if not meshes:
            raise ValueError("meshes must contain at least one mesh")
        if any(not isinstance(mesh, cls) for mesh in meshes):
            raise TypeError(f"All meshes must be {cls.__name__} instances")

        vertices_by_mesh = [mesh.vertices for mesh in meshes]
        vertices = np.concatenate(vertices_by_mesh, axis=0)
        offsets = np.cumsum([0] + [v.shape[0] for v in vertices_by_mesh[:-1]], dtype=np.int64)
        elements = np.concatenate([mesh.elements + offset for mesh, offset in zip(meshes, offsets)], axis=0)
        return cls(vertices, elements)


class TriMeshData(_MeshDataBase):
    """Triangle surface mesh data (MeshData<3>)."""

    _core_type = _core.PyTriMeshData
    _create = staticmethod(_core.create_tri_mesh_data)
    _element_width = 3


class TetMeshData(_MeshDataBase):
    """Tetrahedral volume mesh data (MeshData<4>)."""

    _core_type = _core.PyTetMeshData
    _create = staticmethod(_core.create_tet_mesh_data)
    _element_width = 4

    @property
    def volume(self) -> float:
        points = self.vertices[self.elements]
        return float(_tet_volumes(points).sum())

    @property
    def center_of_mass(self) -> np.ndarray:
        points = self.vertices[self.elements]
        volumes = _tet_volumes(points)
        if volumes.size == 0 or np.all(volumes == 0.0):
            return self.vertices.mean(axis=0)
        centroids = points.mean(axis=1)
        return np.average(centroids, axis=0, weights=volumes)


class CubicMeshData(_MeshDataBase):
    """Cubic/hexahedral volume mesh data (MeshData<8>)."""

    _core_type = _core.PyCubicMeshData
    _create = staticmethod(_core.create_cubic_mesh_data)
    _element_width = 8

    _tet_decomposition = np.array(
        [
            [0, 1, 3, 4],
            [1, 2, 3, 6],
            [1, 3, 4, 6],
            [1, 4, 5, 6],
            [3, 4, 6, 7],
        ],
        dtype=np.int64,
    )

    def _decomposed_tets(self) -> np.ndarray:
        cubes = self.vertices[self.elements]
        return cubes[:, self._tet_decomposition].reshape(-1, 4, 3)

    @property
    def volume(self) -> float:
        return float(_tet_volumes(self._decomposed_tets()).sum())

    @property
    def center_of_mass(self) -> np.ndarray:
        tets = self._decomposed_tets()
        volumes = _tet_volumes(tets)
        if volumes.size == 0 or np.all(volumes == 0.0):
            return self.vertices.mean(axis=0)
        centroids = tets.mean(axis=1)
        return np.average(centroids, axis=0, weights=volumes)
