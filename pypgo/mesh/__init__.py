"""Mesh data containers, OBJ I/O, and mesh factories."""

from __future__ import annotations

from importlib import import_module

import numpy as np

import pypgo._core as _core
from pypgo._arrays import float_matrix, index_matrix

MeshDataType = _core.MeshDataType

__all__ = [
    "MeshDataType",
    "TriMeshData",
    "TetMeshData",
    "CubicMeshData",
    "read_obj",
    "write_obj",
    "create_box",
    "create_sphere",
    "create_cylinder",
    "create_torus",
    "geo",
    "veg",
]


def _flat_vertices(vertices):
    return float_matrix("vertices", vertices, 3).ravel().tolist()


def _flat_indices(name, value, columns, *, num_vertices):
    return index_matrix(name, value, columns, num_vertices=num_vertices).ravel().tolist()


def _vec3(name: str, value) -> list[float]:
    arr = np.ascontiguousarray(value, dtype=np.float64)
    if arr.shape != (3,):
        raise ValueError(f"{name} must be a 3-vector, got shape {arr.shape}")
    return arr.tolist()


def _array_from_core(core_obj, method_name, columns, dtype):
    values = getattr(core_obj, method_name)()
    return np.array(values, dtype=dtype).reshape(-1, columns)


def _wrap_mesh_data_core(core_obj):
    if isinstance(core_obj, _core.TriMeshDataCore):
        return TriMeshData(core_obj)
    if isinstance(core_obj, _core.TetMeshDataCore):
        return TetMeshData(core_obj)
    if isinstance(core_obj, _core.CubicMeshDataCore):
        return CubicMeshData(core_obj)
    raise RuntimeError(f"Unexpected core mesh data type: {type(core_obj).__name__}")


def _tet_volumes(points: np.ndarray) -> np.ndarray:
    edges = points[:, 1:] - points[:, 0:1]
    return np.abs(np.linalg.det(edges)) / 6.0


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

    @property
    def bbox(self) -> tuple[np.ndarray, np.ndarray]:
        vertices = self.vertices
        if vertices.shape[0] == 0:
            raise ValueError("Cannot compute bbox for an empty vertex array")
        return vertices.min(axis=0), vertices.max(axis=0)

    def element_vtx_id(self, element_id: int, local_vertex_id: int) -> int:
        return self._core_obj.element_vtx_id(int(element_id), int(local_vertex_id))

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

    _core_type = _core.TriMeshDataCore
    _create = staticmethod(_core.create_tri_mesh_data)
    _element_width = 3


class TetMeshData(_MeshDataBase):
    """Tetrahedral volume mesh data (MeshData<4>)."""

    _core_type = _core.TetMeshDataCore
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

    _core_type = _core.CubicMeshDataCore
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


def read_obj(path: str) -> TriMeshData:
    """Read an OBJ surface mesh."""
    return TriMeshData(_core.read_obj(str(path)))


def write_obj(path: str, surface_data: TriMeshData) -> None:
    """Write a triangle surface mesh to OBJ."""
    if not isinstance(surface_data, TriMeshData):
        raise TypeError(f"surface_data must be a TriMeshData, got {type(surface_data).__name__}")
    _core.write_obj(str(path), surface_data._core_obj)


def create_box(*, bmin, bmax) -> TriMeshData:
    """Create an axis-aligned box surface mesh."""
    return TriMeshData(_core.create_box_mesh(_vec3("bmin", bmin), _vec3("bmax", bmax)))


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
    if name in {"geo", "veg"}:
        module = import_module(f"{__name__}.{name}")
        globals()[name] = module
        return module
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
