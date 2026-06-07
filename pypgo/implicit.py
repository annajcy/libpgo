"""Lazy implicit surface fields and extraction helpers."""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo.mesh import TriMeshData


def _vec3(name: str, value) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.shape != (3,):
        raise ValueError(f"{name} must be a 3-element array")
    return np.ascontiguousarray(arr)


class GridSpec:
    def __init__(self, bmin, bmax, resolution: int):
        self._handle = _core.PyGridSpec(_vec3("bmin", bmin), _vec3("bmax", bmax), int(resolution))

    @classmethod
    def _from_core(cls, core_obj) -> "GridSpec":
        obj = object.__new__(cls)
        obj._handle = core_obj
        return obj

    @classmethod
    def from_mesh(cls, mesh: TriMeshData, resolution: int, padding: float = 0.1) -> "GridSpec":
        if not isinstance(mesh, TriMeshData):
            raise TypeError(f"mesh must be TriMeshData, got {type(mesh).__name__}")
        bmin, bmax = mesh.bbox
        extent = bmax - bmin
        fallback = max(float(np.max(extent)) * float(padding), float(padding), 1e-6)
        pad = np.where(extent > 0.0, extent * float(padding), fallback)
        return cls(bmin - pad, bmax + pad, resolution)

    @property
    def resolution(self) -> int:
        return int(self._handle.resolution)

    @property
    def bmin(self) -> np.ndarray:
        return np.asarray(self._handle.bmin(), dtype=np.float64)

    @property
    def bmax(self) -> np.ndarray:
        return np.asarray(self._handle.bmax(), dtype=np.float64)

    def __repr__(self) -> str:
        return f"GridSpec(bmin={self.bmin.tolist()}, bmax={self.bmax.tolist()}, resolution={self.resolution})"


class ImplicitField:
    def __init__(self, core_obj):
        self._handle = core_obj

    @classmethod
    def _from_core(cls, core_obj):
        obj = object.__new__(cls)
        obj._handle = core_obj
        return obj

    def eval(self, p) -> float:
        return float(self._handle.eval(_vec3("p", p)))

    def bounds(self):
        result = self._handle.bounds()
        if result is None:
            return None
        return tuple(np.asarray(v, dtype=np.float64) for v in result)

    def sample_to_grid(self, grid_spec: GridSpec, *, num_threads: int | None = None) -> "GridField":
        if not isinstance(grid_spec, GridSpec):
            raise TypeError(f"grid_spec must be GridSpec, got {type(grid_spec).__name__}")
        if num_threads is None:
            core_num_threads = 0
        else:
            core_num_threads = int(num_threads)
            if core_num_threads <= 0:
                raise ValueError("num_threads must be a positive integer or None")
        return GridField(self._handle.sample_to_grid(grid_spec._handle, core_num_threads))

    def __or__(self, other: "ImplicitField") -> "ImplicitField":
        return ImplicitField(_core.implicit_union(self._handle, _field_core(other)))

    def __and__(self, other: "ImplicitField") -> "ImplicitField":
        return ImplicitField(_core.implicit_intersection(self._handle, _field_core(other)))

    def __sub__(self, other: "ImplicitField") -> "ImplicitField":
        return ImplicitField(_core.implicit_difference(self._handle, _field_core(other)))

    def offset(self, value: float) -> "ImplicitField":
        return ImplicitField(_core.implicit_offset(self._handle, float(value)))


def _field_core(value: ImplicitField):
    if not isinstance(value, ImplicitField):
        raise TypeError(f"value must be ImplicitField, got {type(value).__name__}")
    return value._handle


class GridField(ImplicitField):
    @property
    def values(self) -> np.ndarray:
        return np.asarray(self._handle)

    @property
    def grid_spec(self) -> GridSpec:
        return GridSpec._from_core(self._handle.grid_spec())


class SphereField(ImplicitField):
    def __init__(self, center, radius: float):
        super().__init__(_core.PySphereField(_vec3("center", center), float(radius)))

    @classmethod
    def from_mesh_bbox(cls, mesh: TriMeshData) -> "SphereField":
        if not isinstance(mesh, TriMeshData):
            raise TypeError(f"mesh must be TriMeshData, got {type(mesh).__name__}")
        return cls._from_core(_core.PySphereField.from_mesh_bbox(mesh._handle))

    @property
    def center(self) -> np.ndarray:
        return np.asarray(self._handle.center(), dtype=np.float64)

    @property
    def radius(self) -> float:
        return float(self._handle.radius())


class MeshUnsignedDistanceField(ImplicitField):
    def __init__(self, mesh: TriMeshData):
        if not isinstance(mesh, TriMeshData):
            raise TypeError(f"mesh must be TriMeshData, got {type(mesh).__name__}")
        super().__init__(_core.PyMeshUnsignedDistanceField(mesh._handle))


class BoxField(ImplicitField):
    def __init__(self, center, half_extent):
        super().__init__(_core.PyBoxField(_vec3("center", center), _vec3("half_extent", half_extent)))

    @classmethod
    def from_bbox(cls, bmin, bmax) -> "BoxField":
        return cls._from_core(_core.PyBoxField.from_bbox(_vec3("bmin", bmin), _vec3("bmax", bmax)))


def extract_marching_cubes(field: GridField, *, iso_offset: float = 0.0) -> TriMeshData:
    if not isinstance(field, GridField):
        raise TypeError("field must be a GridField; call .sample_to_grid() first")
    return TriMeshData(_core.extract_marching_cubes(field._handle, float(iso_offset)))


def has_openvdb() -> bool:
    return bool(_core.has_openvdb())


class OpenVDBOptions:
    def __init__(self, voxel_size: float, half_width: float = 3.0, adaptivity: float = 0.0, smooth_steps: int = 0):
        self._handle = _core.PyOpenVDBOptions(float(voxel_size))
        self._handle.half_width = float(half_width)
        self._handle.adaptivity = float(adaptivity)
        self._handle.smooth_steps = int(smooth_steps)


def build_openvdb_shell_from_mesh(mesh: TriMeshData, shell_thickness: float, options: OpenVDBOptions):
    if not has_openvdb():
        raise RuntimeError("OpenVDB is not available in this build")
    if not isinstance(mesh, TriMeshData):
        raise TypeError(f"mesh must be TriMeshData, got {type(mesh).__name__}")
    return _core.build_openvdb_shell_from_mesh(mesh._handle, float(shell_thickness), options._handle)


def build_openvdb_from_grid_field(field: GridField, options: OpenVDBOptions):
    if not has_openvdb():
        raise RuntimeError("OpenVDB is not available in this build")
    if not isinstance(field, GridField):
        raise TypeError("field must be a GridField; call .sample_to_grid() first")
    return _core.build_openvdb_from_grid_field(field._handle, options._handle)


def extract_openvdb(levelset, options: OpenVDBOptions) -> TriMeshData:
    if not has_openvdb():
        raise RuntimeError("OpenVDB is not available in this build")
    return TriMeshData(_core.extract_openvdb(levelset, options._handle))


def thicken_mesh_surface(
    mesh: TriMeshData,
    *,
    thickness: float,
    resolution: int,
    padding: float = 0.1,
    iso_offset: float = 0.0,
) -> TriMeshData:
    if not isinstance(mesh, TriMeshData):
        raise TypeError(f"mesh must be TriMeshData, got {type(mesh).__name__}")
    grid_spec = GridSpec.from_mesh(mesh, resolution, padding=padding)
    field = MeshUnsignedDistanceField(mesh).offset(0.5 * float(thickness))
    grid = field.sample_to_grid(grid_spec)
    return extract_marching_cubes(grid, iso_offset=iso_offset)
