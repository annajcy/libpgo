"""Volume mesh wrapper, VegFile data model, and .veg I/O."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import pypgo._core as _core
from pypgo.mesh.data import CubicMeshData, TetMeshData, TriMeshData, _wrap_mesh_data_core
from pypgo.mesh.volume.material import (
    ENuMaterial,
    MaterialLike,
    MooneyRivlinMaterial,
    _material_to_core_payload,
    _wrap_material_payload,
)


# ---------------------------------------------------------------------------
# VegFile data model
# ---------------------------------------------------------------------------


@dataclass
class MeshSet:
    name: str
    elements: list[int]

    def __post_init__(self):
        self.elements = sorted(set(int(e) for e in self.elements))


@dataclass
class MeshRegion:
    material_index: int
    set_index: int


@dataclass
class VegFile:
    mesh_data: TetMeshData | CubicMeshData
    materials: list[MaterialLike]
    sets: list[MeshSet]
    regions: list[MeshRegion]

    @classmethod
    def from_single_material(cls, mesh_data, material: MaterialLike) -> "VegFile":
        if not isinstance(mesh_data, (TetMeshData, CubicMeshData)):
            raise TypeError(f"mesh_data must be a TetMeshData or CubicMeshData, got {type(mesh_data).__name__}")
        return cls(
            mesh_data=mesh_data,
            materials=[material],
            sets=[MeshSet("allElements", list(range(mesh_data.num_elements)))],
            regions=[MeshRegion(0, 0)],
        )

    def first_material(self) -> MaterialLike:
        if len(self.materials) != 1:
            raise ValueError(f"expected exactly one material, got {len(self.materials)}")
        return self.materials[0]

    def to_volume_regions(self) -> list[tuple[str, MaterialLike, list[int]]]:
        return [
            (
                self.sets[region.set_index].name,
                self.materials[region.material_index],
                list(self.sets[region.set_index].elements),
            )
            for region in self.regions
        ]


# ---------------------------------------------------------------------------
# Region validation helper
# ---------------------------------------------------------------------------


def _validate_and_split_regions(regions, num_elements: int):
    names: set[str] = set()
    assigned: dict[int, str] = {}
    materials = []
    sets = []
    region_payloads = []

    for region_id, item in enumerate(regions):
        try:
            name, material, elements = item
        except (TypeError, ValueError) as exc:
            raise TypeError("each region must be (name, material, elements)") from exc

        if name in names:
            raise ValueError(f"duplicate region name '{name}'")
        names.add(name)

        element_list = [int(e) for e in elements]
        for element in element_list:
            if element < 0 or element >= num_elements:
                raise ValueError(f"region '{name}' references element {element} out of [0, {num_elements})")
            if element in assigned:
                raise ValueError(f"element {element} assigned to both '{assigned[element]}' and '{name}'")
            assigned[element] = name

        materials.append(_material_to_core_payload(material))
        sets.append((str(name), sorted(set(element_list))))
        region_payloads.append((region_id, region_id))

    for element in range(num_elements):
        if element not in assigned:
            raise ValueError(f"element {element} not assigned to any region")

    return materials, sets, region_payloads


# ---------------------------------------------------------------------------
# VolumeMesh
# ---------------------------------------------------------------------------


class VolumeMesh:
    """Vega FEM volumetric mesh wrapper.

    Use pypgo.fem.SimulationAsset factory methods for solver-ready assets.
    """

    def __init__(self, mesh_data, regions):
        if not isinstance(mesh_data, (TetMeshData, CubicMeshData)):
            raise TypeError(f"mesh_data must be a TetMeshData or CubicMeshData, got {type(mesh_data).__name__}")

        self._mesh_data = mesh_data
        materials, sets, region_payloads = _validate_and_split_regions(regions, mesh_data.num_elements)
        self._handle = _core.create_volume_mesh_multi(
            mesh_data._handle, materials, sets, region_payloads)

    @classmethod
    def create_from_single_material(cls, mesh_data, material) -> "VolumeMesh":
        if not isinstance(mesh_data, (TetMeshData, CubicMeshData)):
            raise TypeError(f"mesh_data must be a TetMeshData or CubicMeshData, got {type(mesh_data).__name__}")
        if not isinstance(material, (ENuMaterial, MooneyRivlinMaterial)):
            raise TypeError(f"material must be a veg material type, got {type(material).__name__}")
        obj = cls.__new__(cls)
        obj._mesh_data = mesh_data
        obj._handle = _core.create_volume_mesh_multi(
            mesh_data._handle,
            [_material_to_core_payload(material)],
            [("allElements", list(range(mesh_data.num_elements)))],
            [(0, 0)],
        )
        return obj

    @classmethod
    def from_veg_file(cls, veg: VegFile) -> "VolumeMesh":
        return cls(veg.mesh_data, veg.to_volume_regions())

    def extract_surface_mesh(self, *, triangulate: bool = True) -> TriMeshData:
        return TriMeshData(_core.extract_surface_mesh(self._handle, bool(triangulate)))

    def _mass_matrix(self, *, inflate3dim: bool = True):
        """(Internal) Consistent mass matrix — use formulation-level API instead.

    Prefer ``pypgo.fem.VolumeDensity`` + ``formulation.mass_matrix(asset, density)``.
        """
        from pypgo.sparse import SparseMatrix
        return SparseMatrix(_core.compute_mass_matrix(self._handle, bool(inflate3dim)))

    @property
    def num_vertices(self) -> int:
        return self._handle.num_vertices()

    @property
    def num_elements(self) -> int:
        return self._handle.num_elements()

    @property
    def mesh_type(self):
        return self._handle.mesh_type()

    @property
    def mesh_data(self):
        if self._mesh_data is None:
            self._mesh_data = _wrap_mesh_data_core(self._handle.export_geometry())
        return self._mesh_data

    @property
    def geometry(self):
        return self.mesh_data

    @property
    def material(self):
        return _wrap_material_payload(self._handle.export_material_payload())

    @property
    def material_spec(self):
        return self.material

    def to_veg_file(self) -> VegFile:
        payload = _core.extract_veg_payload_from_volume_mesh(self._handle)
        return VegFile(
            mesh_data=_wrap_mesh_data_core(payload.mesh_data),
            materials=[_wrap_material_payload(m) for m in payload.materials],
            sets=[MeshSet(name, list(elements)) for name, elements in payload.sets],
            regions=[MeshRegion(mi, si) for mi, si in payload.regions],
        )

    def __repr__(self) -> str:
        return (
            f"VolumeMesh(type={self.mesh_type}, "
            f"vertices={self.num_vertices}, elements={self.num_elements})"
        )


# ---------------------------------------------------------------------------
# .veg I/O
# ---------------------------------------------------------------------------


def read_msh(path: str) -> TetMeshData:
    """Load a Gmsh .msh file and return a TetMeshData (geometry only, no material)."""
    return _wrap_mesh_data_core(_core.read_msh(str(path)))


def _mesh_data_from_veg_record(mesh_kind, vertices, elements):
    vertex_array = np.asarray(vertices, dtype=np.float64).reshape(-1, 3)
    if mesh_kind == "tet":
        return TetMeshData(vertex_array, np.asarray(elements, dtype=np.int64).reshape(-1, 4))
    if mesh_kind == "cubic":
        return CubicMeshData(vertex_array, np.asarray(elements, dtype=np.int64).reshape(-1, 8))
    raise RuntimeError(f"Unexpected veg mesh kind from _core: {mesh_kind!r}")


def _material_from_veg_record(record):
    kind = record[0]
    if kind == "enu":
        _, name, density, E, nu = record
        return ENuMaterial(name, density=float(density), E=float(E), nu=float(nu))
    if kind == "mooney_rivlin":
        _, name, density, mu01, mu10, v1 = record
        return MooneyRivlinMaterial(
            name, density=float(density), mu01=float(mu01), mu10=float(mu10), v1=float(v1))
    raise RuntimeError(f"Unexpected material payload from _core: {kind!r}")


def read_veg(path: str) -> VegFile:
    mesh_kind, vertices, elements, materials, sets, regions = _core.read_veg(str(path))
    return VegFile(
        mesh_data=_mesh_data_from_veg_record(mesh_kind, vertices, elements),
        materials=[_material_from_veg_record(m) for m in materials],
        sets=[MeshSet(name, list(elements)) for name, elements in sets],
        regions=[MeshRegion(material_index, set_index) for material_index, set_index in regions],
    )


def write_veg(path: str, veg: VegFile) -> None:
    if not isinstance(veg.mesh_data, (TetMeshData, CubicMeshData)):
        raise TypeError(f"mesh_data must be a TetMeshData or CubicMeshData, got {type(veg.mesh_data).__name__}")
    _core.write_veg(
        str(path),
        veg.mesh_data._handle,
        [_material_to_core_payload(m) for m in veg.materials],
        [(s.name, list(s.elements)) for s in veg.sets],
        [(r.material_index, r.set_index) for r in veg.regions],
    )
