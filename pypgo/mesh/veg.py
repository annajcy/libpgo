"""Vega volumetric mesh I/O, materials, regions, and volume wrapper."""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar, Literal

import pypgo._core as _core
from pypgo.mesh import CubicMeshData, TetMeshData, TriMeshData, _wrap_mesh_data_core


@dataclass
class ENuMaterial:
    """Linear isotropic material, represented as a pure Python data carrier."""

    name: str = "defaultMaterial"
    density: float = 1000.0
    E: float = 1e9
    nu: float = 0.45
    type: ClassVar[Literal["enu"]] = "enu"

    @property
    def lam(self) -> float:
        return self.E * self.nu / ((1 + self.nu) * (1 - 2 * self.nu))

    @property
    def mu(self) -> float:
        return self.E / (2 * (1 + self.nu))


MaterialSpec = ENuMaterial


@dataclass
class MooneyRivlinMaterial:
    """Mooney-Rivlin hyperelastic material, represented as Python data."""

    name: str = "mooneyRivlinMaterial"
    density: float = 1000.0
    mu01: float = 0.0
    mu10: float = 0.0
    v1: float = 0.0
    type: ClassVar[Literal["mooney_rivlin"]] = "mooney_rivlin"


@dataclass
class OrthotropicMaterial:
    """Orthotropic material, represented as Python data."""

    name: str = "orthotropicMaterial"
    density: float = 1000.0
    E1: float = 0.0
    E2: float = 0.0
    E3: float = 0.0
    nu12: float = 0.0
    nu23: float = 0.0
    nu31: float = 0.0
    G12: float = 0.0
    G23: float = 0.0
    G31: float = 0.0
    R: tuple[float, float, float, float, float, float, float, float, float] = (
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    )
    type: ClassVar[Literal["orthotropic"]] = "orthotropic"


MaterialLike = ENuMaterial | MooneyRivlinMaterial | OrthotropicMaterial


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


def _wrap_material_payload(m) -> MaterialLike:
    if isinstance(m, _core.VegENuMaterialPayloadCore):
        return ENuMaterial(m.name, density=m.density, E=m.E, nu=m.nu)
    if isinstance(m, _core.VegMooneyRivlinMaterialPayloadCore):
        return MooneyRivlinMaterial(
            m.name, density=m.density, mu01=m.mu01, mu10=m.mu10, v1=m.v1)
    if isinstance(m, _core.VegOrthotropicMaterialPayloadCore):
        return OrthotropicMaterial(
            m.name, density=m.density,
            E1=m.E1, E2=m.E2, E3=m.E3,
            nu12=m.nu12, nu23=m.nu23, nu31=m.nu31,
            G12=m.G12, G23=m.G23, G31=m.G31,
            R=tuple(m.R),
        )
    raise RuntimeError(f"Unexpected material payload from _core: {type(m).__name__}")


def _material_to_core_payload(m: MaterialLike):
    if isinstance(m, ENuMaterial):
        return _core.create_enu_material_payload(m.name, m.density, m.E, m.nu)
    if isinstance(m, MooneyRivlinMaterial):
        return _core.create_mooney_rivlin_material_payload(
            m.name, m.density, m.mu01, m.mu10, m.v1)
    if isinstance(m, OrthotropicMaterial):
        if len(m.R) != 9:
            raise ValueError("OrthotropicMaterial.R must contain 9 row-major values")
        return _core.create_orthotropic_material_payload(
            m.name, m.density,
            m.E1, m.E2, m.E3,
            m.nu12, m.nu23, m.nu31,
            m.G12, m.G23, m.G31,
            tuple(float(x) for x in m.R),
        )
    raise TypeError(f"unsupported material type: {type(m).__name__}")


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


class VolumeMesh:
    """Vega FEM volumetric mesh wrapper.

    Use pypgo.sim.SimulationMesh factory methods for solver-ready meshes.
    """

    def __init__(self, mesh_data, regions):
        if not isinstance(mesh_data, (TetMeshData, CubicMeshData)):
            raise TypeError(f"mesh_data must be a TetMeshData or CubicMeshData, got {type(mesh_data).__name__}")

        self._mesh_data = mesh_data
        self._material = None
        materials, sets, region_payloads = _validate_and_split_regions(regions, mesh_data.num_elements)
        self._core_obj = _core.create_volume_mesh_multi(
            mesh_data._core_obj, materials, sets, region_payloads)

    @classmethod
    def create_from_single_material(cls, mesh_data, material) -> "VolumeMesh":
        if not isinstance(mesh_data, (TetMeshData, CubicMeshData)):
            raise TypeError(f"mesh_data must be a TetMeshData or CubicMeshData, got {type(mesh_data).__name__}")
        if not isinstance(material, (ENuMaterial, MooneyRivlinMaterial, OrthotropicMaterial)):
            raise TypeError(f"material must be a veg material type, got {type(material).__name__}")
        obj = cls.__new__(cls)
        obj._mesh_data = mesh_data
        obj._material = material
        obj._core_obj = _core.create_volume_mesh_multi(
            mesh_data._core_obj,
            [_material_to_core_payload(material)],
            [("allElements", list(range(mesh_data.num_elements)))],
            [(0, 0)],
        )
        return obj

    @classmethod
    def load(cls, path: str) -> "VolumeMesh":
        obj = cls.__new__(cls)
        obj._core_obj = _core.load_volume_mesh(str(path))
        obj._mesh_data = None
        obj._material = None
        return obj

    def save(self, path: str) -> None:
        _core.save_volume_mesh(str(path), self._core_obj)

    def extract_surface_mesh(self, *, triangulate: bool = True) -> TriMeshData:
        return TriMeshData(_core.extract_surface_mesh(self._core_obj, bool(triangulate)))

    @property
    def num_vertices(self) -> int:
        return self._core_obj.num_vertices()

    @property
    def num_elements(self) -> int:
        return self._core_obj.num_elements()

    @property
    def mesh_type(self):
        return self._core_obj.mesh_type()

    @property
    def mesh_data(self):
        if self._mesh_data is None:
            self._mesh_data = _wrap_mesh_data_core(self._core_obj.export_geometry())
        return self._mesh_data

    @property
    def geometry(self):
        return self.mesh_data

    @property
    def material(self):
        if self._material is None:
            self._material = _wrap_material_payload(self._core_obj.export_material_payload())
        return self._material

    @property
    def material_spec(self):
        return self.material

    def __repr__(self) -> str:
        return (
            f"VolumeMesh(type={self.mesh_type}, "
            f"vertices={self.num_vertices}, elements={self.num_elements})"
        )


def read_veg(path: str) -> VegFile:
    payload = _core.read_veg(str(path))
    return VegFile(
        mesh_data=_wrap_mesh_data_core(payload.mesh_data),
        materials=[_wrap_material_payload(m) for m in payload.materials],
        sets=[MeshSet(name, list(elements)) for name, elements in payload.sets],
        regions=[MeshRegion(material_index, set_index) for material_index, set_index in payload.regions],
    )


def write_veg(path: str, veg: VegFile) -> None:
    if not isinstance(veg.mesh_data, (TetMeshData, CubicMeshData)):
        raise TypeError(f"mesh_data must be a TetMeshData or CubicMeshData, got {type(veg.mesh_data).__name__}")
    _core.write_veg(
        str(path),
        veg.mesh_data._core_obj,
        [_material_to_core_payload(m) for m in veg.materials],
        [(s.name, list(s.elements)) for s in veg.sets],
        [(r.material_index, r.set_index) for r in veg.regions],
    )
