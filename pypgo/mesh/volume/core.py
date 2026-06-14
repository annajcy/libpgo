"""Volume mesh wrapper, VegFile data model, and .veg I/O."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

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

    Use pypgo.fem.SimulationMesh factory methods for solver-ready meshes.
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

        Prefer ``pypgo.fem.VolumeDensity`` + ``formulation.mass_matrix(sim_mesh, mass_field)``.
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


def _compact(text: str) -> str:
    return "".join(text.split())


def _parse_ints(text: str) -> list[int]:
    return [int(value) for value in text.replace(",", " ").split()]


def _parse_floats(text: str) -> list[float]:
    return [float(value) for value in text.replace(",", " ").split()]


def _read_ascii_lines(path: Path) -> list[str]:
    lines: list[str] = []
    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("*INCLUDE "):
            lines.extend(_read_ascii_lines(path.parent / line[9:].strip()))
            continue
        lines.append(line)
    return lines


def _parse_material(name: str, spec: str) -> MaterialLike:
    if "," not in spec:
        raise RuntimeError(f"Malformed material '{name}': {spec}")
    material_type, values_text = spec.split(",", 1)
    material_type = _compact(material_type).upper()
    values = _parse_floats(values_text)

    if material_type == "ENU":
        if len(values) < 3:
            raise RuntimeError(f"ENU material '{name}' requires density, E, and nu")
        return ENuMaterial(name, density=values[0], E=values[1], nu=values[2])

    if material_type.startswith("MOONEYRIVLIN"):
        if len(values) < 4:
            raise RuntimeError(f"Mooney-Rivlin material '{name}' requires density, mu01, mu10, and v1")
        return MooneyRivlinMaterial(name, density=values[0], mu01=values[1], mu10=values[2], v1=values[3])

    raise RuntimeError(f"Unsupported material type in ASCII .veg file: {material_type}")


def _read_ascii_veg(path: str) -> VegFile:
    veg_path = Path(path)
    lines = _read_ascii_lines(veg_path)
    vertices: list[list[float]] = []
    elements: list[list[int]] = []
    element_width = 0
    parsed_num_elements = 0
    one_indexed_vertices = True
    one_indexed_elements = True
    materials: list[MaterialLike] = []
    sets: list[MeshSet] = []
    regions: list[MeshRegion] = []
    material_map: dict[str, int] = {}
    set_map: dict[str, int] = {}

    i = 0
    while i < len(lines):
        line = lines[i].strip()

        if line.startswith("*VERTICES"):
            i += 1
            if i >= len(lines):
                raise RuntimeError(f"Missing *VERTICES header in {path}")
            header = _parse_ints(lines[i])
            i += 1
            if not header or header[0] < 0:
                raise RuntimeError(f"Invalid vertex count in {path}")
            for _ in range(header[0]):
                if i >= len(lines):
                    raise RuntimeError(f"Missing vertex row in {path}")
                values = _parse_floats(lines[i])
                i += 1
                if len(values) < 4:
                    raise RuntimeError(f"Malformed vertex row in {path}: {lines[i - 1]}")
                if int(values[0]) == 0:
                    one_indexed_vertices = False
                vertices.append([values[1], values[2], values[3]])
            continue

        if line.startswith("*ELEMENTS"):
            i += 1
            if i >= len(lines):
                raise RuntimeError(f"Missing element type in {path}")
            element_type = _compact(lines[i]).upper()
            i += 1
            if element_type == "TET":
                element_width = 4
            elif element_type == "CUBIC":
                element_width = 8
            else:
                raise RuntimeError(f"Unsupported element type in {path}: {element_type}")

            if i >= len(lines):
                raise RuntimeError(f"Missing element count in {path}")
            header = _parse_ints(lines[i])
            i += 1
            if not header or header[0] < 0:
                raise RuntimeError(f"Invalid element count in {path}")
            parsed_num_elements = header[0]
            vertex_offset = 1 if one_indexed_vertices else 0
            for _ in range(parsed_num_elements):
                if i >= len(lines):
                    raise RuntimeError(f"Missing element row in {path}")
                values = _parse_ints(lines[i])
                i += 1
                if len(values) < element_width + 1:
                    raise RuntimeError(f"Malformed element row in {path}: {lines[i - 1]}")
                if values[0] == 0:
                    one_indexed_elements = False
                elements.append([v - vertex_offset for v in values[1:element_width + 1]])
            if not sets:
                set_map["allElements"] = 0
                sets.append(MeshSet("allElements", list(range(parsed_num_elements))))
            continue

        if line.startswith("*MATERIAL"):
            name = _compact(line[9:])
            i += 1
            if i >= len(lines):
                raise RuntimeError(f"Missing material payload in {path}")
            material_map[name] = len(materials)
            materials.append(_parse_material(name, lines[i]))
            i += 1
            continue

        if line.startswith("*SET"):
            name = _compact(line[4:])
            element_offset = 1 if one_indexed_elements else 0
            set_elements: list[int] = []
            i += 1
            while i < len(lines) and not lines[i].startswith("*"):
                set_elements.extend(element - element_offset for element in _parse_ints(lines[i]))
                i += 1
            set_map[name] = len(sets)
            sets.append(MeshSet(name, set_elements))
            continue

        if line.startswith("*REGION"):
            i += 1
            if i >= len(lines):
                raise RuntimeError(f"Missing region payload in {path}")
            spec = _compact(lines[i])
            i += 1
            if "," not in spec:
                raise RuntimeError(f"Malformed region line in {path}: {spec}")
            set_name, material_name = spec.split(",", 1)
            if set_name not in set_map:
                raise RuntimeError(f"Region references unknown set in {path}: {set_name}")
            if material_name not in material_map:
                raise RuntimeError(f"Region references unknown material in {path}: {material_name}")
            regions.append(MeshRegion(material_map[material_name], set_map[set_name]))
            continue

        i += 1

    if element_width == 4:
        mesh_data = TetMeshData(vertices, elements)
    elif element_width == 8:
        mesh_data = CubicMeshData(vertices, elements)
    else:
        raise RuntimeError(f"No *ELEMENTS section found in {path}")

    if not materials:
        materials.append(ENuMaterial())
    if not regions:
        regions.append(MeshRegion(len(materials) - 1, 0))

    return VegFile(mesh_data=mesh_data, materials=materials, sets=sets, regions=regions)


def read_msh(path: str) -> TetMeshData:
    """Load a Gmsh .msh file and return a TetMeshData (geometry only, no material)."""
    return _wrap_mesh_data_core(_core.read_msh(str(path)))


def read_veg(path: str) -> VegFile:
    if Path(path).suffix.lower() != ".vegb":
        return _read_ascii_veg(str(path))

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
        veg.mesh_data._handle,
        [_material_to_core_payload(m) for m in veg.materials],
        [(s.name, list(s.elements)) for s in veg.sets],
        [(r.material_index, r.set_index) for r in veg.regions],
    )
