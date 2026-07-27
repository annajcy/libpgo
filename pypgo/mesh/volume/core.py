"""Volume mesh wrapper, VegFile data model, and .veg I/O."""

from __future__ import annotations

from dataclasses import dataclass

import pypgo._core as _core
from pypgo.mesh.data import CubicMeshData, TetMeshData, TriMeshData, _wrap_mesh_data_core
from pypgo.mesh.volume.material import (
    MaterialLike,
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

    def _to_core_payload(self):
        return _core._create_veg_payload(
            self.mesh_data._handle,
            [_material_to_core_payload(material) for material in self.materials],
            [(mesh_set.name, list(mesh_set.elements)) for mesh_set in self.sets],
            [
                (region.material_index, region.set_index)
                for region in self.regions
            ],
        )

    @classmethod
    def _from_core_payload(cls, payload) -> "VegFile":
        return cls(
            mesh_data=_wrap_mesh_data_core(payload.mesh_data),
            materials=[
                _wrap_material_payload(material)
                for material in payload.materials
            ],
            sets=[
                MeshSet(name, list(elements))
                for name, elements in payload.sets
            ],
            regions=[
                MeshRegion(material_index, set_index)
                for material_index, set_index in payload.regions
            ],
        )


# ---------------------------------------------------------------------------
# VolumeMesh
# ---------------------------------------------------------------------------


class VolumeMesh:
    """Native Vega volume mesh constructed from a lossless :class:`VegFile`."""

    def __init__(self, source, material: MaterialLike | None = None):
        if isinstance(source, VegFile):
            if material is not None:
                raise TypeError("material must be omitted when source is a VegFile")
            veg = source
        elif isinstance(source, (TetMeshData, CubicMeshData)):
            if material is None:
                raise TypeError(
                    "VolumeMesh(mesh_data, material) requires a material"
                )
            veg = VegFile.from_single_material(source, material)
        else:
            raise TypeError(
                "source must be a VegFile or TetMeshData or CubicMeshData"
            )

        self._mesh_data = veg.mesh_data
        self._handle = _core._create_volume_mesh_from_veg_payload(
            veg._to_core_payload()
        )

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
        return VegFile._from_core_payload(
            _core.extract_veg_payload_from_volume_mesh(self._handle)
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


def read_veg(path: str) -> VegFile:
    return VegFile._from_core_payload(
        _core._read_veg_payload(str(path))
    )


def write_veg(path: str, veg: VegFile) -> None:
    if not isinstance(veg, VegFile):
        raise TypeError(f"veg must be a VegFile, got {type(veg).__name__}")
    _core._write_veg_payload(str(path), veg._to_core_payload())
