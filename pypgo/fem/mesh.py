"""Solver-facing simulation assets and shell import helpers."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

import pypgo._core as _core
from pypgo.mesh import TriMeshData, read_obj, write_obj
from pypgo.fem.elastic import KoiterStVKShellMaterial, ShellMaterialLike


class SimulationAsset:
    """Geometry plus neutral imported material data for one simulation."""

    def __init__(self, core_obj):
        if not isinstance(core_obj, _core.PySimulationAsset):
            raise TypeError(f"core_obj must be PySimulationAsset, got {type(core_obj).__name__}")
        self._handle = core_obj
        self.mesh = SimulationMesh(core_obj)

    @classmethod
    def create_volumetric(cls, volume_mesh) -> "SimulationAsset":
        from pypgo.mesh.volume import VolumeMesh

        if not isinstance(volume_mesh, VolumeMesh):
            raise TypeError(f"volume_mesh must be a VolumeMesh, got {type(volume_mesh).__name__}")
        return cls(_core.create_simulation_asset_from_volume(volume_mesh._handle))

    @classmethod
    def create_shell(cls, surface: TriMeshData, material: ShellMaterialLike) -> "SimulationAsset":
        if not isinstance(surface, TriMeshData):
            raise TypeError(f"surface must be a TriMeshData, got {type(surface).__name__}")
        if not isinstance(material, KoiterStVKShellMaterial):
            raise TypeError(f"material must be a KoiterStVKShellMaterial, got {type(material).__name__}")
        return cls(_core.create_simulation_asset_from_shell(
            surface._handle,
            float(material.thickness),
            float(material.E_membrane),
            float(material.nu_membrane),
        ))

    @property
    def mesh_type(self) -> str:
        return self._handle.mesh_type()

    @property
    def num_vertices(self) -> int:
        return self._handle.num_vertices()

    @property
    def num_elements(self) -> int:
        return self._handle.num_elements()

    @property
    def num_element_vertices(self) -> int:
        return self._handle.num_element_vertices()

    @property
    def material_data(self):
        return ImportedMaterialData(self._handle.material_data)


class SimulationMesh:
    """Geometry-only view owned by a :class:`SimulationAsset`."""

    def __init__(self, core_obj):
        if not isinstance(core_obj, _core.PySimulationMesh):
            raise TypeError(
                f"core_obj must be PySimulationMesh, got {type(core_obj).__name__}"
            )
        self._handle = core_obj

    @property
    def mesh_type(self) -> str:
        return self._handle.mesh_type()

    @property
    def num_vertices(self) -> int:
        return self._handle.num_vertices()

    @property
    def num_elements(self) -> int:
        return self._handle.num_elements()

    @property
    def num_element_vertices(self) -> int:
        return self._handle.num_element_vertices()


class ImportedMaterialRecord:
    def __init__(self, handle):
        self._handle = handle

    @property
    def name(self) -> str:
        return self._handle.name

    @property
    def family(self) -> str:
        return self._handle.family

    @property
    def properties(self) -> dict:
        return dict(self._handle.properties)


class ImportedElementSet:
    def __init__(self, handle):
        self._handle = handle

    @property
    def name(self) -> str:
        return self._handle.name

    @property
    def elements(self) -> list[int]:
        return [int(v) for v in self._handle.elements]


class ImportedMaterialRegion:
    def __init__(self, handle):
        self._handle = handle

    @property
    def material_index(self) -> int:
        return int(self._handle.material_index)

    @property
    def set_index(self) -> int:
        return int(self._handle.set_index)


class ImportedMaterialField:
    def __init__(self, handle):
        self._handle = handle

    @property
    def name(self) -> str:
        return self._handle.name

    @property
    def channel_names(self) -> tuple[str, ...]:
        return tuple(self._handle.channel_names)

    @property
    def value_rows(self) -> np.ndarray:
        return np.asarray(self._handle.value_rows, dtype=np.float64)

    @property
    def element_to_row(self) -> np.ndarray:
        return np.asarray(self._handle.element_to_row, dtype=np.int64)


class ImportedMaterialData:
    def __init__(self, handle):
        if not isinstance(handle, _core.PyImportedMaterialData):
            raise TypeError(
                "handle must be PyImportedMaterialData, "
                f"got {type(handle).__name__}"
            )
        self._handle = handle

    @property
    def num_elements(self) -> int:
        return int(self._handle.num_elements)

    @property
    def materials(self) -> tuple[ImportedMaterialRecord, ...]:
        return tuple(ImportedMaterialRecord(v) for v in self._handle.materials)

    @property
    def sets(self) -> tuple[ImportedElementSet, ...]:
        return tuple(ImportedElementSet(v) for v in self._handle.sets)

    @property
    def regions(self) -> tuple[ImportedMaterialRegion, ...]:
        return tuple(ImportedMaterialRegion(v) for v in self._handle.regions)

    @property
    def fields(self) -> tuple[ImportedMaterialField, ...]:
        return tuple(ImportedMaterialField(v) for v in self._handle.fields)

    @property
    def element_material_indices(self) -> np.ndarray:
        return np.asarray(self._handle.element_material_indices, dtype=np.int64)


def write_shell_config(path, surface: TriMeshData, material: ShellMaterialLike) -> None:
    if not isinstance(surface, TriMeshData):
        raise TypeError(f"surface must be a TriMeshData, got {type(surface).__name__}")
    if not isinstance(material, KoiterStVKShellMaterial):
        raise TypeError(f"material must be a KoiterStVKShellMaterial, got {type(material).__name__}")

    shell_path = Path(path)
    obj_path = shell_path.with_suffix("").with_suffix(".obj")
    write_obj(str(obj_path), surface)
    payload = {
        "mesh_obj": obj_path.name,
        "material": {
            "kind": "KoiterStVKShellMaterial",
            **asdict(material),
        },
    }
    shell_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def read_shell_config(path):
    shell_path = Path(path)
    payload = json.loads(shell_path.read_text())
    surface = read_obj(str(shell_path.parent / payload["mesh_obj"]))
    material_payload = dict(payload["material"])
    kind = material_payload.pop("kind")
    if kind != "KoiterStVKShellMaterial":
        raise ValueError(f"unsupported shell material kind: {kind}")
    return surface, KoiterStVKShellMaterial(**material_payload)
