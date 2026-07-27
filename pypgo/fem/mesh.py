"""Solver-facing meshes and neutral material import results."""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo.mesh import TriMeshData


class SimulationImportResult:
    """Mesh and neutral material data produced by one import operation."""

    def __init__(self, source):
        from pypgo.mesh.volume import VolumeMesh

        if isinstance(source, VolumeMesh):
            handle = _core._import_simulation_mesh_from_volume(source._handle)
        else:
            raise TypeError("source must be a VolumeMesh")

        self._handle = handle
        self.mesh = SimulationMesh._from_handle(handle.mesh)

    @classmethod
    def _from_handle(cls, handle):
        if not isinstance(handle, _core.PySimulationImportResult):
            raise TypeError("handle must be PySimulationImportResult")
        result = cls.__new__(cls)
        result._handle = handle
        result.mesh = SimulationMesh._from_handle(handle.mesh)
        return result

    @property
    def mesh_type(self) -> str:
        return self.mesh.mesh_type

    @property
    def num_vertices(self) -> int:
        return self.mesh.num_vertices

    @property
    def num_elements(self) -> int:
        return self.mesh.num_elements

    @property
    def num_element_vertices(self) -> int:
        return self.mesh.num_element_vertices

    @property
    def material_catalog(self):
        return ImportedMaterialCatalog._from_handle(
            self._handle.material_catalog)


class SimulationMesh:
    """Standalone solver geometry and topology."""

    def __init__(self, source):
        if isinstance(source, TriMeshData):
            self._handle = _core._create_shell_simulation_mesh(source._handle)
        else:
            raise TypeError(
                f"source must be TriMeshData, got {type(source).__name__}"
            )

    @classmethod
    def _from_handle(cls, handle):
        if not isinstance(handle, _core.PySimulationMesh):
            raise TypeError(
                f"handle must be PySimulationMesh, got {type(handle).__name__}"
            )
        result = cls.__new__(cls)
        result._handle = handle
        return result

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
    def __init__(self, name, family, properties):
        self._handle = _core.PyImportedMaterialRecord(
            str(name), str(family), dict(properties))

    @classmethod
    def _from_handle(cls, handle):
        result = cls.__new__(cls)
        result._handle = handle
        return result

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
    def __init__(self, name, elements):
        self._handle = _core.PyImportedElementSet(
            str(name), [int(v) for v in elements])

    @classmethod
    def _from_handle(cls, handle):
        result = cls.__new__(cls)
        result._handle = handle
        return result

    @property
    def name(self) -> str:
        return self._handle.name

    @property
    def elements(self) -> list[int]:
        return [int(v) for v in self._handle.elements]


class ImportedMaterialRegion:
    def __init__(self, material_index, set_index):
        self._handle = _core.PyImportedMaterialRegion(
            int(material_index), int(set_index))

    @classmethod
    def _from_handle(cls, handle):
        result = cls.__new__(cls)
        result._handle = handle
        return result

    @property
    def material_index(self) -> int:
        return int(self._handle.material_index)

    @property
    def set_index(self) -> int:
        return int(self._handle.set_index)


class ImportedMaterialCatalog:
    def __init__(self, num_elements, materials, sets, regions):
        self._handle = _core.PyImportedMaterialCatalog(
            int(num_elements),
            [value._handle for value in materials],
            [value._handle for value in sets],
            [value._handle for value in regions])

    @classmethod
    def _from_handle(cls, handle):
        if not isinstance(handle, _core.PyImportedMaterialCatalog):
            raise TypeError("handle must be PyImportedMaterialCatalog")
        result = cls.__new__(cls)
        result._handle = handle
        return result

    @property
    def num_elements(self) -> int:
        return int(self._handle.num_elements)

    @property
    def materials(self) -> tuple[ImportedMaterialRecord, ...]:
        return tuple(ImportedMaterialRecord._from_handle(v) for v in self._handle.materials)

    @property
    def sets(self) -> tuple[ImportedElementSet, ...]:
        return tuple(ImportedElementSet._from_handle(v) for v in self._handle.sets)

    @property
    def regions(self) -> tuple[ImportedMaterialRegion, ...]:
        return tuple(ImportedMaterialRegion._from_handle(v) for v in self._handle.regions)

    @property
    def element_material_indices(self) -> np.ndarray:
        return np.asarray(self._handle.element_material_indices, dtype=np.int64)
