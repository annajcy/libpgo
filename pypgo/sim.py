"""Solver-facing simulation mesh and shell material helpers."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

import pypgo._core as _core
from pypgo.mesh import TriMeshData, read_obj, write_obj


@dataclass(frozen=True)
class KoiterStVKShellMaterial:
    name: str = "shell"
    thickness: float = 0.001
    E_membrane: float = 1e6
    nu_membrane: float = 0.4


ShellMaterialLike = KoiterStVKShellMaterial


class SimulationMesh:
    """Solver-ready simulation mesh created by explicit factory methods."""

    def __init__(self, core_obj):
        if not isinstance(core_obj, _core.SimulationMeshCore):
            raise TypeError(f"core_obj must be SimulationMeshCore, got {type(core_obj).__name__}")
        self._core_obj = core_obj

    @classmethod
    def create_volumetric(cls, volume_mesh) -> "SimulationMesh":
        from pypgo.mesh.veg import VolumeMesh

        if not isinstance(volume_mesh, VolumeMesh):
            raise TypeError(f"volume_mesh must be a VolumeMesh, got {type(volume_mesh).__name__}")
        return cls(_core.create_simulation_mesh_from_volume(volume_mesh._core_obj))

    @classmethod
    def create_shell(cls, surface: TriMeshData, material: ShellMaterialLike) -> "SimulationMesh":
        if not isinstance(surface, TriMeshData):
            raise TypeError(f"surface must be a TriMeshData, got {type(surface).__name__}")
        if not isinstance(material, KoiterStVKShellMaterial):
            raise TypeError(f"material must be a KoiterStVKShellMaterial, got {type(material).__name__}")
        return cls(_core.create_simulation_mesh_from_shell(
            surface._core_obj,
            float(material.thickness),
            float(material.E_membrane),
            float(material.nu_membrane),
        ))

    @property
    def mesh_type(self) -> str:
        return self._core_obj.mesh_type()

    @property
    def num_vertices(self) -> int:
        return self._core_obj.num_vertices()

    @property
    def num_elements(self) -> int:
        return self._core_obj.num_elements()

    @property
    def num_element_vertices(self) -> int:
        return self._core_obj.num_element_vertices()


def write_shell(path, surface: TriMeshData, material: ShellMaterialLike) -> None:
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


def read_shell(path):
    shell_path = Path(path)
    payload = json.loads(shell_path.read_text())
    surface = read_obj(str(shell_path.parent / payload["mesh_obj"]))
    material_payload = dict(payload["material"])
    kind = material_payload.pop("kind")
    if kind != "KoiterStVKShellMaterial":
        raise ValueError(f"unsupported shell material kind: {kind}")
    return surface, KoiterStVKShellMaterial(**material_payload)
