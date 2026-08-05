"""Solver-facing geometry and topology."""

from __future__ import annotations

import pypgo._core as _core
from pypgo.mesh import TriMeshData


class SimulationMesh:
    """Standalone FEM geometry and topology."""

    def __init__(self, source):
        from pypgo.mesh.volume import VolumeMesh

        if isinstance(source, VolumeMesh):
            self._handle = _core._create_volume_simulation_mesh(source._handle)
        elif isinstance(source, TriMeshData):
            self._handle = _core._create_shell_simulation_mesh(source._handle)
        else:
            raise TypeError(
                "source must be a VolumeMesh or TriMeshData, "
                f"got {type(source).__name__}"
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


__all__ = ["SimulationMesh"]
