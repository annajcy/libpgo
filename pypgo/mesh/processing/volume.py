"""Volume mesh generation and geometric diagnostics."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import pypgo._core as _core
from pypgo.mesh.data import CubicMeshData, TetMeshData, TriMeshData


# ---------------------------------------------------------------------------
# Backend availability
# ---------------------------------------------------------------------------


def has_tetwild() -> bool:
    return bool(_core.has_tetwild())


def has_cgal_remesher() -> bool:
    return bool(_core.has_cgal_remesher())


# ---------------------------------------------------------------------------
# Volume mesh generators
# ---------------------------------------------------------------------------


def cubic_mesher(
    tri_data: TriMeshData,
    *,
    resolution: int,
    E: float = 1e6,
    nu: float = 0.45,
    density: float = 1000.0,
) -> CubicMeshData:
    """Voxelize a closed triangle surface into a cubic volume mesh."""
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")
    return CubicMeshData(
        _core.cubic_mesher(
            tri_data._handle,
            int(resolution),
            float(E),
            float(nu),
            float(density),
        )
    )


def tet_mesher(tri_data: TriMeshData, *, backend: str = "tetgen", config: dict | None = None) -> TetMeshData:
    """Tetrahedralize a closed triangle surface with tetgen or tetwild."""
    if not isinstance(tri_data, TriMeshData):
        raise TypeError(f"tri_data must be a TriMeshData, got {type(tri_data).__name__}")

    config = {} if config is None else dict(config)
    backend = str(backend)
    if backend == "tetwild" and not has_tetwild():
        raise RuntimeError("tetwild backend is not available")

    tetgen_command = str(config.get("command", "pq1.414"))
    tetwild_la = float(config.get("la", 0.0))
    return TetMeshData(
        _core.tet_mesher(
            tri_data._handle,
            backend,
            tetgen_command,
            float(config.get("lr", 0.05)),
            tetwild_la,
            "la" in config,
            float(config.get("epsr", 0.001)),
            float(config.get("stop_energy", 10.0)),
            int(config.get("max_threads", 0)),
        )
    )


# ---------------------------------------------------------------------------
# Volume mesh info
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class VolumetricMeshInfo:
    num_vertices: int
    num_elements: int
    num_element_vertices: int
    total_volume: float
    center_of_mass: np.ndarray

    def __str__(self) -> str:
        cx, cy, cz = self.center_of_mass
        return (
            f"#vtx:              {self.num_vertices}\n"
            f"#elements:         {self.num_elements}\n"
            f"#element vertices: {self.num_element_vertices}\n"
            f"total volume:      {self.total_volume:.17g}\n"
            f"center of mass:    {cx:.17g} {cy:.17g} {cz:.17g}"
        )


def volume_mesh_info(mesh) -> VolumetricMeshInfo:
    """Geometric summary of a volume mesh.

    Accepts TetMeshData, CubicMeshData, or VolumeMesh.
    """
    from pypgo.mesh.volume import VegFile, VolumeMesh

    if isinstance(mesh, (VegFile, VolumeMesh)):
        mesh = mesh.mesh_data
    if not isinstance(mesh, (TetMeshData, CubicMeshData)):
        raise TypeError(
            f"mesh must be a TetMeshData, CubicMeshData, or VolumeMesh, got {type(mesh).__name__}"
        )
    return VolumetricMeshInfo(
        num_vertices=mesh.num_vertices,
        num_elements=mesh.num_elements,
        num_element_vertices=int(mesh.elements.shape[1]),
        total_volume=mesh.volume,
        center_of_mass=mesh.center_of_mass,
    )
