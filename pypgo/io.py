"""File I/O for pypgo geometry types.

Provides read/write for .veg (volumetric) and .obj (surface) files.
"""

import pypgo._core as _core
from pypgo.mesh_geo import TriCellMeshGeo, TetCellMeshGeo, CubicCellMeshGeo
from pypgo.mesh import MaterialSpec


def read_veg_geo(path: str):
    """Read a .veg file and return (cell_mesh_geo, material_spec).

    Returns:
        tuple[TetCellMeshGeo | CubicCellMeshGeo, MaterialSpec]
    """
    cell_core, mat_core = _core.read_veg_geo(str(path))

    if isinstance(cell_core, _core.TetCellMeshGeoCore):
        geo = TetCellMeshGeo(cell_core)
    elif isinstance(cell_core, _core.CubicCellMeshGeoCore):
        geo = CubicCellMeshGeo(cell_core)
    else:
        raise RuntimeError(f"Unexpected core type from read_veg_geo: {type(cell_core)}")

    return geo, MaterialSpec(_core_obj=mat_core)


def write_veg_geo(path: str, cell_mesh_geo, material_spec: MaterialSpec) -> None:
    """Write a volumetric mesh geometry and material to a .veg file."""
    _core.write_veg_geo(str(path), cell_mesh_geo._core_obj, material_spec._core_obj)


def read_obj_geo(path: str) -> TriCellMeshGeo:
    """Read a .obj file and return a TriCellMeshGeo."""
    core_obj = _core.read_obj_geo(str(path))
    return TriCellMeshGeo(core_obj)


def write_obj_geo(path: str, surface_geo: TriCellMeshGeo) -> None:
    """Write a TriCellMeshGeo to a .obj file."""
    _core.write_obj_geo(str(path), surface_geo._core_obj)
