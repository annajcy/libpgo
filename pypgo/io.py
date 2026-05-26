"""File I/O for pypgo mesh data types."""

import pypgo._core as _core
from pypgo.mesh_geo import CubicMeshData, TetMeshData, TriMeshData
from pypgo.mesh import MaterialSpec


def read_veg_geo(path: str):
    """Read a .veg file and return (mesh_data, material_spec).

    Returns:
        tuple[TetMeshData | CubicMeshData, MaterialSpec]
    """
    data_core, mat_core = _core.read_veg_geo(str(path))

    if isinstance(data_core, _core.TetMeshDataCore):
        data = TetMeshData(data_core)
    elif isinstance(data_core, _core.CubicMeshDataCore):
        data = CubicMeshData(data_core)
    else:
        raise RuntimeError(f"Unexpected core type from read_veg_geo: {type(data_core)}")

    return data, MaterialSpec(_core_obj=mat_core)


def write_veg_geo(path: str, mesh_data, material_spec: MaterialSpec) -> None:
    """Write volumetric MeshData and material to a .veg file."""
    if not isinstance(mesh_data, (TetMeshData, CubicMeshData)):
        raise TypeError(f"mesh_data must be a TetMeshData or CubicMeshData, got {type(mesh_data).__name__}")
    if not isinstance(material_spec, MaterialSpec):
        raise TypeError(f"material_spec must be a MaterialSpec, got {type(material_spec).__name__}")
    _core.write_veg_geo(str(path), mesh_data._core_obj, material_spec._core_obj)


def read_obj_geo(path: str) -> TriMeshData:
    """Read an .obj file and return TriMeshData."""
    core_obj = _core.read_obj_geo(str(path))
    return TriMeshData(core_obj)


def write_obj_geo(path: str, surface_data: TriMeshData) -> None:
    """Write TriMeshData to an .obj file."""
    if not isinstance(surface_data, TriMeshData):
        raise TypeError(f"surface_data must be a TriMeshData, got {type(surface_data).__name__}")
    _core.write_obj_geo(str(path), surface_data._core_obj)
