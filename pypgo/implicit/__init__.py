"""pypgo.implicit — lazy implicit surface fields and extraction helpers.

``ImplicitField`` is the stable contract; shape primitives (and the
``thicken_mesh_surface`` pipeline that builds on them) live in ``fields.py``
and are the side expected to grow.  ``grid`` holds the sampling grid spec, and
``extract`` the marching-cubes surface extraction plus the optional OpenVDB
level-set backend.
"""

from pypgo.implicit.grid import GridSpec
from pypgo.implicit.base import ImplicitField, GridField
from pypgo.implicit.extract import (
    OpenVDBOptions,
    build_openvdb_from_grid_field,
    build_openvdb_shell_from_mesh,
    extract_marching_cubes,
    extract_openvdb,
    has_openvdb,
)
from pypgo.implicit.fields import (
    BoxField,
    MeshUnsignedDistanceField,
    SphereField,
    thicken_mesh_surface,
)

__all__ = [
    "BoxField",
    "GridField",
    "GridSpec",
    "ImplicitField",
    "MeshUnsignedDistanceField",
    "OpenVDBOptions",
    "SphereField",
    "build_openvdb_from_grid_field",
    "build_openvdb_shell_from_mesh",
    "extract_marching_cubes",
    "extract_openvdb",
    "has_openvdb",
    "thicken_mesh_surface",
]
