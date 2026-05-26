"""Simulation mesh and material types for pypgo.

Exposes MaterialSpec (value object) and VolumeMesh (simulation mesh).
"""

import pypgo._core as _core


class MaterialSpec:
    """Homogeneous ENu material specification."""

    def __init__(self, E: float = 1e9, nu: float = 0.45, density: float = 1000.0, *, _core_obj=None):
        if _core_obj is not None:
            if not isinstance(_core_obj, _core.MaterialSpecCore):
                raise TypeError("_core_obj must be a MaterialSpecCore instance")
            self._core_obj = _core_obj
        else:
            self._core_obj = _core.MaterialSpecCore(float(E), float(nu), float(density))

    @property
    def E(self) -> float:
        return self._core_obj.E()

    @property
    def nu(self) -> float:
        return self._core_obj.nu()

    @property
    def density(self) -> float:
        return self._core_obj.density()

    def __repr__(self) -> str:
        return f"MaterialSpec(E={self.E}, nu={self.nu}, density={self.density})"


class VolumeMesh:
    """Simulation volume mesh.

    Can be constructed in two ways:

    1. From geometry + material (programmatic construction)::

        vol = VolumeMesh(tet_geo, material)

    2. From a .veg file (zero redundant construction)::

        vol = VolumeMesh.load("box.veg")
    """

    def __init__(self, cell_mesh, material_spec: MaterialSpec):
        from pypgo.mesh_geo import TetCellMeshGeo, CubicCellMeshGeo

        if not isinstance(material_spec, MaterialSpec):
            raise TypeError(f"material_spec must be a MaterialSpec, got {type(material_spec).__name__}")

        if not isinstance(cell_mesh, (TetCellMeshGeo, CubicCellMeshGeo)):
            raise TypeError(
                f"cell_mesh must be a TetCellMeshGeo or CubicCellMeshGeo, got {type(cell_mesh).__name__}"
            )

        self._geometry = cell_mesh
        self._material = material_spec
        self._core_obj = _core.create_volume_mesh(cell_mesh._core_obj, material_spec._core_obj)

    @classmethod
    def load(cls, path: str) -> "VolumeMesh":
        """Load a VolumeMesh directly from a .veg file.

        This is the preferred way to load .veg files when you need a simulation
        mesh. Unlike ``read_veg_geo()`` followed by ``VolumeMesh(geo, mat)``,
        this path constructs the underlying C++ VolumetricMesh only once.
        """
        obj = cls.__new__(cls)
        obj._core_obj = _core.load_volume_mesh(str(path))
        obj._geometry = None  # lazy
        obj._material = None  # lazy
        return obj

    def save(self, path: str) -> None:
        """Save this VolumeMesh to a .veg file."""
        _core.save_volume_mesh(str(path), self._core_obj)

    @property
    def num_vertices(self) -> int:
        return self._core_obj.num_vertices()

    @property
    def num_elements(self) -> int:
        return self._core_obj.num_elements()

    @property
    def mesh_type(self):
        return self._core_obj.mesh_type()

    @property
    def geometry(self):
        """The geometry of this mesh (lazy export from C++ VolumetricMesh)."""
        if self._geometry is None:
            from pypgo.mesh_geo import TetCellMeshGeo, CubicCellMeshGeo
            core_geo = self._core_obj.export_geometry()
            if isinstance(core_geo, _core.TetCellMeshGeoCore):
                self._geometry = TetCellMeshGeo(core_geo)
            elif isinstance(core_geo, _core.CubicCellMeshGeoCore):
                self._geometry = CubicCellMeshGeo(core_geo)
            else:
                raise RuntimeError(f"Unexpected core geometry type: {type(core_geo).__name__}")
        return self._geometry

    @property
    def cell_mesh(self):
        """Backward-compatible alias for geometry."""
        return self.geometry

    @property
    def material(self) -> MaterialSpec:
        """The material of this mesh (lazy export from C++ VolumetricMesh)."""
        if self._material is None:
            self._material = MaterialSpec(_core_obj=self._core_obj.export_material())
        return self._material

    @property
    def material_spec(self) -> MaterialSpec:
        """Backward-compatible alias for material."""
        return self.material

    def __repr__(self) -> str:
        return (
            f"VolumeMesh(type={self.mesh_type}, "
            f"vertices={self.num_vertices}, elements={self.num_elements})"
        )
