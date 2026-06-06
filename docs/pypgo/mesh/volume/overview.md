# `pypgo.mesh.volume` — Volume Mesh & Materials

## Purpose

Simulation-ready volume mesh wrapper, material definitions, and `.veg` file I/O.

## Package Structure

```
volume/
├── __init__.py       # Re-exports public API
├── core.py           # VolumeMesh, VegFile, .veg I/O
└── material.py       # Material dataclasses (ENuMaterial, MooneyRivlin, Orthotropic)
```

## Quick Reference

```python
# Materials
from pypgo.mesh.volume import ENuMaterial, MooneyRivlinMaterial
mat = ENuMaterial("rubber", E=1e6, nu=0.45)

# Volume mesh
from pypgo.mesh.volume import VolumeMesh
vol = VolumeMesh.create_from_single_material(tet_data, mat)
surf = vol.extract_surface_mesh()
M = vol.mass_matrix()

# .veg I/O
from pypgo.mesh.volume import read_veg, write_veg, VegFile
veg = read_veg("mesh.veg")
```

## Further Reading

- [`core.md`](core.md) — `VolumeMesh`, `VegFile`, `MeshSet`, `MeshRegion`, `read_veg`, `write_veg`
- [`material.md`](material.md) — `ENuMaterial`, `MooneyRivlinMaterial`, `OrthotropicMaterial`
