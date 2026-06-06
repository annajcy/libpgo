# `pypgo.mesh.geo` — Geometry

## Purpose

Geometry facades, barycentric/surface embedding, and pure-Python geometry algorithms.

## Package Structure

```
geo/
├── __init__.py       # Re-exports public API
├── core.py           # C++ thin wrappers (TriMeshGeo, BarycentricEmbedding, SurfaceEmbedding)
└── algorithms.py     # Pure-Python algorithms (connected components, bounding sphere)
```

## Quick Reference

```python
# Geometry facades
from pypgo.mesh.geo import TriMeshGeo, TetMeshGeo, CubicMeshGeo
geo = TriMeshGeo.from_mesh_data(tri_data)
print(geo.face_areas, geo.vertex_normals)

# Embedding
from pypgo.mesh.geo import BarycentricEmbedding, SurfaceEmbedding

# Algorithms
from pypgo.mesh.geo import (
    minimum_bounding_sphere, split_components,
    connected_components_by_edge, filter_small_components,
)
```

## Further Reading

- [`core.md`](core.md) — `TriMeshGeo`, `TetMeshGeo`, `CubicMeshGeo`, `BarycentricEmbedding`, `SurfaceEmbedding`
- [`algorithms.md`](algorithms.md) — Connected components, `minimum_bounding_sphere`
