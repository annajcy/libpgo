# `pypgo.mesh.processing` — Meshing, Repair & Quality

## Purpose

Volume mesh generation, surface repair, and quality diagnostics.

## Package Structure

```
processing/
├── __init__.py       # Re-exports public API
├── volume.py         # Volume meshing (cubic_mesher, tet_mesher) + volume info
└── surface.py        # Surface repair (CGAL) + quality diagnostics
```

## Quick Reference

```python
# Meshing
from pypgo.mesh import cubic_mesher, tet_mesher, has_tetwild

# Quality
from pypgo.mesh import check_surface_quality, QualityReport

# Surface repair
from pypgo.mesh import (
    cgal_smooth, cgal_isotropic_remesh, cgal_simplify,
    cgal_repair_self_intersections, raw_surface_cleanup, merge_close_vertices,
    has_cgal_remesher,
)
```

## Further Reading

- [`volume.md`](volume.md) — `cubic_mesher`, `tet_mesher`, backend checks, `VolumetricMeshInfo`
- [`surface.md`](surface.md) — `check_surface_quality`, CGAL operations, surface cleanup
