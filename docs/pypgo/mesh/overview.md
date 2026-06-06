# `pypgo.mesh` — Mesh Data & Geometry

## Purpose

`pypgo.mesh` is pypgo's **mesh infrastructure layer**. It provides:

- **Data containers** — `TriMeshData`, `TetMeshData`, `CubicMeshData` for vertex/element storage
- **Geometry facades** — typed accessors with NumPy-based derived properties (normals, areas, volumes)
- **Volume mesh wrapper** — `VolumeMesh` with material assignment, mass matrix, surface extraction
- **Mesh I/O** — OBJ surface I/O, Vega `.veg` volume I/O
- **Shape factories** — axis-aligned box, sphere, cylinder, torus
- **Mesh generation** — cubic voxelization, tetrahedralization (tetgen / tetwild)
- **Surface processing** — CGAL smoothing, isotropic remeshing, simplification, self-intersection repair
- **Quality diagnostics** — degenerate triangles, short edges, non-manifold edges, flipped winding, self-intersections
- **Geometric algorithms** — connected components, minimum bounding sphere, barycentric embedding

## Package Structure

```
pypgo.mesh
├── __init__.py          # Namespace aggregation, OBJ I/O, shape factories
├── data.py              # Pure data containers (zero internal dependencies)
├── geo/                 # Geometry facades & algorithms
│   ├── __init__.py
│   ├── core.py          # C++ thin wrappers (TriMeshGeo, BarycentricEmbedding, SurfaceEmbedding)
│   └── algorithms.py    # Pure-Python geometry algorithms
├── volume/              # Volume mesh & materials
│   ├── __init__.py
│   ├── core.py          # VolumeMesh, VegFile, .veg I/O
│   └── material.py      # Material definitions (ENuMaterial, MooneyRivlin, Orthotropic)
└── processing/          # Mesh generation & surface repair
    ├── __init__.py
    ├── volume.py        # Volume meshing (cubic_mesher, tet_mesher)
    └── surface.py       # Surface repair & quality diagnostics
```

## Architecture

```
data.py          ← zero internal dependencies (only _core + _arrays)
    ↑
geo/core.py, geo/algorithms.py, volume/material.py, volume/core.py,
processing/volume.py, processing/surface.py, __init__.py
```

All submodules depend **one-way** on `data.py`. There are no circular imports — `geo/core.py` and `processing/volume.py` use lazy imports for cross-cutting dependencies on `volume`.

## Quick Reference

### "I need to load a surface mesh"

```python
from pypgo.mesh import read_obj, TriMeshData
mesh = read_obj("bunny.obj")
print(mesh.num_vertices, mesh.num_elements)
```

### "I need a volume mesh for simulation"

```python
from pypgo.mesh import read_obj, tet_mesher
from pypgo.mesh.volume import VolumeMesh, ENuMaterial

surface = read_obj("bunny.obj")
tet_data = tet_mesher(surface, backend="tetgen")
volume = VolumeMesh.create_from_single_material(tet_data, ENuMaterial(E=1e6))
```

### "I need to check surface mesh quality"

```python
from pypgo.mesh import read_obj, check_surface_quality
report = check_surface_quality(read_obj("bunny.obj"))
print(report.is_clean, report.degenerate_tris)
```

### "I need to remesh a surface"

```python
from pypgo.mesh import read_obj, cgal_isotropic_remesh
remeshed = cgal_isotropic_remesh(read_obj("bunny.obj"), target_edge_length=0.01)
```

## Exported API

All symbols are importable from `pypgo.mesh` directly:

```python
from pypgo.mesh import (
    # Data containers
    MeshDataType, TriMeshData, TetMeshData, CubicMeshData,
    # Geometry & embedding
    SurfaceEmbedding,
    # OBJ I/O
    read_obj, write_obj,
    # Shape factories
    create_box, create_sphere, create_cylinder, create_torus,
    # Quality & diagnostics
    VolumetricMeshInfo, QualityReport,
    MergeCloseVerticesResult, RawSurfaceCleanupStats,
    RawSurfaceCleanupReport, RawSurfaceCleanupResult,
    volume_mesh_info, check_surface_quality,
    # Meshing
    cubic_mesher, tet_mesher, has_tetwild, has_cgal_remesher,
    # Surface repair
    remove_isolated_vertices, merge_close_vertices, raw_surface_cleanup,
    cgal_smooth, cgal_isotropic_remesh, cgal_repair_self_intersections, cgal_simplify,
)
```

Submodules are also directly accessible:

```python
from pypgo.mesh.geo import TriMeshGeo, minimum_bounding_sphere
from pypgo.mesh.volume import VolumeMesh, ENuMaterial, read_veg
from pypgo.mesh.processing import check_surface_quality
```

## Further Reading

- [`data.md`](data.md) — MeshData containers (`TriMeshData`, `TetMeshData`, `CubicMeshData`)
- [`geo/overview.md`](geo/overview.md) — Geometry facades, embedding, and algorithms
- [`volume/overview.md`](volume/overview.md) — `VolumeMesh`, materials, and `.veg` I/O
- [`processing/overview.md`](processing/overview.md) — Meshing, surface repair, and quality diagnostics
