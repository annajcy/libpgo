# `pypgo.animation` — Animation I/O Overview

## Purpose

The `pypgo.animation` package handles **exporting simulation results into standard 3D interchange formats**. It converts libpgo's internal per-frame data (vertex displacements, per-element stress) into files that can be loaded directly into DCC tools (Maya, Houdini, Blender) for visualization, rendering, and further processing.

## Package Structure

```
pypgo.animation
├── __init__.py          # Re-exports public API from submodules
├── abc.py               # Alembic (.abc) export + Eigen binary (.u) I/O
└── stress_vdb.py        # OpenVDB (.vdb) stress field export
```

| Module | Output Format | Data Carried | Target Use |
|--------|--------------|-------------|------------|
| `abc` | Alembic `.abc` | Deformed mesh geometry | Surface deformation visualization |
| `stress_vdb` | OpenVDB `.vdb` | Per-element von Mises stress | Volumetric stress rendering |

## Exported API

All symbols are re-exported from `pypgo.animation`:

```python
from pypgo.animation import (
    # abc
    AbcWriter,
    AnimationLoader,
    dump_animation,
    has_animation_io,
    read_u_file,
    write_u_file,
    # stress_vdb
    StressFieldVDBExporter,
    dump_stress_vdb,
    has_stress_vdb_export,
)
```

## Typical Pipeline

```
Simulation output
    │
    ├── states/deform{frame}.u       ──→ AbcWriter / AnimationLoader ──→ output.abc
    │                                      (mesh geometry)
    │
    └── stress/von_mises{frame}.json ──→ StressFieldVDBExporter      ──→ output_{frame}.vdb
                                           (volumetric stress)
```

The two pipelines are independent — you can export geometry without stress, stress without geometry, or both together.

## Quick Reference

### "I have displacements in Python and want a `.abc` file"

Use `AbcWriter`:

```python
from pypgo.animation import AbcWriter

with AbcWriter("out.abc", "my_mesh", rest_positions=rest, triangles=tris) as w:
    for disp in displacements:
        w.add_frame(disp)
```

### "I have `.u` files on disk and a JSON config"

Use `dump_animation`:

```python
from pypgo.animation import dump_animation
dump_animation("config.json", "output/")
```

### "I want to export stress to `.vdb`"

Use `dump_stress_vdb`:

```python
from pypgo.animation import dump_stress_vdb
dump_stress_vdb("mesh.veg", "sim_output/", "output_vdb/")
```

## Build-time Features

Both modules require optional CMake flags to be enabled at build time:

| Feature | CMake Flag | Check Function |
|---------|-----------|----------------|
| Alembic I/O | `PGO_ENABLE_ANIM_IO=ON` | `has_animation_io()` |
| OpenVDB export | `PGO_ENABLE_OPENVDB=ON` | `has_stress_vdb_export()` |

If a feature was not compiled in, the corresponding constructor/function raises `RuntimeError`.

## CLI Tools

The package provides two console entry points:

```bash
# Convert .u sequences to .abc via JSON config
pypgo-animation-convert config.json -o output/

# Splat per-tet stress into OpenVDB sequence
pypgo-stress-vdb mesh.veg sim_output/ output_vdb/
```

## Dependencies

| Module | Python Deps | Native Deps |
|--------|------------|-------------|
| `abc` | `numpy` | Alembic SDK |
| `stress_vdb` | none (stdlib only) | OpenVDB |

## Further Reading

- [`abc.md`](abc.md) — Full API reference for `AbcWriter`, `AnimationLoader`, `.u` file I/O
- [`stress_vdb.md`](stress_vdb.md) — Full API reference for `StressFieldVDBExporter` and `dump_stress_vdb`
