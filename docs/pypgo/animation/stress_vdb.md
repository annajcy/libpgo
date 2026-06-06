# `pypgo.animation.stress_vdb` — OpenVDB Stress Field Export

## Overview

Samples **per-tetrahedron von Mises stress** and writes it as a per-frame sequence of **OpenVDB (`.vdb`)** files. This is a key post-processing step in the libpgo simulation pipeline — stress data is stored in a sparse volumetric format that can be rendered directly in Houdini and other DCC tools.

## Architecture

```
pypgo.animation.stress_vdb
├── has_stress_vdb_export()          # Check whether OpenVDB support was compiled in
├── StressFieldVDBExporter           # Stress-to-VDB pipeline class
└── dump_stress_vdb()                # One-liner convenience (auto-detects frame range)
```

## Data Flow

```
.veg file (tet mesh rest configuration)
    +
states/deform{frame}.u (per-frame displacement)
    +
stress/von_mises{frame}.json (per-frame von Mises stress)
    │
    ▼
StressFieldVDBExporter
    │  splats per-tet stress into a voxel grid
    ▼
output/vonMises{frame}.vdb (OpenVDB sequence)
```

## API Reference

---

### `has_stress_vdb_export() -> bool`

Returns `True` if the current build was compiled with OpenVDB support (`PGO_ENABLE_OPENVDB=ON` at CMake time).

```python
from pypgo.animation import has_stress_vdb_export
if not has_stress_vdb_export():
    raise SystemExit("OpenVDB not available in this build")
```

---

### `class StressFieldVDBExporter`

Splats per-tet von Mises stress into a per-frame OpenVDB sequence.

**Expected file layout:**

```
{sim_output}/
├── states/
│   ├── deform_0000.u
│   ├── deform_0001.u
│   └── ...
└── stress/
    ├── von_mises_0000.json
    ├── von_mises_0001.json
    └── ...
```

**Methods:**

| Method | Description |
|--------|-------------|
| `load_tet_mesh(veg_path)` | Load the rest-configuration tet mesh from a `.veg` file |
| `load_deformation_sequence(folder, pattern, start, end)` | Load per-frame displacement `.u` file sequence |
| `load_von_mises_sequence(folder, pattern, start, end)` | Load per-frame von Mises stress JSON sequence |
| `export_animation_vdb(output_dir, prefix, voxel_size)` | Write the VDB sequence |
| `num_frames` (property) | Number of frames loaded |

**Usage:**

```python
from pypgo.animation import StressFieldVDBExporter

exporter = StressFieldVDBExporter()

# 1. Load tet mesh
exporter.load_tet_mesh("bunny.veg")

# 2. Load displacement sequence [0, 24)
exporter.load_deformation_sequence(
    "sim/states", "deform{:04d}.u", 0, 24
)

# 3. Load stress sequence [0, 24)
exporter.load_von_mises_sequence(
    "sim/stress", "von_mises{:04d}.json", 0, 24
)

# 4. Export VDB
exporter.export_animation_vdb("output/", prefix="vonMises", voxel_size=0.01)
# voxel_size <= 0  → auto-derived from tet mesh
```

---

### `dump_stress_vdb(veg_path, sim_output, output_dir, *, prefix, voxel_size, frame_start, frame_end) -> int`

**Convenience function** that wraps the full `StressFieldVDBExporter` workflow and auto-detects the frame range.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `veg_path` | `str \| Path` | — | `.veg` tet mesh file |
| `sim_output` | `str \| Path` | — | Simulation output root (must contain `states/` and `stress/` subdirectories) |
| `output_dir` | `str \| Path` | — | Destination for `.vdb` files |
| `prefix` | `str` | `"vonMises"` | Output filename prefix |
| `voxel_size` | `float` | `0.0` | VDB voxel size; `<= 0` auto-derives from the tet mesh |
| `frame_start` | `int` | `0` | First frame index (inclusive) |
| `frame_end` | `int` | `-1` | Last frame index (exclusive); `-1` auto-detects |

**Returns:** number of frames written.

```python
from pypgo.animation import dump_stress_vdb

n = dump_stress_vdb(
    "bunny.veg",
    "sim_output/",
    "output_vdb/",
    prefix="stress",
    voxel_size=0.005,
)
print(f"Wrote {n} frames")
```

**Frame range auto-detection:** When `frame_end < 0`, the function starts at `frame_start` and increments until neither `states/deform{frame:04d}.u` nor `stress/von_mises{frame:04d}.json` exists.

## CLI Entry Point

This module is exposed via the `pypgo-stress-vdb` console script:

```bash
pypgo-stress-vdb bunny.veg sim_output/ output_vdb/ \
    --prefix vonMises \
    --voxel-size 0.01 \
    --frame-start 0 \
    --frame-end 100
```

Source: `pypgo/tools/animation/stress_vdb.py`.

## Build Dependency

- `PGO_ENABLE_OPENVDB=ON` — requires CMake to locate the OpenVDB library.
- OpenVDB is **not** a default dependency of libpgo; it must be explicitly enabled.

If OpenVDB is unavailable at runtime, both `StressFieldVDBExporter()` and `dump_stress_vdb()` raise `RuntimeError`.

## Relationship with the `abc` Module

| Module | Output Format | Data Content | Target Tools |
|--------|--------------|--------------|--------------|
| `abc` | Alembic `.abc` | Mesh geometry (vertex displacement) | Maya, Blender, Houdini |
| `stress_vdb` | OpenVDB `.vdb` | Volumetric stress field (von Mises) | Houdini (volume rendering) |

The two modules are typically used together: `.abc` for geometric deformation visualization, `.vdb` for volumetric stress field rendering.
