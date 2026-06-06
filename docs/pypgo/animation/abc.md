# `pypgo.animation.abc` — Alembic Export & Eigen Binary Matrix I/O

## Overview

This module provides **Alembic (`.abc`) animation export** and **Eigen binary matrix (`.u`) read/write** functionality. It is the core pipeline for visualizing libpgo simulation results — per-frame displacement data is packaged into industry-standard Alembic format for viewing in DCC tools such as Maya, Houdini, or Blender.

## Architecture

```
pypgo.animation.abc
├── has_animation_io()              # Check whether Alembic support was compiled in
├── read_u_file / write_u_file      # Eigen binary matrix I/O (per-frame displacement data)
├── AbcWriter                       # Incremental Alembic writer
├── AnimationLoader                 # JSON-config-driven batch pipeline
└── dump_animation()                # One-liner: config → .abc
```

## API Reference

---

### `has_animation_io() -> bool`

Returns `True` if the current build was compiled with Alembic I/O support (`PGO_ENABLE_ANIM_IO=ON` at CMake time).

```python
from pypgo.animation import has_animation_io
if not has_animation_io():
    print("Alembic I/O not available")
```

---

### `read_u_file(path) -> np.ndarray`

Read an Eigen binary matrix (`.u` file). Returns a `(nrows, ncols)` `float64` array in column-major order, matching Eigen's default storage convention.

**File format:**
- 12-byte header: `(nrows, ncols, entry_size)` — three 4-byte integers
- Body: `nrows × ncols` floats (`entry_size` = 4 for float32, 8 for float64)

```python
from pypgo.animation import read_u_file
disp = read_u_file("path/to/deform_0000.u")  # shape: (nverts*3, 1)
```

---

### `write_u_file(path, mat)`

Write a 2-D array as an Eigen binary `.u` file. Data is automatically converted to `float64` column-major before writing.

```python
from pypgo.animation import write_u_file
import numpy as np
write_u_file("deform_0000.u", np.random.rand(300, 1))
```

---

### `class AbcWriter`

**Incremental Alembic writer.** Accumulates displacement frames in memory and writes the archive in one shot. Supports the context-manager protocol — the file is written automatically on exit.

| Parameter | Type | Description |
|-----------|------|-------------|
| `path` | `str \| Path` | Output `.abc` file path |
| `name` | `str` | Alembic object name |
| `rest_positions` | `np.ndarray` | Flat `(3*n_verts,)` float64 rest positions |
| `triangles` | `np.ndarray` | Flat `(3*n_tris,)` int32 face indices |
| `fps` | `float` | Frame rate written into the archive (default 24) |

**Methods:**

| Method | Description |
|--------|-------------|
| `add_frame(displacement)` | Append one frame of displacement data (must match `rest_positions` shape) |
| `write()` | Flush all accumulated frames to the `.abc` file |
| `dump(path, name, ...)` | **Class method** — write all frames at once |

**Usage:**

```python
from pypgo.animation import AbcWriter

# Incremental (compute displacements frame-by-frame)
with AbcWriter("out.abc", "bunny",
               rest_positions=rest, triangles=tris) as w:
    for disp in displacement_frames:
        w.add_frame(disp)
# write() called automatically on context exit

# One-shot (all frames known upfront)
AbcWriter.dump("out.abc", "bunny",
               rest_positions=rest, triangles=tris,
               displacements=list_of_disps)
```

If no frames are added, `write()` is **not** called on context exit.

---

### `class AnimationLoader`

**JSON-config-driven batch animation pipeline.** Parses a JSON configuration file, loads per-frame displacement sequences for multiple meshes, and writes corresponding `.abc` files.

**JSON config format:**

```json
{
    "meshes": [
        {
            "name": "bunny",
            "driving-mesh": "path/to/rest.obj",
            "sequence": "path/to/frame_{:04d}.u",
            "sequence-type": "u",
            "sequence-range": [0, 100]
        }
    ]
}
```

| Field | Description |
|-------|-------------|
| `name` | Alembic object name |
| `driving-mesh` | Rest-configuration OBJ file |
| `sequence` | printf-style path pattern for per-frame displacement files |
| `sequence-type` | Sequence format (currently only `"u"`) |
| `sequence-range` | `[start, end)` (end is exclusive) |
| `output-folder` | Optional; defaults to the config file's parent directory |

Relative paths are resolved against the config file's parent directory by the C++ backend.

```python
from pypgo.animation import AnimationLoader

loader = AnimationLoader()
loader.load("anim_config.json")
loader.save_abc("output/")
```

**AbcWriter vs AnimationLoader:** `AnimationLoader` is a higher-level C++ wrapper for "batch-convert existing displacement sequences." `AbcWriter` is a lower-level Python interface for "generating displacements in a simulation loop."

---

### `dump_animation(config_path, output_folder=None)`

One-liner that does `AnimationLoader.load → save_abc`.

- When `output_folder` is `None`, it takes the `output-folder` field from the config, falling back to the config file's parent directory.

```python
from pypgo.animation import dump_animation
dump_animation("config.json")            # output next to config file
dump_animation("config.json", "out/")    # explicit output directory
```

## CLI Entry Point

This module is exposed via the `pypgo-animation-convert` console script:

```bash
pypgo-animation-convert config.json -o output/
```

Source: `pypgo/tools/animation/abc_convert.py`.

## Data Flow

```
.u displacement sequence ──→ AnimationLoader.load() ──→ save_abc() ──→ .abc file
                                ↑ JSON config

Per-frame displacements ──→ AbcWriter.add_frame() ──→ write() ──→ .abc file
```

## Dependencies

- **Runtime:** `numpy` (for `read_u_file` / `write_u_file` only)
- **Build-time:** `PGO_ENABLE_ANIM_IO=ON` (Alembic SDK)
