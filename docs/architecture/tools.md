# Command-Line Tools

libpgo ships several standalone tools under `src/tools/`.

## runSim

Reads a simulation JSON config and runs a physics simulation.

```bash
cmake --build build --target runSim
build/bin/runSim examples/box/box.json
```

Supports `--deterministic` for single-threaded, reproducible execution.

## cubicMesher

Generates cubic/hexahedral volumetric meshes (`.veg`) from a uniform grid, with optional surface mesh export (`.obj`).

```bash
cmake --build build --target cubicMesher
build/bin/cubicMesher uniform --help
```

Example — generate a 4x4x4 box:

```bash
build/bin/cubicMesher uniform \
  --resolution 4 \
  --output-mesh examples/cubic-box/cubic-box.veg \
  --output-surface examples/cubic-box/cubic-box.obj \
  --E 1e6 --nu 0.45 --density 1000
```

## tetMesher

Tetrahedral mesh generator.

Source: `src/tools/tetMesher/`

## remeshSurface

Surface remeshing tool.

Source: `src/tools/remeshSurface/`

## animation

Animation export utilities for producing Alembic (`.abc`) files from simulation output.

Source: `src/tools/animation/`
