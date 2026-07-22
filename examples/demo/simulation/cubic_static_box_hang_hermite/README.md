# Tricubic Hermite box hang

This example is the direct Python-API version of
`examples/sim_configs/cubic_static_box_hang_hermite.json`.

The box uses `CubicTricubicHermite`, which has 24 generalized DOFs per volume
vertex: three displacement values and their derivative data. The script clamps
all DOFs on the top face, applies gravity, solves static equilibrium, and maps
the high-order displacement back to the OBJ render surface.

Run from the repository root:

```bash
python examples/demo/simulation/cubic_static_box_hang_hermite/main.py
```

The final surface is written to
`examples/demo/simulation/cubic_static_box_hang_hermite/output/final_surface.obj`.

The static scene keeps the Hermite example focused and avoids the much higher
cost of combining the formulation with a long IPC time sequence.
