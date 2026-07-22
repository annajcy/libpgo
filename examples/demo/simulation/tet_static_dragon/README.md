# Tetrahedral static dragon

This example is a lightweight Python-API adaptation of
`examples/sim_configs/tet_static_dragon.json`.

It uses the smaller `dragon.veg` instead of the config's `dragon_big.veg`, then
evaluates Stable Neo-Hookean elasticity with the
linear tetrahedral formulation, applies gravity, and softly pins selected
vertices on the embedded render surface. The attachment is expressed through
the surface interpolation matrix, so the constrained points belong to the OBJ
surface rather than the simulation mesh.

Run from the repository root:

```bash
python examples/demo/simulation/tet_static_dragon/main.py
```

The final surface is written to
`examples/demo/simulation/tet_static_dragon/output/final_surface.obj`.

The JSON scene remains the preferred entry point for CLI batch runs and output
configuration. This demo exists to show how the same scene is assembled from
the public `pypgo` APIs.
