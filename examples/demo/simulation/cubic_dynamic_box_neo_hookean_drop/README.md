# Cubic box drop with Neo-Hookean materials

This example drops a cubic FEM box onto a penalty floor under gravity. It can
run either the classical logarithmic compressible Neo-Hookean material or the
existing stable Neo-Hookean material while keeping the mesh, physical
parameters, time integration, contact, and solver settings identical.

The classical material uses

\[
\psi(F)
=
\frac{\mu}{2}\left(\lVert F\rVert_F^2-3\right)
-\mu\log J
+\frac{\lambda}{2}(\log J)^2,
\qquad J=\det F>0.
\]

Both material choices use

```text
E       = 2.0e5
nu      = 0.35
density = 1000
dt      = 0.002
steps   = 200
```

## Run

Run from the repository root.

Classical Neo-Hookean:

```bash
python examples/demo/simulation/cubic_dynamic_box_neo_hookean_drop/main.py \
  --material neo_hookean \
  --output-dir /tmp/libpgo-box-drop/neo_hookean \
  --no-visualize
```

Stable Neo-Hookean:

```bash
python examples/demo/simulation/cubic_dynamic_box_neo_hookean_drop/main.py \
  --material stable_neo \
  --output-dir /tmp/libpgo-box-drop/stable_neo \
  --no-visualize
```

Use separate output directories when comparing the two materials. Without
`--output-dir`, both runs write to this example's `output/` directory and the
second run overwrites `animation.abc`.

Useful short-run controls:

```bash
python examples/demo/simulation/cubic_dynamic_box_neo_hookean_drop/main.py \
  --material neo_hookean \
  --num-steps 20 \
  --dump-interval 5 \
  --no-visualize
```

## Output

Each run writes:

- `animation.abc`: the complete surface animation, with one sample per
  simulation step plus the rest frame;
- `surface0000.obj`, `surface0020.obj`, ...: surface snapshots at the requested
  dump interval.

Since `dt = 0.002`, the Alembic time sampling is 500 fps and the default
200-step animation has 201 samples over 0.4 seconds. Use `--no-abc` to disable
Alembic output. If this build has no Alembic support, the simulation continues
with OBJ output and prints a warning.

