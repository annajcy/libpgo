# Cubic box drop with Systematic Poking material

This example drops a cubic FEM box onto a penalty floor using the
principal-stretch material from *Systematic Poking of 3D Objects*
(SIGGRAPH Asia 2023):

\[
\psi(s_1,s_2,s_3)
=
\sum_{i=1}^{3} f(s_i)+h(J),
\qquad J=s_1s_2s_3.
\]

Both \(f\) and the fixed volumetric shape are represented by integrated
linear-curvature splines. The stretch spline has nine log-uniform knots over
\([0.5,2]\), and the volume spline has nine knots over
\([\exp(-1),\exp(1)]\). Both knot arrays contain the rest coordinate \(1\).

For this comparison demo, the optimizable parameters are initialized from the
classical compressible Neo-Hookean material with

```text
E       = 2.0e5
nu      = 0.35
density = 1000
dt      = 0.002
steps   = 200
```

Specifically,

\[
f''(s_k)=\mu\left(1+\frac{1}{s_k^2}\right),
\]

and the final material parameter is the Lamé parameter \(\lambda\). The
volumetric spline shape samples the curvature of
\(\tfrac12(\log J)^2\) and is scaled by \(\lambda\).

## Run

Run from the repository root:

```bash
python examples/demo/simulation/cubic_dynamic_box_systematic_poking_drop/main.py \
  --output-dir /tmp/libpgo-box-drop/systematic_poking \
  --no-visualize
```

For a short diagnostic run:

```bash
python examples/demo/simulation/cubic_dynamic_box_systematic_poking_drop/main.py \
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

To compare against classical or stable Neo-Hookean under the same box-drop
configuration, use
`examples/demo/simulation/cubic_dynamic_box_neo_hookean_drop/main.py`.

