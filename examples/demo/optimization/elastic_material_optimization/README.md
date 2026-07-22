# Elastic material optimization

This example recovers a shell's spatially varying membrane stiffness from a
target equilibrium shape. The executable workflow lives in
[`main.py`](./main.py); the modeling and interpretation notes live here.

## Model

The shell is a triangulated `5 x 5` grid using the Koiter StVK model. Its elastic
field has five channels per triangle:

1. membrane Young's modulus;
2. membrane Poisson ratio;
3. bending Young's modulus;
4. bending Poisson ratio; and
5. thickness.

Only the membrane Young's modulus is optimized. The remaining channels and the
plastic field stay fixed. A small neural network maps each triangle center
`(x, y)` to its modulus, providing a smooth, low-dimensional material field.

The inner problem is static equilibrium under self-weight:

\[
u^*(b) = \arg\min_u \left(E(u,b) - f_g(b)^T u\right).
\]

`ElasticStaticEquilibriumLayer` differentiates the observed surface through
this solve. Because gravity depends on the elastic thickness channel, the
external load is passed separately to the layer so its parameter Jacobian is
included in the adjoint derivative. The load is added exactly once to the
forward objective.

The outer objective combines surface mismatch and network weight decay:

\[
\frac{1}{2}\|x_s(u^*(b)) - x_{target}\|^2 + R(\theta).
\]

## Synthetic target

The target is generated from a hidden modulus field that becomes softer away
from the clamped edge. A small sinusoidal out-of-plane perturbation is added so
the inverse problem is not merely replaying the target-generation solve. The
reported correlation compares the hidden and recovered membrane moduli.

This is an inverse-design demonstration, not a uniqueness claim: a single
equilibrium shape generally cannot identify every elastic channel. Restricting
the design to one smooth modulus field is the prior that makes this example
informative.

## Run

From the repository root:

```bash
python examples/demo/optimization/elastic_material_optimization/main.py
```

The example runs 300 Adam iterations and opens an interactive target-versus-fit
mesh view. It writes
`examples/demo/optimization/elastic_material_optimization/output/elastic_shape_match_weights.npz`
with the mesh, hidden and recovered fields, target and optimized vertices,
gravity force, and optimization history.

For production use, add finite-difference gradient checks, multiple load cases,
parameter bounds, and validation data before interpreting the recovered field
as a physical material estimate.
