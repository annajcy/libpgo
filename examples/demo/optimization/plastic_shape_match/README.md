# Plastic shape matching

This example optimizes a volumetric plastic field so that a static elastic body
matches a prescribed surface shape. The executable code is intentionally kept
in [`main.py`](./main.py); this document contains the modeling background and
workflow notes.

## Model

The body is a `4 x 4 x 4` cubic mesh using:

- `StVK` elasticity;
- the `CubicLinear` finite-element formulation;
- a symmetric plastic tensor with six parameters per field value; and
- fixed displacement on the `y = 0` face.

The deformation gradient is decomposed as

\[
F = F_e F_p,
\]

and static displacement is obtained from the inner equilibrium problem

\[
u^*(a) = \arg\min_u E(u, a),
\]

where `a` is the plastic parameter vector. The outer problem minimizes

\[
\frac{1}{2}\|x_s(u^*(a)) - x_{target}\|^2
+ \frac{\lambda}{2}\|a-a_0\|^2.
\]

`PlasticStaticEquilibriumLayer` exposes the equilibrium solve as a PyTorch
operation. Its backward pass differentiates through equilibrium with an
adjoint solve and the plastic Jacobian supplied by libpgo.

## Design variables

The example uses an elementwise field with six plastic parameters per element.
Keeping one design representation makes the script a direct demonstration of
`PlasticStaticEquilibriumLayer`: build the model, run Adam, and save the fitted
field.

## Run

From the repository root:

```bash
python examples/demo/optimization/plastic_shape_match/main.py
```

The example runs 60 Adam iterations and opens an interactive target-versus-fit
mesh view. It writes
`examples/demo/optimization/plastic_shape_match/output/plastic_shape_match_weights.npz`, which
contains the mesh, target, optimized parameters, optimized vertices, and
optimization history.

The example uses the true material Hessian (`enforce_spd=False`) so that the
adjoint gradient corresponds to the modeled equilibrium problem. For larger or
less well-conditioned cases, use a more robust inner solve and verify gradients
with finite differences before relying on the recovered field.
