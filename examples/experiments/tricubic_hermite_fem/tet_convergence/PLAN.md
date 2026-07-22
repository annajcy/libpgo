# Tet Spatial Convergence Test

## Decision to Enable

Determine whether `cubic_linear_x27`, the current `tet_ref`, or
`cubic_hermite` is closest to the static continuous-space solution on the
shared conservative R15 domain.  The test is deliberately static: dynamic
time-integration and contact errors are out of scope.

## Target Hypothesis

On the common original-surface displacement field,

```text
distance(cubic_linear_x27, u_infinity)
  < distance(current tet_ref, u_infinity).
```

The same interval comparison will rank `cubic_hermite` against x27 when the
remaining tet uncertainty is small enough.

## Strongest Competing Explanations

1. x27 overshoots the continuous solution and the current tet reference or
   Hermite is more accurate.
2. The apparent ordering is smaller than the unresolved tet tail and cannot
   be identified with the available meshes.
3. Solver tolerance, attachment residual, mesh quality, or vertex sampling
   creates the apparent ordering.

## Minimum Setup

The existing tet reference is `tet_L0`.  Two independently remeshed TetGen
levels target two and four times its actual DOF count.  A factor-four cubic
subdivision provides an independent low-order convergence check.

| case | target relative to current tet | role |
|---|---:|---|
| `tet_L0` | 1x | reproduce current numerical reference |
| `tet_L0_tight` | 1x | solver-tolerance negative control |
| `tet_L1` | 2x | first finer tet level |
| `tet_L2` | 4x | second finer tet level |
| `cubic_linear_x64` | n/a | independent cubic-linear convergence control |

All physical settings, input surface vertices, attachment vertices, material,
gravity, sparse-solver selection, and TBB concurrency match the canonical
static experiment.  `tet_L0_tight` changes only the gradient tolerance from
`1e-5` to `1e-6`.

Generated meshes, manifests, and results live under
`examples/experiments/tricubic_hermite_fem/output/<study>/tet_convergence`; the canonical
tet selection manifest is read but never modified.

## Primary Observable

The primary norm is the lumped-area-weighted L2 norm of displacement on the
non-attached vertices of the shared original surface.  The existing
vertex-weighted free-surface L2, y-only L2, point errors, displacement RMS,
and pin residual are secondary diagnostics.

For tet displacement fields `u0`, `u1`, and `u2`, define

```text
d10 = ||u1 - u0||
d21 = ||u2 - u1||
rho = d21 / d10
```

The sequence is eligible for extrapolation only when:

- `rho < 0.8`;
- the weighted cosine between `u1-u0` and `u2-u1` is at least `0.9`;
- `||u_L0_tight-u_L0|| <= 0.1*d21`;
- all solves converge and pass the existing pin-residual limit.

When eligible, the unresolved tail radius around L2 is estimated before
ranking candidates:

```text
delta = rho / (1-rho) * d21 + solver_noise.
```

Each candidate receives the conservative distance interval
`[max(0, ||u-u2||-delta), ||u-u2||+delta]`.

## Result Interpretation Fixed in Advance

- **Supports the hypothesis:** the x27 interval lies wholly below the current
  tet L0 interval.
- **Falsifies/materially weakens it:** the current tet L0 interval lies wholly
  below the x27 interval.
- **Inconclusive:** convergence eligibility fails or the intervals overlap.

Hermite versus x27 uses the same rule.  RMS alone never determines the result.

## Stop and Escalation Rules

- Stop after L2 when the primary intervals separate and secondary diagnostics
  do not show a material contradiction.
- Report inconclusive rather than extrapolate when the eligibility checks fail.
- Add an L3 tet level only when L2 ran successfully but the primary intervals
  still overlap or the tet tail is too large.
- Use a `nu=0.3` material ablation only to diagnose suspected near-incompressible
  P1 locking; it is not part of the primary claim.

## Residual Risks

The tet meshes are not nested, the tail bound assumes approximately geometric
asymptotic behavior, and soft surface attachments are part of the problem
definition.  The fixed original surface makes correspondence exact, while the
area-weighted metric reduces sensitivity to nonuniform surface sampling.
