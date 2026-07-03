# Bunny Dynamic Drop FEM Comparison on a Conservative R15 Domain

This report summarizes a dynamic bunny drop experiment comparing cubic-linear
FEM and cubic-tricubic-Hermite FEM on a conservative cubic domain. A
same-domain tet-linear solve is used as a numerical reference.

The main question is:

> During a contact-rich dynamic drop, does tricubic Hermite produce a visible
> surface trajectory closer to a same-domain tet-linear numerical reference
> than cubic-linear FEM, including a refined cubic-linear run with comparable
> DOFs?

The short answer is yes for this test. Before contact, all cubic formulations
are close to the tet reference. After impact with the ground obstacle, the
trajectories separate: post-contact mean relative surface trajectory error is
`4.25%` for the original cubic-linear mesh, `1.42%` for the 2x2x2-refined
cubic-linear mesh, and `0.35%` for tricubic Hermite.

## Experimental Design

### Cases

| case | mesh | formulation | role |
|---|---|---|---|
| `tet_ref` | same-domain tetrahedral mesh | tet-linear | numerical reference |
| `cubic_linear` | conservative r15 cubic mesh | cubic-linear | low-order cubic baseline |
| `cubic_linear_x8` | same conservative r15 domain, each cube split 2x2x2 | cubic-linear | comparable-DOF low-order baseline |
| `cubic_hermite` | conservative r15 cubic mesh | cubic-tricubic-Hermite | high-order method under test |

The original cubic-linear and Hermite cases use the same cubic volume mesh:

```text
examples/scripts/tricubic-hermit-dynamic-compare/assets/veg/cubic/bunny-conservative-r15.veg
```

The refined cubic-linear case uses:

```text
examples/scripts/tricubic-hermit-dynamic-compare/assets/veg/cubic/bunny-conservative-r15-subdiv2.veg
```

The tet reference uses:

```text
examples/scripts/tricubic-hermit-dynamic-compare/assets/veg/tet/bunny-conservative-r15-tet-a2.89036e-9.veg
```

### Geometry and Dynamic Setup

| asset | path |
|---|---|
| original surface | `examples/scripts/tricubic-hermit-dynamic-compare/assets/obj/bunny.obj` |
| ground obstacle | `examples/scripts/tricubic-hermit-dynamic-compare/assets/obj/bottom.1.obj` |
| cubic volume mesh | `examples/scripts/tricubic-hermit-dynamic-compare/assets/veg/cubic/bunny-conservative-r15.veg` |
| refined cubic-linear mesh | `examples/scripts/tricubic-hermit-dynamic-compare/assets/veg/cubic/bunny-conservative-r15-subdiv2.veg` |
| tet baseline mesh | `examples/scripts/tricubic-hermit-dynamic-compare/assets/veg/tet/bunny-conservative-r15-tet-a2.89036e-9.veg` |

All cases use the same original visible surface for output and metric
evaluation. Each dumped surface OBJ has `4214` vertices and `8424` faces with
matching topology across cases.

| setting | value |
|---|---:|
| elastic material | Stable Neo-Hookean |
| gravity | `(0, -9.81, 0)` |
| initial velocity | `(0, 0, 0)` |
| timestep | `0.001` |
| simulated steps | `800` |
| output dump interval | `10` |
| integrator | implicit Euler |
| damping | `(0, 0)` |
| IPC `dhat` | `0.002` |
| IPC external `dhat` | `0.005` |
| IPC `kappa` | `3000` |

The conservative cubic domain has the same volume ratio in all cubic cases:

| case | volume ratio |
|---|---:|
| `tet_ref` | `1.479165781488413` |
| `cubic_linear` | `1.479165781488412` |
| `cubic_linear_x8` | `1.479165781486210` |
| `cubic_hermite` | `1.479165781488412` |

### DOF Scale

| case | DOFs | dumped surface frames |
|---|---:|---:|
| `tet_ref` | 363,714 | 80 |
| `cubic_linear` | 9,102 | 80 |
| `cubic_linear_x8` | 62,355 | 80 |
| `cubic_hermite` | 72,816 | 80 |

The `cubic_linear_x8` case is included to test whether Hermite's improvement is
only a DOF-count effect. Its DOF count is close to the Hermite case:

```text
cubic_linear_x8 / cubic_hermite = 62,355 / 72,816 = 0.86x
```

## Metric Definitions

Let:

- `S` be the set of visible surface vertices.
- `x_i^0` be the rest position of surface vertex `i`.
- `x_i^c(t)` be the dumped position of vertex `i` for case `c` at dumped frame
  `t`.
- `x_i^ref(t)` be the dumped position of vertex `i` in `tet_ref`.
- `u_i^c(t) = x_i^c(t) - x_i^0`.
- `u_i^ref(t) = x_i^ref(t) - x_i^0`.

For a dumped frame `t`, the relative surface displacement error is:

```math
E_c(t) =
\frac{
  \sqrt{\sum_{i \in S} \|u_i^c(t) - u_i^{ref}(t)\|_2^2}
}{
  \sqrt{\sum_{i \in S} \|u_i^{ref}(t)\|_2^2}
}.
```

The y-only relative error is:

```math
E_{c,y}(t) =
\frac{
  \sqrt{\sum_{i \in S} (u_{i,y}^c(t) - u_{i,y}^{ref}(t))^2}
}{
  \sqrt{\sum_{i \in S} (u_{i,y}^{ref}(t))^2}
}.
```

The final pointwise 95th-percentile error is computed at final dumped frame
`t = 790`:

```math
E_{95,c}^{final} =
\operatorname{P95}_{i \in S}
\|u_i^c(790) - u_i^{ref}(790)\|_2.
```

The impact frame is the first dumped frame satisfying:

```math
\min_{i \in S} x_{i,y}^c(t) \le y_{obs}^{top} + d_{hat}^{external}.
```

For this experiment:

```text
y_obs_top = -0.762576
dhat_external = 0.005
impact threshold = -0.757576
```

Windowed metrics are averages of `E_c(t)` over selected dumped frames:

| metric | frame window |
|---|---|
| `full trajectory mean rel L2` | `10, 20, ..., 790` |
| `fall_pre_contact mean rel L2` | `100, 110, ..., 390` |
| `post_contact mean rel L2` | `410, 420, ..., 790` |
| `late mean rel L2` | `600, 610, ..., 790` |
| `final rel L2` | frame `790` only |

The `post_contact` window starts at frame `410` because `tet_ref` first reaches
the contact threshold at frame `410`. The `late` window is a diagnostic window
for accumulated post-impact trajectory differences. The frame `0` relative
error is not used as a headline metric because the reference displacement norm
is very small at the start of the simulation, making relative errors unstable.

## Results

### Trajectory Accuracy Against Tet Reference

The full trajectory window averages `E_c(t)` over all dumped frames except
frame `0`: `10, 20, ..., 790`. This provides a broad trajectory summary while
avoiding the unstable relative error caused by the near-zero initial reference
displacement.

| window | statistic | `cubic_linear` | `cubic_linear_x8` | `cubic_hermite` |
|---|---|---:|---:|---:|
| full trajectory | mean rel L2 | 0.033027 | 0.013403 | 0.006327 |
| full trajectory | p95 rel L2 | 0.103229 | 0.029229 | 0.012940 |
| pre-contact | mean rel L2 | 0.001863 | 0.001314 | 0.001175 |
| pre-contact | p95 rel L2 | 0.004613 | 0.002783 | 0.002333 |
| post-contact | mean rel L2 | 0.042519 | 0.014161 | 0.003456 |
| post-contact | p95 rel L2 | 0.103229 | 0.027403 | 0.009296 |
| late | mean rel L2 | 0.063331 | 0.020601 | 0.005298 |
| late | p95 rel L2 | 0.114724 | 0.029174 | 0.010991 |

### Contact Timing

| case | impact frame |
|---|---:|
| `tet_ref` | 410 |
| `cubic_linear` | 400 |
| `cubic_linear_x8` | 410 |
| `cubic_hermite` | 410 |

### Free-Fall and Contact Windows

Before contact, all three cubic cases are close to the tet reference. The
differences become much clearer after impact.

| case | pre-contact mean rel L2 | post-contact mean rel L2 | post/pre ratio |
|---|---:|---:|---:|
| `cubic_linear` | 0.001863 | 0.042519 | 22.8x |
| `cubic_linear_x8` | 0.001314 | 0.014161 | 10.8x |
| `cubic_hermite` | 0.001175 | 0.003456 | 2.9x |

### Late-Window Error Reduction

| comparison | value |
|---|---:|
| `cubic_linear` late error / `cubic_linear_x8` late error | 3.1x |
| `cubic_linear_x8` late error / `cubic_hermite` late error | 3.9x |
| `cubic_linear` late error / `cubic_hermite` late error | 12.0x |

## Analysis

### Accuracy

The refined cubic-linear case removes much of the original low-order error:
post-contact mean relative L2 drops from `4.25%` to `1.42%`, and late-window
mean relative L2 drops from `6.33%` to `2.06%`.

Hermite still improves substantially over the comparable-DOF low-order
baseline. Against `cubic_linear_x8`, Hermite reduces:

- post-contact mean relative L2 by about `4.1x`;
- late-window mean relative L2 by about `3.9x`;

### Contact Timing

The tet reference reaches the contact threshold at dumped frame `410`.
`cubic_linear_x8` and `cubic_hermite` match that dumped impact frame. The
original `cubic_linear` case reaches the threshold at frame `400`, one dump
interval earlier.

### Interpretation

This experiment is mainly a contact-response test. In the pre-contact free-fall
window, all cubic methods are close to the tet reference. After contact, the
low-order method diverges more strongly. Refinement helps, but the Hermite
surface trajectory remains closest to the tet reference at comparable DOF
scale.

## Conclusions

For the dynamic bunny drop case on the conservative r15 cubic domain:

1. The original cubic-linear method is the least accurate after contact.
2. The 2x2x2 refined cubic-linear baseline is much stronger and is necessary
   for a fairer comparison.
3. Tricubic Hermite is still more accurate than the comparable-DOF refined
   cubic-linear baseline.
4. The Hermite advantage is most visible in post-contact and late-time
   trajectory error.
5. The tet reference is a numerical baseline on the same conservative domain,
   not ground truth.

## Reproducibility Pointers

Primary output directory:

```text
examples/outputs/bunny-dynamic-drop-compare-conservative-r15
```

Runner:

```text
examples/scripts/tricubic-hermit-dynamic-compare/dynamic_compare.py
```

Surface dumps used for the metrics:

```text
examples/outputs/bunny-dynamic-drop-compare-conservative-r15/<case>/surface/surface0000.obj
...
examples/outputs/bunny-dynamic-drop-compare-conservative-r15/<case>/surface/surface0790.obj
```
