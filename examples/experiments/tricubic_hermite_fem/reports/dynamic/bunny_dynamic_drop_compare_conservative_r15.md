# Bunny Dynamic Drop FEM Comparison on a Conservative R15 Domain

This report summarizes the complete server rerun of the dynamic bunny drop
experiment. It compares cubic-linear FEM and cubic-tricubic-Hermite FEM on a
conservative cubic domain, using a same-domain tet-linear solve as a numerical
reference.

The main question is:

> During a contact-rich dynamic drop, does tricubic Hermite produce a visible
> surface trajectory closer to a same-domain tet-linear numerical reference
> than cubic-linear FEM, including a refined cubic-linear run with comparable
> DOFs?

The answer is yes for this experiment. Before contact, all cubic formulations
remain close to the tet reference. After impact, the trajectories separate:
post-contact mean relative surface displacement error is `4.34%` for the
original cubic-linear mesh, `1.57%` for the 2x2x2-refined cubic-linear mesh,
and `0.21%` for tricubic Hermite. At comparable DOF scale, Hermite's
post-contact error is `7.33x` lower than the refined cubic-linear baseline,
while it uses `6.86x` as much wall time. Hermite still uses `14.44%` less wall
time than the tet reference (`12.28 h` versus `14.36 h`).

## Data Provenance and Validation

The report uses the complete rerun data in:

```text
examples/outputs/bunny-dynamic-drop-compare-conservative-r15
```

Run metadata comes from each case's current `summary.json`. All four summaries
describe fresh runs from timestep 0 through timestep 800; none was resumed.
Trajectory metrics in this report were recomputed directly from the 80 surface
dumps per case (`surface0000.obj` through `surface0790.obj`).

The existing `comparison.json` and `comparison.csv` predate the final copied
summaries and still contain stale resumed-run metadata for `cubic_linear`.
Their `cubic_linear` run metadata and aggregate values are therefore not used
here.

Validation checks:

- every case reached timestep `800` and accepted all `800` simulation steps;
- every case has `80` surface dumps at a 10-step interval;
- every dumped surface has `4214` vertices and `8424` faces;
- visible-surface face connectivity matches the original bunny surface across
  cases and checked frames;
- the four runs use the same dynamics and contact settings.

## Experimental Design

### Cases

| case | mesh | formulation | role |
|---|---|---|---|
| `tet_ref` | same-domain tetrahedral mesh | tet-linear | numerical reference |
| `cubic_linear` | conservative r15 cubic mesh | cubic-linear | low-order cubic baseline |
| `cubic_linear_x8` | same domain, each cube split 2x2x2 | cubic-linear | comparable-DOF low-order baseline |
| `cubic_hermite` | conservative r15 cubic mesh | cubic-tricubic-Hermite | high-order method under test |

The original cubic-linear and Hermite cases use the same cubic volume mesh:

```text
examples/experiments/tricubic_hermite_fem/assets/veg/cubic/bunny-conservative-r15.veg
```

The refined cubic-linear case uses:

```text
examples/experiments/tricubic_hermite_fem/assets/veg/cubic/bunny-conservative-r15-subdiv2.veg
```

The tet reference uses:

```text
examples/experiments/tricubic_hermite_fem/assets/veg/tet/bunny-conservative-r15-tet-a2.8768e-9.veg
```

### Geometry and Dynamic Setup

| asset | path |
|---|---|
| original surface | `examples/experiments/tricubic_hermite_fem/assets/obj/bunny.obj` |
| ground obstacle | `examples/experiments/tricubic_hermite_fem/assets/obj/bottom.1.obj` |
| cubic volume mesh | `examples/experiments/tricubic_hermite_fem/assets/veg/cubic/bunny-conservative-r15.veg` |
| refined cubic-linear mesh | `examples/experiments/tricubic_hermite_fem/assets/veg/cubic/bunny-conservative-r15-subdiv2.veg` |
| tet reference mesh | `examples/experiments/tricubic_hermite_fem/assets/veg/tet/bunny-conservative-r15-tet-a2.8768e-9.veg` |

All cases use the same original visible surface for output and metric
evaluation.

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
| maximum nonlinear iterations | `200` |
| gradient tolerance | `1e-4` |
| worker limit | `12` threads |

The conservative cubic domain has effectively the same volume ratio in every
case:

| case | volume ratio |
|---|---:|
| `tet_ref` | 1.479165781488413 |
| `cubic_linear` | 1.479165781488412 |
| `cubic_linear_x8` | 1.479165781486210 |
| `cubic_hermite` | 1.479165781488412 |

### DOF Scale and Run Completion

| case | DOFs | accepted steps | surface dumps |
|---|---:|---:|---:|
| `tet_ref` | 365,184 | 800/800 | 80 |
| `cubic_linear` | 9,102 | 800/800 | 80 |
| `cubic_linear_x8` | 62,355 | 800/800 | 80 |
| `cubic_hermite` | 72,816 | 800/800 | 80 |

The refined cubic-linear case has `85.63%` as many DOFs as Hermite
(`62,355 / 72,816`), so it provides a substantially fairer accuracy comparison
than the original 9,102-DOF cubic-linear mesh.

### Solver Status and Wall Time

| case | converged-status steps | max-iteration-status steps | mean iterations | wall time | vs. original linear |
|---|---:|---:|---:|---:|---:|
| `tet_ref` | 798 | 2 | 7.68 | 51,683.28 s (14.36 h) | 35.02x |
| `cubic_linear` | 800 | 0 | 5.88 | 1,475.92 s (24.60 min) | 1.00x |
| `cubic_linear_x8` | 798 | 2 | 8.66 | 6,445.53 s (1.79 h) | 4.37x |
| `cubic_hermite` | 798 | 2 | 8.25 | 44,218.23 s (12.28 h) | 29.96x |

All steps were accepted. The tet reference, refined cubic-linear, and Hermite
runs each contain two accepted steps whose recorded solver status is
`MAX_ITERATIONS` rather than `CONVERGED`. These are retained in the trajectory,
so the tet solve should be understood as a numerical baseline rather than
ground truth.

The wall times are the complete, accurate server-run measurements for this
experiment under the recorded 12-thread configuration. They are used as a
formal comparison metric below.

## Metric Definitions

Let:

- `S` be the set of visible surface vertices;
- `x_i^0` be the rest position of surface vertex `i`;
- `x_i^c(t)` be its dumped position for case `c` at frame `t`;
- `x_i^ref(t)` be its dumped position in `tet_ref`;
- `u_i^c(t) = x_i^c(t) - x_i^0`;
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

At the final dumped frame `t = 790`, the pointwise 95th-percentile error is:

```math
E_{95,c}^{final} =
\operatorname{P95}_{i \in S}
\|u_i^c(790) - u_i^{ref}(790)\|_2.
```

The impact frame is the first dumped frame satisfying:

```math
\min_{i \in S} x_{i,y}^c(t)
\le y_{obs}^{top} + d_{hat}^{external}.
```

For this experiment:

```text
y_obs_top = -0.762576
dhat_external = 0.005
impact threshold = -0.757576
```

Windowed metrics average `E_c(t)` over the following dumped frames:

| window | frame range | samples |
|---|---|---:|
| full trajectory | `10, 20, ..., 790` | 79 |
| pre-contact | `100, 110, ..., 390` | 30 |
| post-contact | `410, 420, ..., 790` | 39 |
| late | `600, 610, ..., 790` | 20 |

The post-contact window begins at frame `410`, when `tet_ref` first reaches the
contact threshold. Frame `0` is excluded from trajectory statistics because
the reference displacement norm is nearly zero. Even at frame `10` that norm
is still small, so early-frame relative values can be large; post-contact and
late-window metrics are the most informative results for this experiment.

## Results

### Relative Surface Displacement Error

| window | statistic | `cubic_linear` | `cubic_linear_x8` | `cubic_hermite` |
|---|---|---:|---:|---:|
| full trajectory | mean rel L2 | 0.033005 | 0.013654 | **0.005299** |
| full trajectory | p95 rel L2 | 0.104375 | 0.030218 | **0.011334** |
| pre-contact | mean rel L2 | 0.001559 | 0.000824 | **0.000616** |
| pre-contact | p95 rel L2 | 0.004406 | 0.002298 | **0.001752** |
| post-contact | mean rel L2 | 0.043374 | 0.015726 | **0.002145** |
| post-contact | p95 rel L2 | 0.104375 | 0.030218 | **0.007630** |
| late | mean rel L2 | 0.064488 | 0.022897 | **0.003370** |
| late | p95 rel L2 | 0.115819 | 0.032136 | **0.009294** |

In percentage form, the mean relative errors are:

| window | `cubic_linear` | `cubic_linear_x8` | `cubic_hermite` |
|---|---:|---:|---:|
| full trajectory | 3.30% | 1.37% | **0.53%** |
| pre-contact | 0.16% | 0.08% | **0.06%** |
| post-contact | 4.34% | 1.57% | **0.21%** |
| late | 6.45% | 2.29% | **0.34%** |

### Final Frame

| metric at frame 790 | `cubic_linear` | `cubic_linear_x8` | `cubic_hermite` |
|---|---:|---:|---:|
| relative surface L2 | 0.126176 | 0.034082 | **0.010981** |
| y-only relative L2 | 0.073542 | 0.012187 | **0.008422** |
| mean pointwise error | 0.092192 | 0.024895 | **0.007610** |
| p95 pointwise error | 0.133584 | 0.034561 | **0.014199** |
| maximum pointwise error | 0.148974 | 0.038564 | **0.016270** |

The absolute pointwise errors are expressed in the simulation's geometry
length units.

### Contact Timing

| case | impact frame | impact time | delta from tet reference |
|---|---:|---:|---:|
| `tet_ref` | 410 | 0.410 s | 0 steps |
| `cubic_linear` | 400 | 0.400 s | -10 steps |
| `cubic_linear_x8` | 410 | 0.410 s | 0 steps |
| `cubic_hermite` | 410 | 0.410 s | 0 steps |

The original cubic-linear case reaches the contact threshold one dump interval
earlier. The refined cubic-linear and Hermite cases match the tet reference at
the available 10-step temporal resolution.

### Error Ratios

| mean-error comparison | full | pre-contact | post-contact | late |
|---|---:|---:|---:|---:|
| original linear / x8 linear | 2.42x | 1.89x | 2.76x | 2.82x |
| x8 linear / Hermite | 2.58x | 1.34x | **7.33x** | **6.79x** |
| original linear / Hermite | 6.23x | 2.53x | **20.22x** | **19.13x** |

At the final frame, refined cubic-linear is `3.10x` higher in relative surface
L2 error than Hermite; original cubic-linear is `11.49x` higher than Hermite.

## Analysis

### Free Fall

All three cubic methods are close to the tet reference in the pre-contact
window. Refining cubic-linear reduces mean relative error from `0.156%` to
`0.082%`; Hermite reduces it further to `0.062%`. The differences are real but
small compared with those after impact.

### Contact Response

Contact is where the formulations separate. Refinement reduces cubic-linear's
post-contact mean error from `4.34%` to `1.57%`. At similar DOF scale, Hermite
reduces it again to `0.21%`, a `7.33x` reduction relative to refined
cubic-linear.

The same ordering persists in the late window: `6.45%` for original
cubic-linear, `2.29%` for refined cubic-linear, and `0.34%` for Hermite. Thus
the Hermite advantage is not explained solely by having more DOFs than the
original cubic-linear mesh.

### Final State

At frame `790`, relative surface L2 error is `12.62%`, `3.41%`, and `1.10%`
for original cubic-linear, refined cubic-linear, and Hermite respectively.
Hermite also has the lowest y-only and pointwise surface errors.

### Runtime Tradeoff

The measured wall-time ordering is:

```text
cubic_linear < cubic_linear_x8 < cubic_hermite < tet_ref
```

| comparison | wall-time change | post-contact error change | late error change |
|---|---:|---:|---:|
| x8 linear vs. original linear | 4.37x as long | 2.76x lower | 2.82x lower |
| Hermite vs. x8 linear | 6.86x as long | 7.33x lower | 6.79x lower |
| Hermite vs. original linear | 29.96x as long | 20.22x lower | 19.13x lower |
| Hermite vs. tet reference | 14.44% less | not applicable | not applicable |

The most relevant like-for-like tradeoff is Hermite versus refined
cubic-linear: their DOF counts are comparable, and Hermite takes `6.86x` as
long while reducing post-contact mean error by `7.33x`, late mean error by
`6.79x`, and final relative surface L2 error by `3.10x`.

The tet reference is the slowest case. Hermite finishes `7,465.05 s`
(`2.07 h`) sooner, a `14.44%` reduction in wall time. The reference is included
to define trajectory error, so its accuracy ratios are not meaningful.

## Limitations

1. `tet_ref` is a same-domain numerical reference, not ground truth.
2. Three cases contain two accepted steps marked `MAX_ITERATIONS`; those steps
   remain in the reported trajectories.
3. Contact timing is resolved only at dumped frames, every 10 simulation
   steps.
4. `cubic_linear_x8` is comparable in DOF count, not exactly equal to Hermite.
5. The experiment contains one geometry, material configuration, timestep,
   and contact setup; the conclusion should not be generalized without more
   cases.
6. Wall-time conclusions apply to this server and recorded 12-thread setup;
   cross-hardware portability was not tested.

## Conclusions

For this complete dynamic bunny drop rerun on the conservative r15 domain:

1. Original cubic-linear is least accurate after contact and reaches the
   dumped impact threshold 10 simulation steps earlier than the tet reference.
2. 2x2x2 refinement makes cubic-linear substantially more accurate and aligns
   its dumped impact frame with the tet reference.
3. Tricubic Hermite remains more accurate than the comparable-DOF refined
   cubic-linear baseline: post-contact and late mean relative errors are
   `7.33x` and `6.79x` lower, respectively.
4. At frame `790`, Hermite's relative surface L2 error is `1.10%`, compared
   with `3.41%` for refined cubic-linear and `12.62%` for original
   cubic-linear.
5. The runtime cost of Hermite versus comparable-DOF refined cubic-linear is
   `6.86x`; in return, post-contact and late mean errors are `7.33x` and
   `6.79x` lower. Hermite also uses `14.44%` less wall time than the tet
   reference.

## Reproducibility Pointers

Runner:

```text
examples/experiments/tricubic_hermite_fem/run_dynamic.py
```

Per-case run summaries:

```text
examples/outputs/bunny-dynamic-drop-compare-conservative-r15/<case>/summary.json
```

Surface dumps used for the metrics:

```text
examples/outputs/bunny-dynamic-drop-compare-conservative-r15/<case>/surface/surface0000.obj
...
examples/outputs/bunny-dynamic-drop-compare-conservative-r15/<case>/surface/surface0790.obj
```
