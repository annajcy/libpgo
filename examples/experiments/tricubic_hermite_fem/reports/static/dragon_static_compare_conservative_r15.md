# Dragon Static FEM Comparison with a Tet L2 Numerical Reference

This report compares cubic-linear and cubic-tricubic-Hermite FEM on the same
conservative R15 dragon domain under static gravity and a fixed surface patch.
The finest available same-domain tet-linear discretization, `tet_L2`, is the
designated numerical reference for all reference-relative accuracy metrics.

The main question is:

> Relative to the designated tet L2 numerical reference, how do tricubic
> Hermite and successive cubic-linear refinements compare in visible-surface
> displacement accuracy and global DOF count?

At comparable global DOF count, Hermite remains more accurate than
`cubic_linear_x8`: their area-weighted surface discrepancies are `2.89%` and
`5.16%`, respectively. The more highly refined `cubic_linear_x27` case is
closer to tet L2 than Hermite (`1.98%` versus `2.89%`) while using `2.63x` as
many DOFs. The new `cubic_linear_x64` case is closest among the cubic methods
at `0.69%`, below tet L1's `1.29%` discrepancy while using fewer DOFs.

Tet L2 is the reporting reference, not a mathematically certified continuous
solution. The earlier L0 tet reference is retained below as `tet_L0` so that
the effect of reference refinement remains explicit.

## Run Provenance

| item | canonical formulation matrix | tet-convergence matrix |
|---|---|---|
| run date | 2026-07-22 | 2026-07-22 to 2026-07-23 |
| code commit | `916be0e8ae37e88f62c9532392fe85947a89eff4` | same |
| platform | Linux server | same |
| TBB concurrency limit | 12 | 21 |
| MKL threading layer | TBB | TBB |
| execution | fresh sequential `--force` run | fresh sequential tmux run |
| output | `examples/outputs/tricubic_hermite_fem/dragon/static/` | `examples/outputs/tricubic_hermite_fem/dragon/tet_convergence/` |

The canonical matrix supplies `cubic_linear`, `cubic_linear_x8`,
`cubic_hermite`, and `cubic_linear_x27`. The convergence matrix supplies
`tet_L0`, `tet_L0_tight`, `tet_L1`, `tet_L2`, and `cubic_linear_x64`.
Reference-relative displacement comparisons remain valid because the geometry,
physics, evaluation surface, code commit, and stopping rule are controlled.
Wall times must not be compared across the 12-thread and 21-thread matrices.

The decision to designate L2 as the reporting reference was made after the tet
convergence run. Results against L2 are therefore descriptive numerical
comparisons rather than a confirmatory error certification.

## Experimental Design

### Cases

| case | mesh/formulation | DOFs | role |
|---|---|---:|---|
| `tet_ref` (`tet_L2`) | independently remeshed tet-linear L2 | 2,522,595 | designated numerical reference |
| `tet_L1` | independently remeshed tet-linear L1 | 1,236,597 | tet convergence point |
| `tet_L0` | original canonical tet-linear reference mesh | 631,650 | earlier numerical reference |
| `tet_L0_tight` | same L0 mesh, `1e-6` gradient tolerance | 631,650 | solver-tolerance control |
| `cubic_linear` | original R15 cubic mesh, cubic-linear | 15,768 | low-order baseline |
| `cubic_linear_x8` | each base cube split `2x2x2` | 105,027 | comparable-DOF linear baseline |
| `cubic_hermite` | original R15 cubic mesh, tricubic Hermite | 126,144 | high-order method under test |
| `cubic_linear_x27` | each base cube split `3x3x3` | 331,710 | over-resolved linear point |
| `cubic_linear_x64` | each base cube split `4x4x4` | 759,771 | independent linear convergence point |

The tet hierarchy has the following realized sizes:

| level | tet vertices | tetrahedra | DOFs | DOFs / Hermite |
|---|---:|---:|---:|---:|
| L0 | 210,550 | 1,162,507 | 631,650 | 5.01x |
| L1 | 412,199 | 2,329,584 | 1,236,597 | 9.80x |
| L2 | 840,865 | 4,847,517 | 2,522,595 | 20.00x |

L1 and L2 are independent TetGen remeshes of the same polyhedral domain; they
are not nested refinements of L0.

### Geometry and Boundary Data

| asset | path |
|---|---|
| original surface | `examples/experiments/tricubic_hermite_fem/assets/obj/dragon.obj` |
| attached surface patch | `examples/experiments/tricubic_hermite_fem/assets/fixed/dragon-surface-fixed.txt` |
| base cubic volume mesh | `examples/experiments/tricubic_hermite_fem/assets/veg/cubic/dragon-conservative-r15.veg` |
| common-domain boundary | `examples/experiments/tricubic_hermite_fem/assets/obj/dragon-conservative-r15-surface.obj` |
| x8 cubic-linear mesh | `examples/experiments/tricubic_hermite_fem/assets/veg/cubic/dragon-conservative-r15-subdiv2.veg` |
| x27 cubic-linear mesh | `examples/experiments/tricubic_hermite_fem/assets/veg/cubic/dragon-conservative-r15-subdiv3.veg` |
| original tet L0 mesh | `examples/experiments/tricubic_hermite_fem/assets/veg/tet/dragon-conservative-r15-tet-a1.45885e-7.veg` |
| designated tet L2 reference | `examples/outputs/tricubic_hermite_fem/dragon/tet_convergence/meshes/dragon-conservative-r15-tet_L2-a3.48853e-8.veg` |
| x64 cubic-linear mesh | `examples/outputs/tricubic_hermite_fem/dragon/tet_convergence/meshes/dragon-conservative-r15-subdiv4.veg` |

All volume meshes represent the same conservative domain. Its volume is
approximately `1.733817x` the enclosed volume of the original dragon surface.

### Mesh Generation

The conservative cubic mesh and common-domain boundary are generated at cubic
mesher resolution 15:

```bash
conda run -n libpgo python examples/experiments/tricubic_hermite_fem/mesh/generate_cubic_mesh.py \
  --study dragon --resolution 15
```

The convergence utility reads the original L0 selection, tunes independent L1
and L2 TetGen meshes to approximately `2x` and `4x` its actual DOF count, and
generates the factor-four cubic subdivision:

```bash
MKL_THREADING_LAYER=TBB conda run -n libpgo python -u \
  examples/experiments/tricubic_hermite_fem/tet_convergence/generate_meshes.py \
  --study dragon
```

Tet L2 uses TetGen command `pq1.414a3.488528385435315e-08`. Its volume ratio
agrees with the other same-domain cases to the reported precision.

## Solver Settings

| setting | value |
|---|---:|
| elastic material | Stable Neo-Hookean, `E=1e6`, `nu=0.45` |
| gravity | `(0, -9.81, 0)` |
| surface attachment coefficient | `1e5` |
| attached original-surface vertices | 289 |
| max Newton iterations | 300 |
| standard gradient tolerance | `1e-5` |
| L0-tight gradient tolerance | `1e-6` |
| pin residual acceptance limit | `1.5e-3` |

The attachment is the same embedded original-surface patch in every
formulation. Accuracy metrics exclude the attached vertices.

## Metrics

Let `u_ref` be the designated tet L2 displacement and `u_case` another case's
displacement, both evaluated at the identical original-surface vertices.

| metric | meaning |
|---|---|
| `area_rel_l2_to_L2` | lumped-area-weighted relative displacement L2 on non-attached vertices; primary metric |
| `vertex_rel_l2_to_L2` | unweighted vertex relative displacement L2 on non-attached vertices |
| point error mean/p95/max | distribution of Euclidean displacement differences from L2 |
| `surface_displacement_rms` | deformation-scale diagnostic, not an error metric |
| `pin_residual_max` | maximum displacement magnitude on attached vertices |

The area-weighted metric is primary because it approximates a surface integral
and reduces sensitivity to nonuniform vertex sampling.

## Results

### Solve Summary

| case | formulation | DOFs | converged | iterations | final grad max | wall time | threads |
|---|---|---:|---|---:|---:|---:|---:|
| `cubic_linear` | cubic-linear | 15,768 | yes | 32 | `7.858e-06` | 2.910 s | 12 |
| `cubic_linear_x8` | cubic-linear | 105,027 | yes | 32 | `8.277e-06` | 24.799 s | 12 |
| `cubic_hermite` | cubic-tricubic-Hermite | 126,144 | yes | 32 | `7.342e-06` | 134.457 s | 12 |
| `cubic_linear_x27` | cubic-linear | 331,710 | yes | 31 | `9.406e-06` | 108.967 s | 12 |
| `tet_L0` | tet-linear | 631,650 | yes | 31 | `9.978e-06` | 230.689 s | 21 |
| `tet_L0_tight` | tet-linear | 631,650 | yes | 38 | `9.938e-07` | 270.703 s | 21 |
| `cubic_linear_x64` | cubic-linear | 759,771 | yes | 31 | `9.005e-06` | 298.347 s | 21 |
| `tet_L1` | tet-linear | 1,236,597 | yes | 31 | `9.973e-06` | 676.317 s | 21 |
| `tet_ref` (`tet_L2`) | tet-linear | 2,522,595 | yes | 33 | `9.657e-06` | 2,827.785 s | 21 |

Every case reached its configured nonlinear solver tolerance. L2's final
gradient is close to the `1e-5` threshold and L2 has not been repeated at
`1e-6`; this is tracked as a limitation rather than hidden by the reference
designation.

### Surface Discrepancy Against Tet L2

| case | area rel L2 | vertex rel L2 | mean err | p95 err | max err | pin max |
|---|---:|---:|---:|---:|---:|---:|
| `tet_ref` (`tet_L2`) | 0 | 0 | 0 | 0 | 0 | 0.000362 |
| `cubic_linear_x64` | 0.006911 | 0.006872 | 0.000933 | 0.001385 | 0.001680 | 0.000421 |
| `tet_L1` | 0.012945 | 0.012832 | 0.001757 | 0.002473 | 0.002881 | 0.000459 |
| `cubic_linear_x27` | 0.019769 | 0.019584 | 0.002685 | 0.003801 | 0.004445 | 0.000640 |
| `tet_L0` | 0.026138 | 0.025876 | 0.003528 | 0.005052 | 0.005448 | 0.000581 |
| `cubic_hermite` | 0.028896 | 0.028634 | 0.003922 | 0.005519 | 0.006448 | 0.000548 |
| `cubic_linear_x8` | 0.051618 | 0.051064 | 0.006970 | 0.009985 | 0.011546 | 0.001002 |
| `cubic_linear` | 0.160020 | 0.158554 | 0.021528 | 0.031187 | 0.035684 | 0.000925 |

### Displacement Scale

| case | surface displacement RMS |
|---|---:|
| `cubic_linear` | 0.119944 |
| `cubic_linear_x8` | 0.134916 |
| `cubic_hermite` | 0.138067 |
| `tet_L0` | 0.138239 |
| `cubic_linear_x27` | 0.139259 |
| `tet_L1` | 0.140121 |
| `cubic_linear_x64` | 0.141130 |
| `tet_ref` (`tet_L2`) | 0.141808 |

## Analysis

### Tet Reference Refinement

The original tet L0 reference changes materially under refinement:

- L0-to-L2 area-weighted surface discrepancy is `2.61%`;
- surface RMS increases from `0.138239` at L0 to `0.140121` at L1 and
  `0.141808` at L2;
- the RMS increments are `1.36%` from L0 to L1 and `1.20%` from L1 to L2.

The L0-tight displacement change is only `0.074%` of the L1-to-L2 update, so
the observed refinement trend is not explained by the standard nonlinear
solver tolerance.

The pre-registered tet-sequence checks are mixed. The correction direction
passes (`0.903>0.9`), but the contraction ratio is `rho=0.935`, above the
required `0.8`. Thus L2 is suitable as the designated finite-resolution
reporting reference, but the available sequence does not certify it as the
continuous limit or support a reliable unresolved-tail interval.

### Accuracy at Comparable DOF Count

Hermite and x8 have similar global DOF counts (`126,144` versus `105,027`).
Relative to tet L2:

- Hermite area-weighted discrepancy is `2.89%`, versus `5.16%` for x8;
- x8's discrepancy is `1.79x` Hermite's;
- Hermite p95 point error is `0.005519`, versus `0.009985` for x8;
- Hermite has `1.20x` as many DOFs as x8.

This retains evidence for a Hermite accuracy-per-global-DOF advantage over the
comparable-DOF x8 linear baseline. The effect is smaller than the `3.48x` error
ratio obtained when the under-resolved L0 tet was used as the reference.

### Hermite Versus x27

Relative to tet L2, x27 reverses the earlier L0-reference ordering:

- x27 area-weighted discrepancy is `1.98%`, versus `2.89%` for Hermite;
- x27 is `31.6%` lower in this reference-relative metric;
- x27 uses `2.63x` as many global DOFs as Hermite;
- in the original 12-thread matrix, x27 took `108.967 s` and Hermite took
  `134.457 s`.

The report therefore no longer supports a claim that Hermite is more accurate
than x27 with respect to the designated reference. It supports a tradeoff:
Hermite is much smaller in global DOF count, while x27 is closer to tet L2 and
faster in this implementation.

### x64 and Tet L1

The independent low-order x64 point is closer to L2 than tet L1:

- area-weighted discrepancies are `0.691%` for x64 and `1.294%` for tet L1;
- x64 uses `61.4%` of tet L1's global DOFs;
- both convergence cases used 21 threads, with wall times of `298.347 s` and
  `676.317 s`, respectively.

The cubic refinement corrections from x8 to x27 and x27 to x64 have direction
cosine `0.980`, providing a coherent low-order refinement trend toward the L2
reference.

### Constraint Behavior

All pin residuals are at or below `0.001002`, below the configured `1.5e-3`
acceptance limit. Constraint residual does not explain the accuracy ordering.

### Cost and Comparability

| comparison | value |
|---|---:|
| Hermite DOFs / x8 DOFs | 1.20x |
| x8 discrepancy / Hermite discrepancy | 1.79x |
| x27 DOFs / Hermite DOFs | 2.63x |
| Hermite discrepancy / x27 discrepancy | 1.46x |
| x64 DOFs / Hermite DOFs | 6.02x |
| Hermite discrepancy / x64 discrepancy | 4.18x |
| tet L2 DOFs / Hermite DOFs | 20.00x |

Timing comparisons are valid within the canonical 12-thread matrix or within
the convergence 21-thread matrix, not across those two matrices.

## Conclusions

For this static dragon case:

1. All original formulation cases and all five convergence-study cases reach
   their configured nonlinear solver tolerances. The L0-tight control shows
   that standard solver tolerance is not the primary cause of the observed
   L0-to-L2 refinement trend.
2. Tet L2 is adopted as the designated numerical reference for this report.
   The original tet L0 is not spatially converged relative to that reference:
   its area-weighted surface discrepancy is `2.61%`.
3. At comparable global DOF count, Hermite remains closer to tet L2 than x8
   (`2.89%` versus `5.16%`): x8's discrepancy is `1.79x` Hermite's while
   Hermite uses `1.20x` as many DOFs.
4. x27 is closer to tet L2 than Hermite (`1.98%` versus `2.89%`) but uses
   `2.63x` as many global DOFs. The earlier claim that Hermite is more accurate
   than x27 is therefore withdrawn for the L2-reference comparison.
5. x64 is the closest cubic-linear case at `0.69%` and is closer to L2 than
   tet L1's `1.29%` discrepancy while using fewer global DOFs.
6. L2 is the finest available same-domain numerical reference, not a certified
   continuous solution. The failed tet contraction criterion means that the
   experiment does not establish an error bound between L2 and the continuous
   limit.

The supported formulation claim is therefore narrower than in the original
L0-reference report: Hermite improves substantially over the base and
comparable-DOF x8 cubic-linear discretizations, while x27 provides better
L2-relative accuracy at substantially higher global DOF count.

## Limitations

- Only one dragon geometry, load case, material, and attachment patch are used.
- The conservative domain is `1.733817x` the original surface volume.
- L1 and L2 are independent TetGen remeshes rather than nested refinements.
- Tet L2 has not been repeated at the tighter `1e-6` solver tolerance.
- Tet L2 is the designated numerical reference, not certified ground truth.
- The convergence study evaluates surface displacement norms and does not yet
  include volume L2 or energy-norm error estimates.
- The reference designation was selected after observing the convergence run;
  L2-relative results are descriptive rather than confirmatory certification.
- Canonical formulation timings used 12 threads, while convergence timings
  used 21 threads; wall times cannot be compared across the two matrices.
- Each wall time is a single observation without run-to-run dispersion.

## Reproduction

Generate the independent L1/L2 tet meshes and x64 cubic mesh:

```bash
MKL_THREADING_LAYER=TBB conda run -n libpgo python -u \
  examples/experiments/tricubic_hermite_fem/tet_convergence/generate_meshes.py \
  --study dragon
```

Run the convergence cases with the server configuration used here:

```bash
MKL_THREADING_LAYER=TBB conda run -n libpgo python -u \
  examples/experiments/tricubic_hermite_fem/tet_convergence/run_static.py \
  --study dragon --num-threads 21 --force
```

Regenerate the L2-relative convergence artifacts:

```bash
conda run -n libpgo python \
  examples/experiments/tricubic_hermite_fem/tet_convergence/analyze.py \
  --study dragon
```

The reference-relative machine-readable artifacts are written under:

```text
examples/outputs/tricubic_hermite_fem/dragon/tet_convergence/
```
