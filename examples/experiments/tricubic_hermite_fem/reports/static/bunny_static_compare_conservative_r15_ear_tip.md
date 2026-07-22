# Bunny Static FEM Comparison with a Tet L2 Numerical Reference

This report compares cubic-linear and cubic-tricubic-Hermite FEM on the same
conservative R15 bunny domain under static gravity and an ear-tip attachment.
The finest available same-domain tet-linear discretization, `tet_L2`, is the
designated numerical reference for all reference-relative accuracy metrics.

The main question is:

> Relative to the designated tet L2 numerical reference, how do tricubic
> Hermite and successive cubic-linear refinements compare in visible-surface
> displacement accuracy and global DOF count?

At comparable global DOF count, Hermite remains more accurate than
`cubic_linear_x8`: their area-weighted surface discrepancies are `6.74%` and
`12.92%`, respectively. The more highly refined `cubic_linear_x27` case is
closer to tet L2 than Hermite (`5.91%` versus `6.74%`) while using `2.74x` as
many DOFs. The new `cubic_linear_x64` case is closest among the cubic methods
at `2.79%`, nearly matching tet L1's `2.84%` discrepancy with fewer DOFs.

Tet L2 is the reporting reference, not a mathematically certified continuous
solution. The earlier L0 tet reference is retained below as `tet_L0` so that
the effect of reference refinement remains explicit.

## Run Provenance

| item | canonical formulation matrix | tet-convergence matrix |
|---|---|---|
| run date | 2026-07-22 | 2026-07-22 |
| code commit | `916be0e8ae37e88f62c9532392fe85947a89eff4` | same |
| platform | Linux server | same |
| TBB concurrency limit | 12 | 21 |
| MKL threading layer | TBB | TBB |
| execution | fresh sequential `--force` run | fresh sequential tmux run |
| output | `examples/experiments/tricubic_hermite_fem/output/bunny/static/` | `examples/experiments/tricubic_hermite_fem/output/bunny/tet_convergence/` |

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
| `tet_ref` (`tet_L2`) | independently remeshed tet-linear L2 | 1,458,978 | designated numerical reference |
| `tet_L1` | independently remeshed tet-linear L1 | 717,012 | tet convergence point |
| `tet_L0` | original canonical tet-linear reference mesh | 365,184 | earlier numerical reference |
| `tet_L0_tight` | same L0 mesh, `1e-6` gradient tolerance | 365,184 | solver-tolerance control |
| `cubic_linear` | original R15 cubic mesh, cubic-linear | 9,102 | low-order baseline |
| `cubic_linear_x8` | each base cube split `2x2x2` | 62,355 | comparable-DOF linear baseline |
| `cubic_hermite` | original R15 cubic mesh, tricubic Hermite | 72,816 | high-order method under test |
| `cubic_linear_x27` | each base cube split `3x3x3` | 199,326 | over-resolved linear point |
| `cubic_linear_x64` | each base cube split `4x4x4` | 459,579 | independent linear convergence point |

The tet hierarchy has the following realized sizes:

| level | tet vertices | tetrahedra | DOFs | DOFs / Hermite |
|---|---:|---:|---:|---:|
| L0 | 121,728 | 687,523 | 365,184 | 5.02x |
| L1 | 239,004 | 1,376,342 | 717,012 | 9.85x |
| L2 | 486,326 | 2,843,865 | 1,458,978 | 20.04x |

L1 and L2 are independent TetGen remeshes of the same polyhedral domain; they
are not nested refinements of L0.

### Geometry and Boundary Data

| asset | path |
|---|---|
| original surface | `examples/experiments/tricubic_hermite_fem/assets/obj/bunny.obj` |
| attached ear-tip patch | `examples/experiments/tricubic_hermite_fem/assets/fixed/bunny-surface-fixed-ear-tip.txt` |
| base cubic volume mesh | `examples/experiments/tricubic_hermite_fem/assets/veg/cubic/bunny-conservative-r15.veg` |
| common-domain boundary | `examples/experiments/tricubic_hermite_fem/assets/obj/bunny-conservative-r15-surface.obj` |
| x8 cubic-linear mesh | `examples/experiments/tricubic_hermite_fem/assets/veg/cubic/bunny-conservative-r15-subdiv2.veg` |
| x27 cubic-linear mesh | `examples/experiments/tricubic_hermite_fem/assets/veg/cubic/bunny-conservative-r15-subdiv3.veg` |
| original tet L0 mesh | `examples/experiments/tricubic_hermite_fem/assets/veg/tet/bunny-conservative-r15-tet-a2.8768e-9.veg` |
| designated tet L2 reference | `examples/experiments/tricubic_hermite_fem/output/bunny/tet_convergence/meshes/bunny-conservative-r15-tet_L2-a6.94265e-10.veg` |
| x64 cubic-linear mesh | `examples/experiments/tricubic_hermite_fem/output/bunny/tet_convergence/meshes/bunny-conservative-r15-subdiv4.veg` |

All volume meshes represent the same conservative domain. Its volume is
approximately `1.479166x` the enclosed volume of the original bunny surface.

### Mesh Generation

The conservative cubic mesh and common-domain boundary are generated at cubic
mesher resolution 15:

```bash
conda run -n libpgo python examples/experiments/tricubic_hermite_fem/mesh/generate_cubic_mesh.py \
  --study bunny --resolution 15
```

The convergence utility reads the original L0 selection, tunes independent L1
and L2 TetGen meshes to approximately `2x` and `4x` its actual DOF count, and
generates the factor-four cubic subdivision:

```bash
MKL_THREADING_LAYER=TBB conda run -n libpgo python -u \
  examples/experiments/tricubic_hermite_fem/tet_convergence/generate_meshes.py \
  --study bunny
```

Tet L2 uses TetGen command
`pq1.414a6.9426525392247264e-10`. Its volume ratio agrees with the other
same-domain cases to the reported precision.

## Solver Settings

| setting | value |
|---|---:|
| elastic material | Stable Neo-Hookean, `E=1e6`, `nu=0.45` |
| gravity | `(0, -9.81, 0)` |
| surface attachment coefficient | `1e5` |
| attached original-surface vertices | 18 |
| max Newton iterations | 300 |
| standard gradient tolerance | `1e-5` |
| L0-tight gradient tolerance | `1e-6` |
| pin residual acceptance limit | `1.5e-3` |

The attachment is the same embedded original-surface ear-tip patch in every
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
| `cubic_linear` | cubic-linear | 9,102 | yes | 26 | `9.635e-06` | 1.665 s | 12 |
| `cubic_linear_x8` | cubic-linear | 62,355 | yes | 24 | `8.394e-06` | 13.721 s | 12 |
| `cubic_hermite` | cubic-tricubic-Hermite | 72,816 | yes | 30 | `8.409e-06` | 97.743 s | 12 |
| `cubic_linear_x27` | cubic-linear | 199,326 | yes | 27 | `8.096e-06` | 91.546 s | 12 |
| `tet_L0` | tet-linear | 365,184 | yes | 26 | `8.752e-06` | 167.294 s | 21 |
| `tet_L0_tight` | tet-linear | 365,184 | yes | 35 | `9.622e-07` | 216.547 s | 21 |
| `cubic_linear_x64` | cubic-linear | 459,579 | yes | 29 | `8.314e-06` | 293.518 s | 21 |
| `tet_L1` | tet-linear | 717,012 | yes | 30 | `8.444e-06` | 589.778 s | 21 |
| `tet_ref` (`tet_L2`) | tet-linear | 1,458,978 | yes | 30 | `9.963e-06` | 2,655.075 s | 21 |

Every case reached its configured nonlinear solver tolerance. L2's final
gradient is close to the `1e-5` threshold and L2 has not been repeated at
`1e-6`; this is tracked as a limitation rather than hidden by the reference
designation.

### Surface Discrepancy Against Tet L2

| case | area rel L2 | vertex rel L2 | mean err | p95 err | max err | pin max |
|---|---:|---:|---:|---:|---:|---:|
| `tet_ref` (`tet_L2`) | 0 | 0 | 0 | 0 | 0 | 0.0000176 |
| `cubic_linear_x64` | 0.027940 | 0.028081 | 0.000592 | 0.000859 | 0.000902 | 0.0000335 |
| `tet_L1` | 0.028435 | 0.029005 | 0.000623 | 0.000852 | 0.000919 | 0.0000280 |
| `cubic_linear_x27` | 0.059141 | 0.060280 | 0.001290 | 0.001780 | 0.001898 | 0.0000279 |
| `tet_L0` | 0.064149 | 0.064490 | 0.001380 | 0.001861 | 0.001948 | 0.0000300 |
| `cubic_hermite` | 0.067439 | 0.068194 | 0.001470 | 0.001921 | 0.002015 | 0.0000344 |
| `cubic_linear_x8` | 0.129227 | 0.131306 | 0.002815 | 0.003831 | 0.004077 | 0.0000485 |
| `cubic_linear` | 0.249255 | 0.251769 | 0.005394 | 0.007274 | 0.007625 | 0.0000684 |

### Displacement Scale

| case | surface displacement RMS |
|---|---:|
| `cubic_linear` | 0.018100 |
| `cubic_linear_x8` | 0.020250 |
| `cubic_hermite` | 0.021152 |
| `tet_L0` | 0.021180 |
| `cubic_linear_x27` | 0.021379 |
| `tet_L1` | 0.021666 |
| `cubic_linear_x64` | 0.021933 |
| `tet_ref` (`tet_L2`) | 0.022177 |

## Analysis

### Tet Reference Refinement

The original tet L0 reference changes materially under refinement:

- L0-to-L2 area-weighted surface discrepancy is `6.41%`;
- surface RMS increases from `0.021180` at L0 to `0.021666` at L1 and
  `0.022177` at L2;
- the RMS increments are `2.29%` from L0 to L1 and `2.36%` from L1 to L2.

The L0-tight displacement change is only `0.91%` of the L1-to-L2 update, so
the observed refinement trend is not primarily explained by the standard
nonlinear solver tolerance.

The pre-registered tet-sequence checks are mixed. The contraction ratio passes
(`rho=0.743<0.8`), but the direction cosine between successive corrections is
`0.847`, below the required `0.9`. Thus L2 is suitable as the designated
finite-resolution reporting reference, but the available sequence does not
certify it as the continuous limit or support a reliable unresolved-tail
interval.

### Accuracy at Comparable DOF Count

Hermite and x8 have similar global DOF counts (`72,816` versus `62,355`).
Relative to tet L2:

- Hermite area-weighted discrepancy is `6.74%`, versus `12.92%` for x8;
- x8's discrepancy is `1.92x` Hermite's;
- Hermite p95 point error is `0.001921`, versus `0.003831` for x8;
- Hermite has only `1.17x` as many DOFs as x8.

This retains the evidence for a Hermite accuracy-per-global-DOF advantage over
the comparable-DOF x8 linear baseline. The effect is smaller than the `8.70x`
ratio obtained when the under-resolved L0 tet was used as the reference.

### Hermite Versus x27

Relative to tet L2, x27 reverses the earlier L0-reference ordering:

- x27 area-weighted discrepancy is `5.91%`, versus `6.74%` for Hermite;
- x27 is `12.3%` lower in this reference-relative metric;
- x27 uses `2.74x` as many global DOFs as Hermite;
- in the original 12-thread matrix, x27 took `91.546 s` and Hermite took
  `97.743 s`.

The report therefore no longer supports a claim that Hermite is more accurate
than x27 with respect to the designated reference. It supports a tradeoff:
Hermite is much smaller in global DOF count, while x27 is modestly closer to
tet L2.

### x64 and Tet L1

The independent low-order x64 point nearly matches tet L1:

- area-weighted discrepancies are `2.794%` for x64 and `2.844%` for tet L1;
- x64 uses `64.1%` of tet L1's global DOFs;
- both convergence cases used 21 threads, with wall times of `293.518 s` and
  `589.778 s`, respectively.

The cubic refinement corrections from x8 to x27 and x27 to x64 have direction
cosine `0.948`, providing a coherent low-order refinement trend toward the L2
reference.

### Constraint Behavior

All pin residuals are below `6.85e-5`, far below the configured `1.5e-3`
acceptance limit. Constraint residual does not explain the accuracy ordering.

### Cost and Comparability

| comparison | value |
|---|---:|
| Hermite DOFs / x8 DOFs | 1.17x |
| x8 discrepancy / Hermite discrepancy | 1.92x |
| x27 DOFs / Hermite DOFs | 2.74x |
| Hermite discrepancy / x27 discrepancy | 1.14x |
| x64 DOFs / Hermite DOFs | 6.31x |
| Hermite discrepancy / x64 discrepancy | 2.41x |
| tet L2 DOFs / Hermite DOFs | 20.04x |

Timing comparisons are valid within the canonical 12-thread matrix or within
the convergence 21-thread matrix, not across those two matrices.

## Conclusions

For this static bunny ear-tip case:

1. All original formulation cases and all five convergence-study cases reach
   their configured nonlinear solver tolerances. The L0-tight control shows
   that standard solver tolerance is not the primary cause of the observed
   L0-to-L2 refinement trend.
2. Tet L2 is adopted as the designated numerical reference for this report.
   The original tet L0 is not spatially converged relative to that reference:
   its area-weighted surface discrepancy is `6.41%`.
3. At comparable global DOF count, Hermite remains closer to tet L2 than x8
   (`6.74%` versus `12.92%`): x8's discrepancy is `1.92x` Hermite's while
   Hermite uses `1.17x` as many DOFs.
4. x27 is modestly closer to tet L2 than Hermite (`5.91%` versus `6.74%`) but
   uses `2.74x` as many global DOFs. The earlier claim that Hermite is more
   accurate than x27 is therefore withdrawn for the L2-reference comparison.
5. x64 is the closest cubic-linear case at `2.79%` and nearly matches tet L1's
   `2.84%` discrepancy while using fewer global DOFs.
6. L2 is the finest available same-domain numerical reference, not a certified
   continuous solution. The failed tet correction-direction criterion means
   that the experiment does not establish an error bound between L2 and the
   continuous limit.

The supported formulation claim is therefore narrower than in the original
L0-reference report: Hermite improves substantially over the base and
comparable-DOF x8 cubic-linear discretizations, while x27 provides a modest
reference-relative accuracy improvement at substantially higher global DOF
count.

## Limitations

- Only one bunny geometry, load case, material, and attachment patch are used.
- The conservative domain is `1.479166x` the original surface volume.
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
  --study bunny
```

Run the convergence cases with the server configuration used here:

```bash
MKL_THREADING_LAYER=TBB conda run -n libpgo python -u \
  examples/experiments/tricubic_hermite_fem/tet_convergence/run_static.py \
  --study bunny --num-threads 21 --force
```

Regenerate the L2-relative convergence artifacts:

```bash
conda run -n libpgo python \
  examples/experiments/tricubic_hermite_fem/tet_convergence/analyze.py \
  --study bunny
```

The reference-relative machine-readable artifacts are written under:

```text
examples/experiments/tricubic_hermite_fem/output/bunny/tet_convergence/
```
