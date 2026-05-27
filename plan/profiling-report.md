# IPC Simulation Profiling Report

## Environment

| Item | Value |
|------|-------|
| Example | `examples/ipc/cubic/box-with-sphere-lite/box-ipc.json` |
| Mesh type | Cubic (9735 elements) |
| Simulation DOFs | 77880 (25960 vertices × 3) |
| Surface vertices | 522 |
| External obstacles | 1 (bottom plane, static) |
| Timesteps | 2000 |
| Timestep size | 0.001s |
| Solver | Implicit Backward Euler + Newton + Pardiso |
| Material | Stable Neo-Hookean |
| IPC contact | Codimensional IPC (dhat=0.002, kappa=3000, dhat_external=0.005, eps_ee=0) |
| Material max-step | Enabled |
| Max Newton iters | 200 |
| Solver tolerance | 1e-3 |
| Damping | None (0, 0) |

## Applied Optimizations

| # | Optimization | Files Changed | Impact |
|---|-------------|--------------|--------|
| 1 | `func_grad_hessian` unification | `NewtonSolver.cpp`, `implicitBackwardEulerTimeIntegratorHelper.cpp/h` | buildActiveSet -71.8s (-24%) |
| 2 | Extract AABB + `collectPairsParallel` | `surfaceIPCBroadPhase.cpp` | Code: 406→288 lines, no perf change |
| 3 | Classify-once dispatchers (`computeEESqDistAll`/`computePTSqDistAll`) | `ipcDistancePrimitives.cpp/h`, `surfaceIPCBarrierKernels.cpp` | active_set_combined -6s (-8%) |
| 4 | Barrier kernel refactor | `surfaceIPCBarrierKernels.cpp` | Code: 178→114 lines, no perf change |
| 5 | `SpatialHashGrid::build()` bulk hash construction + grid-range cache | `spatialHashGrid.cpp/h`, `surfaceIPCBroadPhase.cpp`, `surfaceIPCMaxStep.cpp`, `obstaclePoseCache.cpp` | Synthetic hash-build microbench: repeated `insert()` 5.48s → `build()` 3.88s (1.41× faster). |
| 6 | Obstacle-level external broad-phase early-out + lazy dynamic tri/edge AABB construction | `obstaclePoseCache.cpp/h`, `surfaceIPCBroadPhase.cpp`, `surfaceIPCMaxStep.cpp` | Final repeated 20-step no-contact smoke: `external` pair build 0.216s → 0.00485-0.00521s; `buildActiveSet` 0.894s → 0.723-0.791s. |
| 7 | `queryAfter()` upper-triangle query for self EE pairs | `spatialHashGrid.cpp/h`, `surfaceIPCBroadPhase.cpp`, `surfaceIPCMaxStep.cpp` | Kept as candidate suppression for self EE. Repeated 20-step smoke is noisy: `self_ee_query` 0.286-0.329s around the pre-change 0.311s sample. |
| 8 | Profile counters for broad-phase and CCD candidate pressure | `scopedProfileSection.cpp/h`, `surfaceIPCProfiling.h`, `surfaceIPCBroadPhase.cpp`, `surfaceIPCMaxStep.cpp`, `runIPCSim.cpp` | Diagnostic-only when profiling is enabled. Final full run now reports hash candidates, exact distance/CCD tests, accepted pairs, and obstacle-overlap counts. |
| 9 | Demote per-active-set and Hessian-nonzero hot-path logs to debug | `surfaceIPCCore.cpp`, `runIPCSim_gtest.cpp` | 20-step counter smoke: `build_active_set` 0.7608s → 0.6932s / 140 calls, and full-run info log no longer contains 25k active-pair lines or 6.7k Hessian lines. |
| 10 | Split combined barrier assembly profiling by self/external and PT/TP/EE | `surfaceIPCProfiling.h`, `surfaceIPCSelfBarrierAssembler.cpp`, `surfaceIPCExternalBarrierAssembler.cpp`, `surfaceIPCBarrierAssembler_gtest.cpp` | Diagnostic-only. Full run shows `active_set_combined` is almost entirely external barrier work: 82.91s total, 82.43s external, 0.48s self; external EE alone is 51.47s. |
| 11 | AABB-overlap filtering inside active-set hash queries | `spatialHashGrid.cpp/h`, `surfaceIPCBroadPhase.cpp`, `spatialHashGrid_gtest.cpp` | Keeps raw hash-candidate counters but returns only AABB-overlapping candidates to broad-phase callers. Final 20-step smoke: `build_active_set` 0.676-0.690s / 140 calls, `adapter.max_step` unchanged at 0.093-0.095s because CCD max-step keeps the old query path. Full broad-only profile improves broad-phase per-call time: `build_active_set` 8.03ms → 7.82ms, `static` 5.07ms → 4.96ms, `external` 2.95ms → 2.86ms. |
| 12 | Packed low-bit spatial hash key | `spatialHashGrid.cpp/h`, `spatialHashGrid_gtest.cpp` | Replaces XOR-prime hash keys with a 63-bit packed `(ix,iy,iz)` low-bit key. Full profile reduces raw candidate enumeration and broad-phase time: `build_active_set` 196.03s / 25054 calls (7.82ms/call) → 189.72s / 25290 calls (7.50ms/call), with self EE raw candidates 4.28B → 3.45B and external EE raw candidates 1.95B → 1.39B. |
| 13 | Hoist query-side vertex/velocity extraction out of candidate loops | `surfaceIPCBroadPhase.cpp`, `surfaceIPCMaxStep.cpp` | Purely local CPU reduction: candidate counts are unchanged, but repeated `segment<3>()` copies for the same query vertex/edge are avoided while scanning candidates. Same-environment 20-step A/B: `build_active_set` 0.331-0.351s → 0.295-0.321s / 140 calls, `self_ee_query` 0.086-0.101s → 0.070-0.081s, and `adapter.max_step` 0.052-0.064s → 0.047-0.052s. |
| 14 | External dynamic-block barrier wrapper + lazy 12-DOF temporaries | `surfaceIPCBarrierKernels.cpp/h`, `surfaceIPCExternalBarrierAssembler.cpp`, `ipcDistancePrimitives.h`, `surfaceIPCBarrierAssembler_gtest.cpp` | External PT/TP/EE assembly now scatters dynamic-only 3/9/6-DOF contribution blocks while preserving the generic 12-DOF PSD-projected dynamic sub-block semantics. Same-environment 600-step A/B: `active_set_combined.external` 14.45s → 13.21s, and `external_ee` 7.13s / 2.29M pairs → 6.74s / 2.28M pairs (3.12us/pair → 2.96us/pair). |
| 15 | Backtracking line-search initial-trial energy reuse | `lineSearch.cpp/h`, `NewtonSolver.cpp`, `NewtonSolver_gtest.cpp` | Reuses the `x + dx` energy that Newton already computes before default backtracking, removing one duplicated alpha=1 `func()` call per Newton iteration. 600-step retained-code profile: `contact.surface.energy` 3024 calls / 46.07s → 1800 calls / 25.43s, and `build_active_set` 6023 calls / 87.57s → 4798 calls / 63.91s, with external EE barrier pair volume unchanged at about 2.275M. |
| 16 | Newton standalone-gradient elimination | `NewtonSolver.cpp`, `solveDiagnostics.h`, `implicitBackwardEulerTimeIntegrator.cpp`, `TRBDF2TimeIntegratorHelper.cpp/h`, `NewtonSolver_gtest.cpp`, `implicitBackwardEulerTimeIntegrator_gtest.cpp` | Removes the duplicate initial Newton `gradient()` call and reuses Newton's final converged gradient diagnostics for implicit-Euler residual printing. 600-step retained-code profile after #15: `contact.surface.gradient` 1200 calls / 15.30s → 0 calls, `build_active_set` 4798 calls / 63.91s → 3598 calls / 35.02s, with `solver.linear_solve` unchanged at 1198 and external EE barrier pairs unchanged at about 2.275M. |
| 17 | Cached implicit energy-component printing | `implicitBackwardEulerTimeIntegrator.cpp`, `implicitBackwardEulerTimeIntegratorHelper.cpp/h`, `implicitBackwardEulerTimeIntegrator_gtest.cpp` | Reuses the per-model energy components already computed by the final converged `func_grad_hessian()` call when printing timestep energy components. 600-step retained-code profile after #16: `contact.surface.energy` 1800 calls / 18.23s → 1206 calls / 11.42s, `build_active_set` 3598 calls / 35.02s → 3005 calls / 27.86s, and log wall-clock 88.4s → 75.2s. |
| 18 | Accepted line-search active-set cache | `embeddedSurfaceIPCPotentialEnergy.cpp/h`, `embeddedSurfaceIPCPotentialEnergy_gtest.cpp`, `NewtonSolver.cpp`, `NewtonSolver_gtest.cpp` | Reuses the active set built by an energy-only line-search evaluation when the next fused Newton evaluation asks for the exact same surface position. 600-step retained-code profile after #17: `build_active_set` 3005 calls / 27.86s → 1837 calls / 26.27s, `contact.adapter.hessian_direct` 35.40s → 28.17s, and standalone IPC gradient calls remain absent. Wall-clock was noisy in this run and is not used as attribution. |
| 19 | Coplanar obstacle-edge filtering for external EE | `obstacleSurface.cpp/h`, `obstaclePoseCache.cpp/h`, `surfaceIPCBroadPhase.cpp`, `surfaceIPCMaxStep.cpp`, `surfaceIPCPairs.h`, `surfaceIPCCore_gtest.cpp`, `surfaceIPCExternalBroadPhase_gtest.cpp` | External EE obstacle caches now keep boundary, nonmanifold, sharp, and degenerate edges, but drop coplanar interior tessellation edges. On the 600-step flat-box profile, `active_set_combined.external_ee` dropped from 8.124s / 2.281M pairs to 0.0001s / 0 pairs, `pair_build.external_ee` from 2.874s to 0.535s, and all 600 timesteps converged. |
| 20 | Line-search active-set superset reuse | `potentialEnergy.h`, `potentialEnergies.cpp/h`, `NewtonSolver.cpp`, `mappedSurfacePotentialEnergy.cpp/h`, `surfaceIPCBroadPhase.cpp/h`, `surfaceIPCCore.cpp/h`, `embeddedSurfaceIPCPotentialEnergy.cpp/h`, `surfaceIPCCore_gtest.cpp`, `embeddedSurfaceIPCPotentialEnergy_gtest.cpp` | Builds one swept-AABB IPC active-set superset for the default line-search segment and reuses it across trial `func()` evaluations. Same-host 600-step A/B versus `/private/tmp/libpgo-post-revert-baseline-run.log`: `build_active_set` 29.51s / 1877 calls → 21.95s / 1734 calls, `contact.surface.energy` 20.86s / 1277 calls → 14.88s / 1134 calls, and all 600 timesteps converged. |

## Current Session Notes (2026-05-18)

Validation commands:

```bash
cmake --build build/base_no_mkl --target runIPCSim scopedProfileSection_gtest spatialHashGrid_gtest surfaceIPCSelfBroadPhase_gtest surfaceIPCExternalBroadPhase_gtest surfaceIPCMaxStep_gtest surfaceIPCExternalMaxStep_gtest surfaceIPCBarrierAssembler_gtest surfaceIPCCore_gtest embeddedSurfaceIPCPotentialEnergy_gtest runIPCSim_gtest -j 8
./build/base_no_mkl/tests/src/core/profiling/scopedProfileSection_gtest
./build/base_no_mkl/tests/src/core/contact/spatialHashGrid_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCExternalBroadPhase_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCMaxStep_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCExternalMaxStep_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCBarrierAssembler_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCCore_gtest
./build/base_no_mkl/tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
```

Short smoke profile used `examples/ipc/cubic/box-with-sphere-lite/box-ipc.json` with `num-timestep=20`, `dump-interval=1000`, `profiling=true`, built with `build/base_no_mkl`. The final retained code was run three times after rebuilding `runIPCSim`; the table reports the observed range.

| Measurement | Before this session | After #5/#6/#7 | Notes |
|-------------|---------------------|----------------|-------|
| `contact.surface.build_active_set` | 0.894s / 140 calls | 0.723-0.791s / 140 calls | Early no-contact phase; full-run impact depends on how long obstacles remain globally separated. |
| `contact.surface.pair_build.external` | 0.216s / 140 calls | 0.00485-0.00521s / 140 calls | Obstacle global AABB early-out skips PT/TP/EE query passes. |
| `contact.surface.pair_build.external_aabb` | 0.040s / 140 calls | 0.00374-0.00399s / 140 calls | Dynamic tri/edge AABBs are now built only if an obstacle overlaps the dynamic vertex AABB. |
| `contact.surface.pair_build.self_ee_query` | 0.311s / 140 calls | 0.286-0.329s / 140 calls | `queryAfter()` avoids collecting lower-triangle EE candidates; isolated timing gain is small/noisy. |
| `contact.adapter.max_step` | 0.137s / 20 calls | 0.0896-0.109s / 20 calls | External CCD returns before dynamic tri/edge swept AABBs when obstacle global AABB is separated. |

Additional 20-step counter/log smoke after #8/#9:

| Measurement | With counters before log demotion | After #9 | Notes |
|-------------|-----------------------------------|----------|-------|
| `contact.surface.build_active_set` | 0.7608s / 140 calls | 0.6932s / 140 calls | Removes default info logging from the timed active-set path. |
| `contact.surface.pair_build.self_ee_query` | 0.3369s / 140 calls | 0.3018s / 140 calls | Same candidate counts; timing remains noisy but improves with less log work. |
| `contact.surface.pair_build.self_pt_query` | 0.1279s / 140 calls | 0.1177s / 140 calls | No-contact early phase. |
| `contact.surface.pair_build.external` | 0.00514s / 140 calls | 0.00582s / 140 calls | All 140 external broad-phase calls are skipped by obstacle AABB early-out. |

Additional 20-step smoke after #11, with overlap filtering limited to active-set broad phase:

| Measurement | After #9 single sample / retained range | After #11 broad-only | Notes |
|-------------|------------------------------------------|----------------------|-------|
| `contact.surface.build_active_set` | 0.6932s / 140 calls | 0.676-0.690s / 140 calls | Slight early-phase improvement; short runs remain noisy. |
| `contact.surface.pair_build.self_ee_query` | 0.3018s / 140 calls | 0.259-0.279s / 140 calls | The query now avoids returning same-cell AABBs that fail the overlap test. |
| `contact.surface.pair_build.self_pt_query` | 0.1177s / 140 calls | 0.116-0.119s / 140 calls | Essentially neutral. |
| `contact.adapter.max_step` | 0.0896-0.109s / 20 calls | 0.093-0.095s / 20 calls | CCD max-step keeps its pre-#11 query path after the broader experiment showed regression risk. |

Additional 20-step smoke after #12, with the packed hash key:

| Measurement | #11 broad-only | #12 packed key | Notes |
|-------------|----------------|----------------|-------|
| `contact.surface.build_active_set` | 0.676-0.690s / 140 calls | 0.616-0.688s / 140 calls after warmup | Initial runs had host-noise outliers up to 1.19s; stable runs improved. |
| `contact.surface.pair_build.self_ee.hash_candidates` | 24.07M / 140 samples | 18.99M / 140 samples | The packed key avoids the dense low-coordinate collisions produced by the previous XOR-prime key. |
| `contact.surface.pair_build.self_pt.hash_candidates` | 8.68M / 140 samples | 7.12M / 140 samples | Exact distance-test counts are unchanged, so this reduces raw cell enumeration/pruning. |
| `contact.adapter.max_step` | 0.093-0.095s / 20 calls | 0.088-0.096s / 20 calls after warmup | Max-step raw hash candidates also drop, but the short timing remains noisy. |

Additional 20-step same-environment A/B after #13, with query-side primitive extraction hoisted out of candidate loops:

| Measurement | #12 same-env baseline | #13 retained | Notes |
|-------------|-----------------------|--------------|-------|
| `contact.surface.build_active_set` | 0.331-0.351s / 140 calls | 0.295-0.321s / 140 calls | Rebuilt and toggled the local hoist in the same host state to avoid attributing machine noise. |
| `contact.surface.pair_build.self_ee_query` | 0.086-0.101s / 140 calls | 0.070-0.081s / 140 calls | Query-side edge endpoints are now loaded once per query edge, not once per candidate edge. |
| `contact.surface.pair_build.self_pt_query` | 0.042-0.046s / 140 calls | 0.033-0.039s / 140 calls | Query vertex extraction is similarly hoisted for PT. |
| `contact.adapter.max_step` | 0.052-0.064s / 20 calls | 0.047-0.052s / 20 calls | Same hoist is applied to self/external CCD loops; this scene still skips all external CCD calls. |
| `pair_build.self_ee.hash_candidates` / `self_pt.hash_candidates` | 18.99M / 7.12M | 18.99M / 7.12M | Candidate pressure is unchanged; #13 only reduces repeated query-side coordinate loads. |

## Full 2000-Step Retained-Code Profile (2026-05-18)

Config: temporary copy of `box-ipc.json` with `profiling=true` and output redirected to `ret-barrier-profile`; 2000 timesteps, `dump-interval=10`. Final code includes #5-#10. Wall-clock log timestamps were 06:26:17.071 → 06:32:56.955, about 400s.

| Measurement | Source profile / previous reference | Final retained code | Notes |
|-------------|-------------------------------------|---------------------|-------|
| `contact.surface.build_active_set` | 224.0s / 24861 calls | 197.22s / 24560 calls | Full-run broad phase is now about 12% lower than the post-#1/#3 reference despite later external contact. |
| `contact.surface.pair_build.static` | self breakdown 145.9s | 124.57s / 24560 calls | Self PT/EE still dominates static broad-phase time, but external overlap also contributes heavily after contact. |
| `contact.surface.pair_build.external` | 95.1s / 24861 calls | 72.54s / 24560 calls | Obstacle AABB early-out skips 2545 static calls in the full run; after contact, external EE/TP dominate. |
| `contact.surface.pair_build.external_aabb` | 8.2s | 4.82s / 46575 scoped AABB blocks | Dynamic tri/edge AABBs are lazy-built only for overlapping-obstacle calls. |
| `contact.adapter.max_step` | 42.6s / 4679 calls | 21.87s / 4701 calls | External CCD broad phase is skipped for all max-step calls; remaining cost is self swept PT/EE. |
| `contact.surface.active_set_combined` | 67.0s / 6669 calls | 82.91s / 6676 calls | Barrier assembly remains prominent; #10 shows this is almost entirely external barrier work. |
| `solver.linear_solve` | 8.7s / 4679 calls | 6.48s / 4701 calls | Still not a bottleneck. |

Candidate counters from the same full run:

| Counter | Total | Avg / sample | Max / sample | Interpretation |
|---------|-------|--------------|--------------|----------------|
| `pair_build.external.overlapping_obstacles` | 22015 / 24560 samples | 89.6% overlapping | 1 | Static external early-out helps mainly before first obstacle contact. |
| `pair_build.self_ee.hash_candidates` → `distance_tests` → `accepted_pairs` | 4.20B → 126.6M → 35.9k | 171k → 5154 → 1.46 | 193k → 5592 → 33 | Self EE spends most time enumerating/pruning hash candidates, not evaluating accepted pairs. |
| `pair_build.self_pt.hash_candidates` → `distance_tests` → `accepted_pairs` | 1.52B → 15.5M → 17.5k | 61.7k → 632 → 0.71 | 70.5k → 751 → 10 | Self PT has the same candidate-pressure shape at lower scale. |
| `pair_build.external_ee.hash_candidates` → `distance_tests` → `accepted_pairs` | 1.91B → 247.7M → 86.3M | 77.6k → 10.1k → 3512 | 90.6k → 15.8k → 7496 | External EE dominates after obstacle overlap and is now the main barrier-assembly input. |
| `max_step.external.overlapping_obstacles` | 0 / 4701 samples | 0 | 0 | External CCD global AABB early-out fires for every max-step call in this run. |
| `max_step.self_ee.hash_candidates` → `ccd_tests` | 644.8M → 5.27M | 137k → 1121 | 150k → 2907 | Remaining CCD cost is self swept EE. |
| `max_step.self_pt.hash_candidates` → `ccd_tests` | 228.8M → 593k | 48.7k → 126 | 53.3k → 392 | Self swept PT is secondary. |

Barrier pair counters from #10:

| Counter | Total pairs processed | Avg / combined call | Max / call | Interpretation |
|---------|-----------------------|---------------------|------------|----------------|
| `active_set_combined.external_ee.pairs` | 23.05M | 3452 | 7496 | Largest active-set kernel workload; matches 51.47s external EE barrier timing. |
| `active_set_combined.external_pt.pairs` | 3.99M | 598 | 1328 | Secondary external barrier workload. |
| `active_set_combined.external_tp.pairs` | 2.91M | 436 | 1034 | Smaller than PT by count but still 9.61s. |
| `active_set_combined.self_ee.pairs` | 9.86k | 1.48 | 33 | Self barrier assembly is not a meaningful target in this scene. |
| `active_set_combined.self_pt.pairs` | 4.60k | 0.69 | 10 | Self barrier assembly is dwarfed by external pairs. |

## Full 2000-Step Broad-Only Overlap-Query Profile (2026-05-18)

Config: temporary copy of `box-ipc.json` with `profiling=true` and output redirected to `/private/tmp/libpgo-overlap-query-full`; 2000 timesteps, `dump-interval=10`. Final retained code includes #5-#11, with #11 applied only to active-set broad phase. Wall-clock log timestamps were 10:26:22.564 → 10:33:02.268, about 400s.

| Measurement | #5-#10 retained code | #11 broad-only | Notes |
|-------------|----------------------|----------------|-------|
| `contact.surface.build_active_set` | 197.22s / 24560 calls (8.03ms/call) | 196.03s / 25054 calls (7.82ms/call) | Call count drifted upward due line-search/Newton path noise, so per-call time is the safer comparison. |
| `contact.surface.pair_build.static` | 124.57s / 24560 calls (5.07ms/call) | 124.28s / 25054 calls (4.96ms/call) | Modest self broad-phase per-call improvement. |
| `contact.surface.pair_build.external` | 72.54s / 24560 calls (2.95ms/call) | 71.64s / 25054 calls (2.86ms/call) | Modest external broad-phase per-call improvement after obstacle overlap. |
| `contact.surface.pair_build.external_aabb` | 4.82s / 46575 scoped AABB blocks | 5.34s / 47563 scoped AABB blocks | More overlapping calls in this run; not attributed to #11. |
| `contact.adapter.max_step` | 21.87s / 4701 calls | 22.63s / 4727 calls | #11 is not applied to max-step in retained code; this is run-to-run/call-count variation. |
| `contact.surface.active_set_combined` | 82.91s / 6676 calls | 78.54s / 6704 calls | Barrier code is unchanged by #11; treat this as run-to-run variation. |
| `solver.linear_solve` | 6.48s / 4701 calls | 6.72s / 4727 calls | Still not a bottleneck. |

Candidate counters from the #11 broad-only full run:

| Counter | Total | Avg / sample | Max / sample | Interpretation |
|---------|-------|--------------|--------------|----------------|
| `pair_build.external.overlapping_obstacles` | 22509 / 25054 samples | 89.8% overlapping | 1 | Same shape as #5-#10: static external early-out is mainly an early-phase win. |
| `pair_build.self_ee.hash_candidates` → `distance_tests` → `accepted_pairs` | 4.28B → 129.2M → 38.5k | 171k → 5155 → 1.54 | 193k → 5592 → 33 | #11 preserves raw hash-candidate counters; it reduces returned candidate-vector pressure, not raw cell overlap. |
| `pair_build.self_pt.hash_candidates` → `distance_tests` → `accepted_pairs` | 1.55B → 15.8M → 18.3k | 61.7k → 632 → 0.73 | 70.5k → 751 → 10 | Candidate pressure remains fundamentally unchanged. |
| `pair_build.external_ee.hash_candidates` → `distance_tests` → `accepted_pairs` | 1.95B → 250.7M → 87.2M | 77.8k → 10.0k → 3481 | 90.6k → 15.8k → 7494 | External EE remains the largest active-pair source after obstacle overlap. |
| `max_step.external.overlapping_obstacles` | 0 / 4727 samples | 0 | 0 | External CCD remains fully skipped by global AABB early-out. |
| `active_set_combined.external_ee.pairs` | 23.10M | 3446 | 7494 | Barrier assembly target is unchanged: external EE still dominates. |

## Full 2000-Step Packed-Hash Profile (2026-05-18)

Config: temporary copy of `box-ipc.json` with `profiling=true` and output redirected to `/private/tmp/libpgo-packed-hash-full`; 2000 timesteps, `dump-interval=10`. Final retained code includes #5-#12. Wall-clock log timestamps were 10:53:11.448 → 10:59:36.294, about 385s.

| Measurement | #11 broad-only | #12 packed key | Notes |
|-------------|----------------|----------------|-------|
| `contact.surface.build_active_set` | 196.03s / 25054 calls (7.82ms/call) | 189.72s / 25290 calls (7.50ms/call) | Per-call broad phase improved despite a slightly higher call count. |
| `contact.surface.pair_build.static` | 124.28s / 25054 calls (4.96ms/call) | 117.87s / 25290 calls (4.66ms/call) | The main retained win from the packed key. |
| `contact.surface.pair_build.external` | 71.64s / 25054 calls (2.86ms/call) | 71.72s / 25290 calls (2.84ms/call) | Essentially neutral/slightly better per call. |
| `contact.adapter.max_step` | 22.63s / 4727 calls (4.79ms/call) | 23.73s / 4735 calls (5.01ms/call) | Subsections are near-neutral; treat this as run/call-count noise rather than a max-step win. |
| `contact.surface.active_set_combined` | 78.54s / 6704 calls | 63.88s / 6710 calls | Barrier code was not changed by #12; do not attribute this full reduction to the hash key. |
| `solver.linear_solve` | 6.72s / 4727 calls | 8.36s / 4735 calls | Still not a target; run-to-run variation. |

Candidate counters from the #12 packed-key full run:

| Counter | #11 broad-only | #12 packed key | Interpretation |
|---------|----------------|----------------|----------------|
| `pair_build.self_ee.hash_candidates` → `distance_tests` → `accepted_pairs` | 4.28B → 129.2M → 38.5k | 3.45B → 130.3M → 33.9k | Raw self EE cell enumeration drops about 19%, while exact tests stay at the same scale. |
| `pair_build.self_pt.hash_candidates` → `distance_tests` → `accepted_pairs` | 1.55B → 15.8M → 18.3k | 1.29B → 16.0M → 16.0k | Raw self PT cell enumeration drops about 16%. |
| `pair_build.external_ee.hash_candidates` → `distance_tests` → `accepted_pairs` | 1.95B → 250.7M → 87.2M | 1.39B → 255.4M → 88.9M | External EE raw hash candidates drop about 29%; accepted pairs drift with the Newton path. |
| `max_step.self_ee.hash_candidates` → `ccd_tests` | 648.3M → 5.29M | 533.8M → 5.29M | Remaining CCD work still comes from self swept EE/PT; packed key reduces raw enumeration but not CCD test count. |
| `max_step.self_pt.hash_candidates` → `ccd_tests` | 230.1M → 595k | 198.1M → 595k | Same pattern at lower scale. |
| `active_set_combined.external_ee.pairs` | 23.10M | 23.17M | Barrier pair volume is effectively unchanged. |

## Full 2000-Step Query-Side Extraction-Hoist Profile (2026-05-18)

Config: temporary copy of `box-ipc.json` with `profiling=true` and output redirected to `/private/tmp/libpgo-query-primitive-cache-full`; 2000 timesteps, `dump-interval=10`. Final retained code includes #5-#13. The run started around 18:42:40 and finished its profile summary at 18:45:27.407, about 167s.

This full run is much faster than the prior full packed-hash run even in sections untouched by #13, so the 20-step same-environment A/B above is the safer attribution evidence for the extraction hoist. The full run still records the retained-code state after #13.

| Measurement | #12 packed key | #13 query-side extraction hoist | Notes |
|-------------|----------------|---------------------------------|-------|
| `contact.surface.build_active_set` | 189.72s / 25290 calls (7.50ms/call) | 79.87s / 24003 calls (3.33ms/call) | Lower retained profile, but stronger than the short A/B and likely includes host/load/path variation. |
| `contact.surface.pair_build.static` | 117.87s / 25290 calls (4.66ms/call) | 51.14s / 24003 calls (2.13ms/call) | Query-side coordinate loads are hoisted here; use the short same-env A/B for attribution. |
| `contact.surface.pair_build.external` | 71.72s / 25290 calls (2.84ms/call) | 28.67s / 24003 calls (1.19ms/call) | External broad phase also uses the hoist, but the full-run delta is larger than expected from that local change alone. |
| `contact.adapter.max_step` | 23.73s / 4735 calls (5.01ms/call) | 11.18s / 4680 calls (2.39ms/call) | Self CCD loops use the same hoist; this scene still skips all external CCD calls. |
| `contact.surface.active_set_combined` | 63.88s / 6710 calls | 28.49s / 6659 calls | Barrier code was not changed by #13; treat this as run-to-run/path variation. |
| `solver.linear_solve` | 8.36s / 4735 calls | 5.61s / 4680 calls | Still not a target. |

Selected #13 full-run breakdown:

| Section | Total / calls | Per call |
|---------|---------------|----------|
| `contact.surface.pair_build.self_ee_query` | 12.36s / 24003 | 0.515ms |
| `contact.surface.pair_build.self_pt_query` | 5.96s / 24003 | 0.248ms |
| `contact.surface.pair_build.external_ee` | 7.22s / 21458 | 0.337ms |
| `contact.surface.pair_build.external_pt` | 2.46s / 21458 | 0.115ms |
| `contact.surface.pair_build.external_tp` | 15.95s / 21458 | 0.743ms |
| `contact.surface.max_step_ee` | 5.53s / 4680 | 1.18ms |
| `contact.surface.max_step_pt` | 4.06s / 4680 | 0.867ms |

Candidate counters from the #13 full run:

| Counter | #12 packed key | #13 query-side extraction hoist | Interpretation |
|---------|----------------|---------------------------------|----------------|
| `pair_build.self_ee.hash_candidates` → `distance_tests` → `accepted_pairs` | 3.45B → 130.3M → 33.9k | 3.28B → 123.6M → 33.3k | Same structural pressure; totals drift with call count/path. |
| `pair_build.self_pt.hash_candidates` → `distance_tests` → `accepted_pairs` | 1.29B → 16.0M → 16.0k | 1.23B → 15.2M → 15.3k | Same shape at lower scale. |
| `pair_build.external_ee.hash_candidates` → `distance_tests` → `accepted_pairs` | 1.39B → 255.4M → 88.9M | 1.31B → 241.8M → 84.2M | External EE remains the largest active-pair source after obstacle overlap. |
| `max_step.self_ee.hash_candidates` → `ccd_tests` | 533.8M → 5.29M | 527.6M → 5.25M | CCD test volume is unchanged structurally; #13 reduces repeated query-side loads. |
| `max_step.self_pt.hash_candidates` → `ccd_tests` | 198.1M → 595k | 195.8M → 591k | Same pattern at lower scale. |
| `active_set_combined.external_ee.pairs` | 23.17M | 23.00M | Barrier pair volume remains effectively unchanged. |

Rejected attempt:

- Self broad phase one-sided AABB inflation (`target` tight, `query` inflated by `dhat`) was tested and reverted. On the same 20-step smoke, `build_active_set` regressed from 0.894s to 1.041s because the extra edge-query AABB construction was not repaid in the no-contact phase. The brute-force self broad-phase regression test was kept because it guards future broad-phase tightening attempts.
- Self broad phase tight-AABB lower-bound filtering before exact PT/EE distance was tested and reverted. It did not reliably improve the 20-step smoke; one comparable run regressed from 0.692s to 0.736s because extra tight AABB construction outweighed the skipped distance work.
- Separate self PT/EE cell-size selection (`triBox` average for PT hash, `edgeBox` average for EE hash) was tested and reverted. It did not beat the retained implementation: one experiment run measured `self_ee_query=0.332s`, `build_active_set=0.781s`, and `contact.adapter.max_step=0.107s`, with no clear improvement over the retained-code smoke range.
- Thread-local external combined gradient accumulation was tested and reverted. Hypothesis: replace atomic gradient scatter inside `computeExternalAll()` with TBB thread-local dense gradients, then reduce once after PT/TP/EE loops. Full 2000-step profile did not improve: `active_set_combined.external` was 82.72s / 6715 calls versus the retained 82.43s / 6676-call reference, while `external_ee` was effectively unchanged at 51.46s. The added TLS setup/reduction was not repaid.
- Applying #11's overlap-filtered query API to CCD max-step was tested and narrowed back out. Hypothesis: the same same-cell-but-non-overlapping candidate suppression would reduce self swept PT/EE candidate-vector pressure. Result: focused tests passed, but the 20-step smoke moved `contact.adapter.max_step` to 0.102-0.116s versus the retained/broad-only 0.093-0.095s range, and a full run measured 23.32s / 4685 calls versus the #5-#10 retained 21.87s / 4701-call reference. Max-step therefore keeps the old query path.
- Exact collision-safe grid-cell keys were tested and rejected as a performance shape. A `GridCoord` unordered-map key and a hash-bucket-plus-coordinate-equality structure both removed legacy spatial-hash collisions, but 20-step smoke timing regressed or stayed above the retained broad-only range because lookup/key overhead outweighed the candidate reduction. The retained #12 uses a cheap packed 64-bit key instead.
- Sorted bulk hash build was tested and rejected. Hypothesis: collect `(key, primitiveId)` references, sort by key, and fill cell vectors in groups to reduce `unordered_map` and vector growth overhead. Result: the 20-step smoke regressed; `self_ee_hash_insert` rose from the packed-key range of about 0.10-0.12s to 0.17-0.19s, `self_pt_hash_insert` rose from about 0.10-0.11s to 0.20-0.21s, and `build_active_set` moved to 0.75-0.79s.
- Visitor/callback overlap queries were tested and reverted. Hypothesis: avoid materializing a temporary candidate vector by invoking distance tests directly while scanning overlapping hash candidates. Short smoke was mixed, and the full run regressed on the safer per-call comparison: `pair_build.static` was 118.05s / 24160 calls (4.89ms/call) versus #12's 117.87s / 25290 calls (4.66ms/call), with `self_ee_query` 1.86ms/call versus #12's 1.68ms/call. The lower total full time came from call-count/path drift, not a retained per-call win.
- Per-call `VXd` to `V3d` vertex/velocity caches in broad phase and CCD max-step were tested and reverted. Hypothesis: cache all surface vertices and displacements once per call so candidate-side loads also avoid repeated `segment<3>()`. Result: focused correctness tests passed, and candidate counters were unchanged, but the 20-step smoke did not improve. Baseline `/private/tmp/libpgo-vertex-cache-baseline-smoke.log` measured `build_active_set=0.3153s`, `self_ee_query=0.0789s`, `self_pt_query=0.0367s`, `adapter.max_step=0.0511s`; the best sequential retained sample `/private/tmp/libpgo-vertex-cache-retained-smoke-4.log` measured `build_active_set=0.3150s`, `self_ee_query=0.0816s`, `self_pt_query=0.0357s`, `adapter.max_step=0.0507s`. The fixed cache-build cost was not repaid by the candidate-side load reduction.
- Persistent obstacle-pose vertex caching was tested and reverted. Hypothesis: store sampled obstacle `V3d` vertices in `ObstaclePoseCache`, then use that cache in external broad phase, external max-step, and external barrier assembly. The 20-step smoke was neutral because the obstacle never overlapped (`/private/tmp/libpgo-obstacle-vertex-cache-smoke.log`: `build_active_set=0.3173s / 140`, `pair_build.external=0.0053s / 140`). The full 2000-step run regressed relative to #13 despite similar pair volume: `/private/tmp/libpgo-obstacle-vertex-cache-full.log` measured `build_active_set=90.06s / 24555` versus #13's `79.87s / 24003`, `pair_build.external=32.44s / 24555` versus `28.67s / 24003`, and `active_set_combined.external=32.13s / 6697` versus `28.21s / 6659`. External EE barrier pairs were only about 0.7% higher, so this was not a credible retained win. Production code was reverted; the surface-bounds cache test was kept.

## 600-Step External Dynamic-Block A/B (2026-05-19)

Config: `box-with-sphere-lite` with `num-timestep=600`, `dump-interval=1000`, `profiling=true`, absolute mesh paths, and output under `/private/tmp`. This shorter run reaches the external contact phase and is used only for same-environment attribution.

| Measurement | Baseline before #14 | #14 retained | Notes |
|-------------|---------------------|--------------|-------|
| `contact.surface.active_set_combined.external` | 14.45s / 1801 calls | 13.21s / 1799 calls | Modest retained win in the target external barrier path. |
| `contact.surface.active_set_combined.external_ee` | 7.13s / 2.29M pairs | 6.74s / 2.28M pairs | Per-pair time improved from about 3.12us to 2.96us. |
| `contact.surface.active_set_combined.external_pt` | 3.07s / 400k pairs | 2.77s / 398k pairs | Same exact-subblock wrapper path for dynamic triangle blocks. |
| `contact.surface.active_set_combined.external_tp` | 2.09s / 291k pairs | 2.01s / 289k pairs | Slight improvement. |

Rejected follow-up attempts:

- Direct external dynamic-block assembly was tested after #14. It preserved the generic 12-DOF PSD-projected dynamic sub-block semantics but regressed the target profile: `external_ee=10.51s / 2.29M pairs`, `active_set_combined.external=16.52s`.
- Sorting external EE pairs by dynamic edge before barrier assembly was tested and reverted. It preserved pair-set correctness, but the 600-step profile regressed to `active_set_combined.external=17.08s` and `external_ee=10.72s / 2.28M pairs`.

## 600-Step Backtracking Initial-Energy Reuse (2026-05-19)

Config: same 600-step `box-with-sphere-lite` profiling setup as the external dynamic-block A/B, with output under `/private/tmp/libpgo-backtracking-reuse-600`. This experiment keeps #14 and changes only the default backtracking line-search path to reuse the already-computed alpha=1 trial energy.

| Measurement | #14 retained profile | #15 retained | Notes |
|-------------|----------------------|--------------|-------|
| `contact.surface.energy` | 46.07s / 3024 calls | 25.43s / 1800 calls | Removes the duplicate alpha=1 `func()` call that immediately followed Newton's pre-line-search full-step energy check. |
| `contact.surface.build_active_set` | 87.57s / 6023 calls | 63.91s / 4798 calls | One less active-set build per Newton iteration in the default backtracking path. |
| `contact.adapter.func` | not listed in #14 table | 25.70s / 1800 calls | Tracks the same reduced line-search/function-evaluation path at the adapter level. |
| `solver.linear_solve` | 2.76s / 1199 calls | 2.71s / 1198 calls | Newton path is effectively unchanged. |
| `active_set_combined.external_ee.pairs` | 2.275M | 2.275M | Active contact workload is unchanged; this optimization reduces duplicate evaluations, not pair generation. |

Correctness guard:

- `NewtonSolver_gtest.BacktrackingReusesInitialTrialEnergy` verifies a one-iteration quadratic solve now performs two `func()` calls instead of the old three-call sequence.

## 600-Step Newton Standalone-Gradient Elimination (2026-05-19)

Config: same 600-step `box-with-sphere-lite` profiling setup, with output under `/private/tmp/libpgo-residual-reuse-600` and stdout profile log at `/private/tmp/libpgo-residual-reuse-600.log`. This experiment keeps #14/#15 and removes two standalone gradient-only evaluations from the default implicit-Euler Newton path.

| Measurement | #15 retained | #16 retained | Notes |
|-------------|--------------|--------------|-------|
| `contact.surface.gradient` | 15.30s / 1200 calls | 0 calls | Initial Newton convergence scale now uses the first fused `func_grad_hessian()` gradient, and timestep residual printing reuses Newton's final gradient diagnostics. |
| `contact.adapter.gradient` | present with standalone gradient evaluations | 0 calls | The intermediate deferred-initial run still had 600 residual-print calls / 8.24s; residual reuse removes that final adapter gradient source. |
| `contact.surface.build_active_set` | 63.91s / 4798 calls | 35.02s / 3598 calls | The remaining builds are 1800 energy-only line-search calls plus 1798 fused Hessian calls. |
| `contact.surface.energy` | 25.43s / 1800 calls | 18.23s / 1800 calls | Call count is unchanged from #15; timing variation comes from host/path noise. |
| `contact.adapter.hessian_direct` | not listed in #15 table | 36.70s / 1798 calls | Tracks fused Newton evaluations after duplicate gradient-only work is removed. |
| `solver.linear_solve` | 2.71s / 1198 calls | 1.55s / 1198 calls | Solver iteration count is unchanged. Timing is not attributed to #16. |
| `active_set_combined.external_ee.pairs` | 2.275M | 2.275M | Active contact workload is unchanged; #16 only removes duplicate gradient evaluations. |

Correctness guards:

- `NewtonSolver_gtest.BacktrackingReusesInitialTrialEnergy` also verifies the one-iteration backtracking solve performs one fused gradient path and no separate pre-loop gradient call.
- `ImplicitBackwardEulerTimeIntegratorGTest.ResidualPrintReusesNewtonFinalGradient` first failed with two model `gradient()` calls, then passed after residual printing reused Newton's final gradient stats.
- The same test suite now covers `TRBDF2TimeIntegratorEnergy::func_grad_hessian()` for non-fixed-topology models, fixing a verification-time crash from the previous default `gradient()+hessian()+func()` fallback.

## 600-Step Cached Energy-Component Printing (2026-05-19)

Config: same 600-step `box-with-sphere-lite` profiling setup, with output under `/private/tmp/libpgo-energy-component-cache-nocopy-600` and stdout profile log at `/private/tmp/libpgo-energy-component-cache-nocopy-600.log`. This experiment keeps #14-#16 and avoids recomputing IPC/material energy solely for the `Energy components:` diagnostic print after a converged implicit-Euler timestep.

| Measurement | #16 retained | #17 retained | Notes |
|-------------|--------------|--------------|-------|
| Wall-clock log timestamps | 18:09:41.780 -> 18:11:10.164 (~88.4s) | 18:30:11.894 -> 18:31:27.102 (~75.2s) | Same 600-step config; wall-clock is still noisy, but this run also has cleaner call-count evidence. |
| `contact.surface.energy` | 18.23s / 1800 calls | 11.42s / 1206 calls | Removes the per-timestep energy-component print pass for all fully converged Newton solves. |
| `contact.adapter.func` | 18.36s / 1800 calls | 11.51s / 1206 calls | Adapter-level count tracks the same removed diagnostic energy evaluations. |
| `contact.surface.build_active_set` | 35.02s / 3598 calls | 27.86s / 3005 calls | Remaining calls are line-search energy builds plus fused Newton builds. |
| `material.energy` | 8.01s / 3598 calls | 5.58s / 3005 calls | Material component printing also reuses the cached values. |
| `solver.linear_solve` | 1.55s / 1198 calls | 1.53s / 1199 calls | Solver path is effectively unchanged. |
| `active_set_combined.external_ee.pairs` | 2.275M | 2.275M | Active contact workload is unchanged; this optimization only removes diagnostic recomputation. |

Rejected implementation detail:

- The first cache version copied the full DOF vector during each fused Newton evaluation and checked exact equality before printing. It reduced call counts, but the 600-step wall-clock and active-set timings were worse in that run (`build_active_set=40.62s / 3224 calls`). The retained version uses `SolveDiagnostics::hasFinalGradientStats` as the validity signal instead, avoiding the large vector copy while still clearing the cache before every solve.

Correctness guard:

- `ImplicitBackwardEulerTimeIntegratorGTest.ResidualPrintReusesNewtonFinalGradient` now verifies both residual and energy-component printing reuse Newton's final evaluation: the test failed at two model `func()` calls before #17 and passes with one call while still printing `sub 0`.

## 600-Step Accepted Line-Search Active-Set Cache (2026-05-19)

Config: same 600-step `box-with-sphere-lite` profiling setup, with output under `/private/tmp/libpgo-active-set-cache-profiled-600` and stdout profile log at `/private/tmp/libpgo-active-set-cache-profiled-600/runIPCSim.log`. This experiment keeps #14-#17 and caches the `SurfaceIPCActiveSet` from an energy-only line-search evaluation for the next fused Newton evaluation when the surface positions compare exactly equal.

| Measurement | #17 retained | #18 retained | Notes |
|-------------|--------------|--------------|-------|
| Wall-clock log timestamps | 18:30:11.894 -> 18:31:27.102 (~75.2s) | 20:44:56.552 -> 20:46:24.376 (~87.8s) | The #18 run overlapped other long-running simulations on the machine, so wall-clock is recorded but not used as direct attribution. |
| `contact.surface.build_active_set` | 27.86s / 3005 calls | 26.27s / 1837 calls | The remaining builds are the 600 first fused evaluations for each timestep plus real line-search energy evaluations. Fused evaluations immediately after an accepted line-search `func()` consume the cached active set instead of rebuilding it. |
| `contact.adapter.hessian_direct` | 35.40s / 1799 calls | 28.17s / 1801 calls | Fused Newton evaluation count is essentially unchanged; the reduction comes from removing nested active-set builds on cache hits. |
| `contact.surface.energy` | 11.42s / 1206 calls | 19.09s / 1237 calls | Energy-only evaluations are still actual line-search trials. This run had slightly more line-search evaluations and higher per-call timing, consistent with the noisy host state. |
| `material.energy` | 5.58s / 3005 calls | 7.58s / 3038 calls | Material evaluation count tracks fused + energy-only solver evaluations; this optimization only targets IPC active-set rebuilds. |
| `solver.linear_solve` | 1.53s / 1199 calls | 2.66s / 1201 calls | Solver path is not targeted. |
| `active_set_combined.external_ee.pairs` | 2.275M | 2.281M | Active contact workload stays the same order; the cache changes active-set construction reuse, not contact semantics. |

Correctness and profiling guards:

- `EmbeddedSurfaceIPCPotentialEnergyGTest.EnergyOnlyEvaluationSeedsNextCombinedActiveSet` first failed with two active-set builds for `func(u); func_grad_hessian(u)`, then passed with one build, one `contact.surface.energy` profile section, one `active_set_energy`, and one `active_set_combined`.
- The cache is exact-position only and is cleared when obstacle time/static state changes. Separate `gradient()` / `hessian()` calls still build independent active sets, preserving the existing standalone evaluation behavior.
- `NewtonSolverGTest.StepTooSmallConvergenceRecordsFinalGradientStats` covers the loose-convergence `dx too small` branch so residual printing does not reintroduce a standalone IPC `gradient()` after that branch. The final 600-step #18 profile has no `contact.adapter.gradient` or `contact.surface.gradient` entries.

## 600-Step Coplanar Obstacle-Edge Filtering (2026-05-20)

Config: same 600-step `box-with-sphere-lite` profiling setup, with output under `/private/tmp/libpgo-feature-edge-600` and stdout profile log at `/private/tmp/libpgo-feature-edge-600/runIPCSim.log`. This experiment keeps #14-#18 and changes only the obstacle edge set used for external EE: `ObstacleSurface::contactEdges()` contains boundary, nonmanifold, sharp, and degenerate obstacle edges, while coplanar two-triangle interior edges stay available in `uniqueEdges()` but are not hashed for external EE.

Motivation from the profiled obstacle mesh (`bottom.1.obj`): 4162 vertices, 8320 faces, and 12480 unique topological edges. Normal classification found 12160 coplanar interior edges and 320 sharp 90-degree feature edges. For this flat box obstacle, almost all previous external EE work was generated by triangulation diagonals rather than geometric features.

| Measurement | #18 retained | #19 retained | Notes |
|-------------|--------------|--------------|-------|
| `contact.surface.active_set_combined.external_ee` | 8.124s / 1801 calls / 2.281M pairs | 0.000098s / 1731 calls / 0 pairs | The flat obstacle has no retained contact edges close enough for EE barriers after coplanar interior edges are removed. |
| `contact.surface.pair_build.external_ee` | 2.874s / 1101 calls / 2.325M accepted | 0.535s / 967 calls / 0 accepted | Remaining time is the cheap empty-edge path for overlapping-obstacle calls. |
| `pair_build.external_ee.hash_candidates` → `distance_tests` | 68.120M → 6.664M | 0 → 0 | Candidate pressure is eliminated for external EE in this scene. |
| `contact.surface.active_set_combined.external` | 15.288s / 1801 calls | 5.383s / 1731 calls | External PT/TP remain; EE is no longer the dominant external barrier section. |
| `contact.surface.pair_build.external` | 7.470s / 1837 calls | 2.636s / 1733 calls | External PT/TP still run for overlapping obstacle calls. |
| `contact.surface.build_active_set` | 26.271s / 1837 calls | 14.579s / 1733 calls | Call-count and Newton-path drift exist, but the external EE counters explain the large local reduction. |
| `contact.adapter.hessian_direct` | 28.168s / 1801 calls | 19.920s / 1731 calls | Includes a noisy 7.21s max pullback-hessian outlier, so use section counters for attribution. |
| `solver.linear_solve` | 2.664s / 1201 calls | 1.342s / 1131 calls | Solver path drifted with the altered contact model; this is not a direct solver optimization. |

Convergence and counters:

- `rg -c "solverRet=Converged"` on the feature-edge log reports 600 converged timesteps; T599 ends with `solverRet=Converged`, `residual=0.159697`, `accepted=true`.
- Failure-pattern search for `LineSearchFailed`, `StepTooSmall`, `NonFinite`, `Line search failed`, `dx too small`, `solverRet=Failed`, and `accepted=false` returned no matches.
- External EE barrier pairs are 0 total and max 0; external PT and TP still contribute 472875 and 348427 combined pairs, respectively.

Correctness guards:

- The first `ObstacleSurface::contactEdges()` tests were written before production code and failed to compile because the API did not exist.
- `ObstacleSurfaceKeepsOnlyFeatureEdgesForExternalEECache` checks that a two-triangle coplanar patch drops the interior diagonal but keeps the boundary edges.
- `ObstacleSurfaceKeepsSharpInteriorEdgesForExternalEECache` checks that a non-coplanar interior edge remains in the external EE cache.
- `ExternalEEDoesNotUseCoplanarInteriorObstacleDiagonal` verifies external broad phase does not generate an EE pair solely from a coplanar obstacle diagonal.
- The filter is recomputed on obstacle pose updates; boundary, nonmanifold, and degenerate edges are retained conservatively.

Rejected follow-up:

- Topology-aware predicate filtering inside `SpatialHashGrid::queryOverlapping*()` was tested and reverted. Hypothesis: filter incident self PT triangles and adjacent self EE edges before appending to the hot candidate vectors, while preserving raw hash-candidate counters. Focused tests passed, but the same 600-step profile regressed: `self_ee_query` moved from 5.754s / 1733 calls to 6.140s / 1740 calls, `self_pt_query` from 2.278s to 2.301s, and `pair_build.static` from 11.938s to 12.078s. Candidate and exact-test totals stayed at the same scale, so the per-raw-candidate predicate overhead was not repaid. Production code and the temporary query API were reverted.
- External EE contact-edge union AABB early-out was tested and reverted. Hypothesis: cache a union AABB for each obstacle's retained contact edges and skip the external EE query when the dynamic edge union misses it. Focused tests passed, but the same 600-step profile regressed: `pair_build.external_ee` moved from 0.535s / 967 calls to 1.149s / 965 calls, `pair_build.external` from 2.636s to 5.638s, and `build_active_set` from 14.579s / 1733 calls to 24.178s / 1731 calls. External EE hash, distance, and accepted counters stayed at zero, so the union boxes were too coarse to skip this scene and the extra union/cache work was not repaid. Production code and the temporary bounds test were reverted.
- External max-step direct swept-surface union was tested and reverted. Hypothesis: compute only a dynamic swept-surface union box first, then build per-vertex swept boxes only if an obstacle surface overlaps it; the #19 profile has zero external max-step obstacle overlaps. Focused max-step tests passed and the 600-step run converged, but the profile regressed: `contact.adapter.max_step` moved from 7.457s / 1131 calls to 14.492s / 1135 calls, `max_step_ee` from 4.320s to 7.541s, `max_step_pt` from 2.004s to 4.424s, and `pair_build.swept` from 0.839s to 1.888s. External max-step counters stayed zero, so the direct-union path did not produce a retained no-overlap win. Production code was reverted.
- Cached obstacle-vertex hashing for external TP was tested and reverted. Hypothesis: cache an obstacle vertex hash in `ObstaclePoseCache` and query dynamic triangle boxes against it, avoiding the per-overlap dynamic triangle hash rebuild in external TP broad phase. The cache test failed first, then focused core/external broad-phase tests passed, and the 600-step run converged. It did not improve the target path: `pair_build.external_tp` moved from 1.152s / 967 calls to 1.470s / 1021 calls, `pair_build.external` from 2.636s to 4.522s, and `build_active_set` from 14.579s / 1733 calls to 25.041s / 1787 calls. The inverted query shape was not a retained win, and production code plus the temporary cache test were reverted.
- Identity embedding sparse-pullback bypass in `MappedSurfacePotentialEnergy` was tested and reverted. Hypothesis: detect an identity `surfaceFromSimulationDispMap` and skip sparse map/pullback products. A red profiling test first confirmed identity adapters still recorded pullback sections, and focused adapter tests passed after implementation. The target 600-step run is phase2 volume IPC with a barycentric embedding, so the identity fast path was not exercised: `pullback_gradient` and `pullback_hessian` still recorded 1731 calls. The profile regressed (`adapter.func` 9.986s -> 22.854s, `adapter.max_step` 7.457s -> 18.251s, `build_active_set` 14.579s -> 32.517s), so production and test changes were reverted.
- Direct `Eigen::Ref` gradient output for combined IPC assembly was tested and reverted. Hypothesis: let `SurfaceIPCCore::computeAll()` and the combined self/external barrier helpers write directly into the adapter's `surfaceGradient` ref, avoiding a temporary `VectorXd` and copy in `EmbeddedSurfaceIPCPotentialEnergy::computeSurfaceAll()`. A compile-red core test first proved the old API could not accept a gradient ref, and focused core/adapter/barrier tests passed after implementation. The target 600-step profile at `/private/tmp/libpgo-ref-gradient-run.log` converged but regressed heavily: `build_active_set` moved from 14.579s / 1733 calls to 38.858s / 2015 calls, `adapter.hessian_direct` from 19.920s / 1731 calls to 27.082s / 1742 calls, and `adapter.max_step` from 7.457s / 1131 calls to 17.558s / 1144 calls. A fresh post-revert retained-code run under the same host state (`/private/tmp/libpgo-post-revert-baseline-run.log`) was also noisy but still lower at `build_active_set=29.509s / 1877`, `adapter.hessian_direct=21.193s / 1735`, and `adapter.max_step=14.620s / 1136`. Production and test changes were reverted.

## 600-Step Line-Search Active-Set Superset Reuse (2026-05-20)

Config: same 600-step `box-with-sphere-lite` profiling setup, with output under `/private/tmp/libpgo-line-search-superset-600` and stdout profile log at `/private/tmp/libpgo-line-search-superset-run.log`. This experiment keeps #14-#19 and adds a line-search lifecycle hook to `PotentialEnergy`. IPC builds one conservative active-set superset for `x + alpha * dx`, `alpha in [0, 1]`, using swept dynamic AABBs inflated by `dhat` / `dhat_external`; each trial energy then updates only the active-set positions. The Newton integration enables the hook for backtracking/simple line search only; golden/Brent can expand their bracket past `alpha=1`, so they keep the exact per-alpha path.

Same-host A/B uses `/private/tmp/libpgo-post-revert-baseline-run.log` as the baseline because earlier #19 timings were taken under a different host load.

| Measurement | Fresh retained baseline | #20 retained | Notes |
|-------------|-------------------------|--------------|-------|
| `contact.adapter.func` | 21.083s / 1277 calls | 15.092s / 1134 calls | Fewer line-search IPC `func()` evaluations rebuild exact active sets. |
| `contact.surface.energy` | 20.857s / 1277 calls | 14.884s / 1134 calls | Superset trial energies reuse the same pair list; barrier kernels return zero for pairs outside `dhat`. |
| `contact.surface.build_active_set` | 29.509s / 1877 calls | 21.951s / 1734 calls | Main target. Superset construction is still counted as `build_active_set`, but replaces multiple exact trial builds. |
| `contact.surface.pair_build.static` | 22.706s / 1877 calls | 17.166s / 1734 calls | Static self broad phase drops with the reduced active-set build count. |
| `contact.surface.pair_build.external` | 6.770s / 1877 calls | 4.769s / 1734 calls | External PT/TP still run when the obstacle overlaps; external EE remains zero after #19. |
| `contact.adapter.hessian_direct` | 21.193s / 1735 calls | 18.538s / 1731 calls | Fused Newton evaluations can consume the accepted trial's superset if it was the last evaluated state. |
| `contact.surface.active_set_combined` | 6.825s / 1735 calls | 6.308s / 1731 calls | Barrier assembly did not regress from the extra inactive superset pairs in this scene. |
| `solver.linear_solve` | 3.194s / 1136 calls | 2.872s / 1131 calls | Solver path is not targeted; call-count drift is recorded for context only. |

Convergence and counters:

- `rg -c "solverRet=Converged"` reports 600 converged timesteps for the #20 run.
- Failure-pattern search for `LineSearchFailed`, `StepTooSmall`, `NonFinite`, `Line search failed`, `dx too small`, `solverRet=Failed`, and `accepted=false` returned no matches on the #20 log.
- External EE remains eliminated for the flat obstacle: `active_set_combined.external_ee.pairs` total 0, max 0. External PT/TP combined pair-list totals are 472875 and 348427.
- Pair-list counters after #20 can include conservative superset pairs on line-search-fed fused evaluations. They remain useful for workload size, but they are no longer always identical to exact `dhat` active-pair counts; the barrier kernels skip inactive `d2 >= dhat^2` pairs.

Correctness guards:

- `EmbeddedSurfaceIPCPotentialEnergyGTest.LineSearchSupersetReusesOneActiveSetAcrossTrialEnergies` first failed to compile before the line-search hooks existed, then passed. It compares three trial energies against exact per-alpha `func()` results and verifies only one active-set build is recorded across the trial sequence plus the accepted fused evaluation.
- `SurfaceIPCCoreGTest.LineSearchActiveSetSupersetContainsExactSampledAlphas` samples multiple alphas on a segment, builds exact active sets at each alpha, checks that exact self/external PT/TP/EE pairs are subsets of the swept superset, and compares superset energy against exact active-set energy.
- The hook is a no-op for ordinary potential energies and is forwarded by `PotentialEnergies` using each child energy's local DOF slice. `MappedSurfacePotentialEnergy` maps the simulation state/step to surface positions/displacements before calling the IPC override.

## Solver Statistics

| Metric | Total | Per Timestep |
|--------|-------|-------------|
| Newton solves (linear solves) | 4679 | 2.34 |
| func_grad_hessian() calls | 6669 | 3.33 |
| func() calls (line search) | 14192 | 7.10 |
| gradient() calls (initial + residual) | 4000 | 2.00 |
| buildActiveSet() calls | 24861 | 12.43 |
| computeMaxStepLimit() calls | 4679 | 2.34 |

## Top-Level Time Breakdown

```
                                 ┌──────────────────────────────────────┐
  buildActiveSet (broad phase)   │██████████████████████████████████████│ 224.0s (57%)
                                 └──────────────────────────────────────┘
                                 ┌──────────────────┐
  Barrier combined (active set)  │██████████████████│  67.0s (17%)
                                 └──────────────────┘
                                 ┌──────────┐
  Material model                 │██████████│  57.0s (14%)
                                 └──────────┘
                                 ┌─────────┐
  CCD max step                   │█████████│  42.6s (11%)
                                 └─────────┘
                                 ┌───┐
  Pullback hessian               │███│  16.5s ( 4%)
                                 └───┘
                                 ┌──┐
  Linear solve (Pardiso)         │██│   8.7s ( 2%)
                                 └──┘
```

> Numbers are from a cold-run baseline (optimization #1) with optimization #3 correction (-6s on active_set_combined).

## buildActiveSet Internal Breakdown (224.0s)

### Self collision pairs (145.9s, 65%)

```
self_ee_query        │██████████████████████████████▊                 │  60.9s (42%)
self_pt_query        │███████████▍                                    │  24.9s (17%)
self_ee_hash_insert  │██████████▌                                     │  23.6s (16%)
self_pt_hash_insert  │██████████                                      │  22.4s (15%)
self_aabb            │███▊                                            │   8.3s ( 6%)
```

### External collision pairs (95.1s, 42%)

```
external_ee          │████████████████▋                                │  37.0s (39%)
external_tp          │████████████████▏                                │  36.0s (38%)
external_pt          │██████                                          │  13.4s (14%)
external_aabb        │███▊                                            │   8.2s ( 9%)
```

### Detail

| Sub-step | Time | Calls | ms/call | Note |
|----------|------|-------|---------|------|
| self_ee_query | 60.9s | 24861 | 2.45 | EE distance for ALL candidate pairs. Dominant cost. |
| self_pt_query | 24.9s | 24861 | 1.00 | PT distance for ALL candidate pairs. |
| self_ee_hash_insert | 23.6s | 24861 | 0.95 | Serial `unordered_map` insert for edge hash. |
| self_pt_hash_insert | 22.4s | 24861 | 0.90 | Serial `unordered_map` insert for tri hash. |
| external_ee | 37.0s | 24861 | 1.49 | Cached obs edge hash query + EE distance. |
| external_tp | 36.0s | 24861 | 1.45 | Rebuilds dyn tri hash (serial) + query. |
| external_pt | 13.4s | 24861 | 0.54 | Cached obs tri hash query + PT distance. |
| self/external aabb | 16.5s | — | — | 6× parallel_for AABB construction. |

## Barrier Computation on Active Set (82.91s)

| Step | Time | Calls | ms/call |
|------|------|-------|---------|
| Combined (energy+grad+hess) | 82.91s | 6676 | 12.4ms |
| Combined external | 82.43s | 6676 | 12.3ms |
| External EE | 51.47s | 6676 | 7.7ms |
| External PT | 13.70s | 6676 | 2.1ms |
| External TP | 9.61s | 6676 | 1.4ms |
| Combined self | 0.48s | 6676 | 0.07ms |
| Self EE | 0.08s | 6676 | 0.01ms |
| Self PT | 0.05s | 6676 | 0.01ms |
| Energy only (line search) | 7.9s | 14192 | 0.56ms |
| Gradient only (residual) | 3.0s | 4000 | 0.74ms |

Combined path uses `computeSelfAll` + `computeExternalAll`; after #10 the full run shows external EE dominates the combined path, while self barrier assembly is negligible.
Barrier kernels call `classifyEE`/`classifyPT` **once** per pair via `computeEESqDistAll`/`computePTSqDistAll`.

## CCD Max Step (42.6s)

| Sub-step | Time |
|----------|------|
| EE swept pairs + CCD | 15.4s |
| PT swept pairs + CCD | 9.1s |
| Adapter overhead + map | 18.1s |

## Material Model (57.0s)

| Sub-step | Time | Calls |
|----------|------|-------|
| Energy | 27.2s | 20861 |
| Hessian | 15.3s | 6670 |
| Gradient | 13.8s | 10669 |
| Inversion CCD max-step | 0.6s | 4699 |

## Pullback / Mapping

| Step | Time | Description |
|------|------|-------------|
| Pullback Hessian | 16.5s | Wᵀ·H_surf·W (sparse triple product) |
| Map sim→surface | 2.9s | W · u_sim = u_surf |
| Pullback Gradient | 0.9s | Wᵀ · g_surf |

## Linear Solve (8.7s)

Pardiso sparse direct solver for 77880×77880 system. 4679 solves, **1.9ms/solve**.

## Optimization History

| Phase | buildActiveSet | active_set | Total (approx) |
|-------|---------------|------------|----------------|
| Original (baseline) | 295.8s | 92.5s (E+G+H separate) | ~460s |
| + func_grad_hessian | 224.0s (-24%) | 72.2s | ~400s (-13%) |
| + classify-once dispatcher | 224.0s | 67.0s (-8%) | ~395s (-14%) |
| **Cumulative** | **-71.8s** | **-25.5s** | **~65s saved** |
