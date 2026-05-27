# IPC Optimization Session Notes

Date: 2026-05-18

## Goal

Reduce IPC wall time along the active-set / broad-phase / CCD max-step path, using `plan/profiling-report.md` as the source profile. Preserve pair-set correctness with focused tests before keeping an optimization.

## Adopted Changes

1. `SpatialHashGrid::build()`
   - Adds bulk hash construction for a vector of AABBs.
   - Precomputes grid ranges once, reserves by estimated cell references, then inserts without recomputing grid coordinates.
   - Replaces repeated `insert()` loops in self broad phase, external TP, obstacle cache, and CCD max-step hashes.
   - Local microbenchmark: repeated `insert()` 5.48s, `build()` 3.88s, 1.41x faster.

2. External obstacle global-AABB early-out
   - `ObstaclePoseCache` now stores an uninflated `surfaceBox`.
   - `buildExternalPairs()` first builds only dynamic vertex AABBs, unions them, filters obstacles by `surfaceBox`, and returns before PT/TP/EE work if no obstacle overlaps.
   - `computeExternalMaxStep()` applies the same early-out before constructing dynamic swept triangle/edge AABBs.
   - Final repeated 20-step no-contact smoke: `contact.surface.pair_build.external` dropped from 0.216s to 0.00485-0.00521s over 140 calls.

3. `SpatialHashGrid::queryAfter()`
   - Adds an upper-triangle query for symmetric self EE pair generation.
   - Self EE broad phase and self EE CCD now avoid collecting candidates with `candidateId <= queryEdgeId`.
   - Repeated 20-step smoke after external early-out was noisy: `self_ee_query` measured 0.286-0.329s over 140 calls around the pre-change 0.311s sample.

4. Candidate-count profiling counters
   - `runIPCSim` now emits `profileCounter` lines alongside timed profile sections when profiling is enabled.
   - Static self/external broad phase records hash candidates, exact distance tests, accepted pairs, and obstacle-overlap counts.
   - Self/external max-step records hash candidates, CCD tests, and obstacle-overlap counts.
   - The counters are diagnostic-only and are gated by `profiling=true`.

5. Hot-path log demotion
   - Per-active-set pair counts and per-Hessian nonzero counts are now debug-level logs instead of info-level logs.
   - The full 2000-step info log no longer contains 25k active-pair lines or 6.7k Hessian lines.
   - 20-step counter smoke improved `contact.surface.build_active_set` from 0.7608s to 0.6932s over 140 calls.

6. Combined barrier assembly profiling split
   - `computeSelfAll()` now records self combined/PT/EE section timings and pair-count counters.
   - `computeExternalAll()` now records external combined/PT/TP/EE section timings and pair-count counters.
   - Full 2000-step profile: `contact.surface.active_set_combined` is 82.91s / 6676 calls; `external` accounts for 82.43s, `self` only 0.48s.
   - External EE is the dominant barrier kernel: 51.47s for 23.05M processed pairs; external PT is 13.70s for 3.99M pairs, and external TP is 9.61s for 2.91M pairs.

7. Active-set broad-phase overlap-filtered hash queries
   - Adds `SpatialHashGrid::queryOverlapping()` / `queryOverlappingAfter()`.
   - The new API still reports raw deduplicated hash candidates, but it returns only candidates whose stored target AABB overlaps the query AABB.
   - `buildSelfPairs()` and `buildExternalPairs()` use it to avoid pushing and re-looping over same-cell-but-non-overlapping candidates before exact distance tests.
   - CCD max-step intentionally keeps the old query path; applying the new API there regressed the smoke/full measurements.
   - Final 20-step smoke after narrowing to broad phase only: `contact.surface.build_active_set` 0.676-0.690s / 140 calls, `self_ee_query` 0.259-0.279s / 140 calls, and `contact.adapter.max_step` remains 0.093-0.095s / 20 calls.
   - Full 2000-step broad-only profile: `contact.surface.build_active_set` 196.03s / 25054 calls (7.82ms/call), versus the #5-#10 retained 197.22s / 24560 calls (8.03ms/call).

8. Packed low-bit spatial hash key
   - Replaces the XOR-prime spatial hash key with a cheap 63-bit packed key using the low 21 bits of each grid coordinate.
   - This keeps the old single-key `unordered_map<uint64_t, vector<int>>` shape, but avoids dense low-coordinate collisions that inflated raw hash candidates.
   - A focused regression test covers a known collision from the old Teschner-style key: `(0,0,0)` and `(19349663,73856093,0)` no longer merge.
   - Exact collision-safe `GridCoord` keys and a hash-bucket/equality variant were tested first, but rejected because lookup overhead outweighed candidate reduction in the 20-step smoke.
   - Final 20-step smoke after warmup: `contact.surface.build_active_set` 0.616-0.688s / 140 calls versus #11's 0.676-0.690s, with self EE raw candidates 24.07M -> 18.99M and self PT raw candidates 8.68M -> 7.12M.
   - Full 2000-step profile: `contact.surface.build_active_set` 196.03s / 25054 calls (7.82ms/call) -> 189.72s / 25290 calls (7.50ms/call); `static` 4.96ms/call -> 4.66ms/call.

9. Query-side primitive extraction hoist
   - `buildSelfPairs()`, `buildExternalPairs()`, `computeSelfMaxStep()`, and `computeExternalMaxStep()` now load the query-side vertex/edge endpoints and displacements once per query primitive before scanning its candidate list.
   - Candidate counts and exact-test counts are unchanged; this only removes repeated `segment<3>()` copies for the same query primitive inside hot candidate loops.
   - Same-environment 20-step A/B, toggling only this hoist: `contact.surface.build_active_set` 0.331-0.351s -> 0.295-0.321s / 140 calls; `self_ee_query` 0.086-0.101s -> 0.070-0.081s; `self_pt_query` 0.042-0.046s -> 0.033-0.039s; `contact.adapter.max_step` 0.052-0.064s -> 0.047-0.052s.
   - Full retained #9 profile: `contact.surface.build_active_set` 79.87s / 24003 calls (3.33ms/call), `static` 51.14s (2.13ms/call), `external` 28.67s (1.19ms/call), and `contact.adapter.max_step` 11.18s / 4680 calls (2.39ms/call). The full run was faster across untouched sections too, so the short same-environment A/B remains the primary attribution evidence.
   - Retained smoke log: `/private/tmp/libpgo-query-primitive-cache-smoke-retained.log`; same-env baselines: `/private/tmp/libpgo-query-primitive-cache-baseline-sameenv-{1,2}.log`.

10. External dynamic-block barrier wrapper + lazy kernel temporaries
   - Adds external-specific barrier contribution types for dynamic point, dynamic triangle, and dynamic edge blocks.
   - `computeExternal*()` now scatters 3/9/6-DOF local blocks instead of receiving a full 12-DOF contribution and then discarding obstacle blocks in the assembler.
   - The wrappers deliberately preserve the old Newton semantics by comparing against and extracting the exact dynamic sub-blocks of the generic 12-DOF PSD-projected kernels.
   - `LocalContribution` and `PTDistAll` / `EEDistAll` no longer pre-zero 12-DOF vectors/matrices on construction; inactive and unrequested fields are still explicitly zeroed. The EE mollifier temporary gradient/Hessian is also only materialized when `eps_ee > 0`.
   - Focused tests compare external dynamic point/triangle/edge kernels against the generic 12-DOF sub-blocks.
   - Same-environment 600-step A/B on `box-with-sphere-lite`: baseline `active_set_combined.external=14.45s / 1801 calls`, `external_ee=7.13s / 2.29M pairs` (3.12us/pair); retained `external=13.21s / 1799 calls`, `external_ee=6.74s / 2.28M pairs` (2.96us/pair). This is a modest retained win for the remaining external barrier hotspot.

11. Backtracking line-search initial-trial energy reuse
   - `NewtonSolver` already evaluates `energy->func(x + dx)` once before calling the selected line search so it can check the full step and choose a short/long line-search budget.
   - The default `LSM_BACKTRACK` path used to immediately evaluate the same alpha=1 point again inside `LineSearch::backtracking()`.
   - `LineSearch::backtrackingWithInitialValue()` now starts from the caller-provided alpha=1 energy, preserving the old Armijo/backtracking decisions while removing one duplicated `func()` call per Newton iteration.
   - Focused `NewtonSolver_gtest` coverage counts a one-iteration quadratic solve and verifies the backtracking path uses 2 `func()` calls instead of the previous 3.
   - 600-step retained-code comparison on `box-with-sphere-lite`: `contact.surface.energy` dropped from 3024 calls / 46.07s to 1800 calls / 25.43s, and `contact.surface.build_active_set` dropped from 6023 calls / 87.57s to 4798 calls / 63.91s. Solver path and active contact pair counts stayed essentially the same (`solver.linear_solve` 1199 -> 1198, external EE barrier pairs 2.275M -> 2.275M).

12. Newton standalone-gradient elimination
   - `NewtonSolver` now initializes the relative convergence scale from the first `func_grad_hessian()` gradient instead of doing a separate pre-loop `gradient()` call at the same state.
   - On convergence, `NewtonSolver` records the final filtered gradient norm/max-norm in `SolveDiagnostics`; `ImplicitBackwardEulerTimeIntegrator` reuses those diagnostics for verbose/residual printing instead of recomputing `eulerEnergy->gradient(z, residual)`.
   - The fallback path still recomputes the residual gradient when the solver exits without a final gradient that is known to match the output state.
   - Verification also exposed that `TRBDF2TimeIntegratorEnergy` had no fused `func_grad_hessian()` override for non-fixed-topology models; it now mirrors the implicit-Euler fused path so Newton does not fall back to separate `gradient()+hessian()+func()` there.
   - Focused tests cover both sides: the one-iteration Newton backtracking test verifies no pre-loop standalone gradient call, and `ImplicitBackwardEulerTimeIntegratorGTest.ResidualPrintReusesNewtonFinalGradient` verifies residual printing does not add another model `gradient()` call after convergence.
   - 600-step retained-code comparison after #11: standalone IPC gradient evaluation was eliminated (`contact.surface.gradient` 1200 calls / 15.30s -> 0 calls), and `contact.surface.build_active_set` dropped from 4798 calls / 63.91s to 3598 calls / 35.02s. Solver path and external EE pair volume stayed unchanged (`solver.linear_solve` 1198 -> 1198, external EE barrier pairs 2.275M -> 2.275M).

13. Cached implicit energy-component printing
   - `ImplicitBackwardEulerEnergy::func_grad_hessian()` now caches the main implicit-Euler energy and per-model component energies it already computes for the final converged Newton state.
   - `ImplicitBackwardEulerTimeIntegrator` clears that cache before each solve and allows `printImplicitEnergy()` to reuse it only when `SolveDiagnostics` proves Newton's final gradient stats came from the returned state.
   - This preserves the existing `Energy components:` output while avoiding another `func()` pass over material and IPC models after each converged timestep.
   - The first cache version copied the full DOF vector for stale-cache checks and was rejected after profiling; the retained version uses the solver diagnostics validity signal and avoids the large vector copy.
   - Focused TDD guard extends `ImplicitBackwardEulerTimeIntegratorGTest.ResidualPrintReusesNewtonFinalGradient`: before the change, residual/energy printing performed two model `func()` calls; after the change it performs one and still prints `sub 0`.
   - 600-step retained-code comparison after #12: `contact.surface.energy` dropped from 1800 calls / 18.23s to 1206 calls / 11.42s, `contact.surface.build_active_set` dropped from 3598 calls / 35.02s to 3005 calls / 27.86s, and wall-clock log timestamps improved from about 88.4s to 75.2s. Solver path and external EE pair volume stayed stable (`solver.linear_solve` 1198 -> 1199, external EE barrier pairs 2.275M -> 2.275M).

14. Accepted line-search active-set cache
   - `EmbeddedSurfaceIPCPotentialEnergy::computeSurfaceEnergy()` now keeps the `SurfaceIPCActiveSet` built for an energy-only line-search evaluation.
   - The next `func_grad_hessian()`, `func_grad()`, or `gradient_hessian()` evaluation may consume that cached active set only when the newly computed surface positions compare exactly equal to the cached positions. The cache is cleared on obstacle time/static-state changes, and standalone `gradient()` / `hessian()` calls keep building independent active sets.
   - This targets the narrow safe case left after #11/#12/#13: Newton evaluates `func(x + alpha dx)` for line search, accepts that point, and the next Newton iteration immediately asks for fused energy/gradient/hessian at the same accepted state.
   - Focused TDD guard `EmbeddedSurfaceIPCPotentialEnergyGTest.EnergyOnlyEvaluationSeedsNextCombinedActiveSet` first failed with two active-set builds for `func(u); func_grad_hessian(u)`, then passed with one build while preserving `contact.surface.energy`, `active_set_energy`, and `active_set_combined` profiling.
   - `NewtonSolverGTest.StepTooSmallConvergenceRecordsFinalGradientStats` also covers the loose-convergence `dx too small` branch so residual printing does not reintroduce standalone IPC gradients after that solver exit.
   - 600-step retained-code comparison after #13: `contact.surface.build_active_set` dropped from 3005 calls / 27.86s to 1837 calls / 26.27s, and `contact.adapter.hessian_direct` dropped from 35.40s to 28.17s. The final profile still had no standalone IPC gradient entries. Wall-clock was noisy because other long-running simulations were active, so the retained evidence is the active-set call-count reduction plus nested hessian-direct timing.

15. Coplanar obstacle-edge filtering for external EE
   - `ObstacleSurface` now exposes both `uniqueEdges()` and `contactEdges()`. `uniqueEdges()` remains the full topological edge set; `contactEdges()` is the obstacle edge set used by external EE broad phase, external EE CCD, and `ObstaclePoseCache` edge boxes/hash.
   - `contactEdges()` keeps boundary, nonmanifold, sharp, and degenerate edges, but removes two-face coplanar interior tessellation edges. It is rebuilt from the current obstacle pose on `ObstacleSurface::update(t)`, so moving/deforming obstacle poses keep their current normals.
   - The retained implementation is deliberately narrower than broad feature-edge simplification: it only removes coplanar interior obstacle edges, while PT/TP triangle contacts still cover the surface and non-coplanar feature edges remain eligible for EE.
   - Focused TDD guards cover the behavior that matters: a flat two-triangle patch drops only the interior diagonal, a folded two-triangle patch keeps the sharp interior edge, and external broad phase does not emit an EE pair solely from a coplanar obstacle diagonal.
   - 600-step retained-code comparison after #14: the flat `bottom.1.obj` obstacle has 12480 topological edges, 12160 coplanar interior edges, and 320 sharp feature edges. `contact.surface.active_set_combined.external_ee` dropped from 8.124s / 2.281M pairs to 0.000098s / 0 pairs; `pair_build.external_ee` dropped from 2.874s to 0.535s with 0 accepted pairs, 0 distance tests, and 0 hash candidates. `contact.surface.build_active_set` dropped from 26.27s / 1837 calls to 14.58s / 1733 calls. All 600 profiled timesteps converged.

## Rejected Attempt

Self broad phase one-sided AABB inflation was tried and reverted.

Hypothesis: hash target primitives with tight AABBs and inflate only query primitives by `dhat`, mirroring the external obstacle path.

Result: On the 20-step no-contact smoke, `contact.surface.build_active_set` regressed from 0.894s to 1.041s. The extra edge-query AABB build dominated because the scene had no active self contacts. The brute-force self broad-phase test added for this attempt remains useful for future tightening experiments.

Self broad phase tight-AABB lower-bound filtering was also tried and reverted.

Hypothesis: keep the existing inflated hash, but build tight vertex/triangle/edge AABBs and skip exact PT/EE distance when the tight AABB distance is already `>= dhat`.

Result: It did not reliably improve the 20-step no-contact smoke after the adopted optimizations. One comparable run regressed from 0.692s to 0.736s; the added tight AABB construction cost outweighed the saved exact-distance calls for this scene.

Separate self PT/EE cell-size selection was also tried and reverted.

Hypothesis: use triangle average AABB diagonal for PT hashes and edge average AABB diagonal for EE hashes instead of sharing the triangle-derived cell size.

Result: It did not beat the retained implementation. One experiment run measured `self_ee_query=0.332s`, `contact.surface.build_active_set=0.781s`, and `contact.adapter.max_step=0.107s`, with no clear improvement over the retained-code smoke range.

Thread-local external combined gradient accumulation was also tried and reverted.

Hypothesis: in `computeExternalAll()`, replace per-pair atomic gradient scatter with TBB thread-local dense gradients and reduce once after the external PT/TP/EE loops. The external combined profile showed large pair counts, so atomics looked like a plausible source of pressure.

Result: Full 2000-step profile did not improve. `contact.surface.active_set_combined.external` measured 82.72s / 6715 calls versus the retained 82.43s / 6676-call reference; `external_ee` was effectively unchanged at 51.46s. The TLS setup and final reduction did not pay for itself in this workload.

Applying overlap-filtered hash queries to CCD max-step was also tried and narrowed back out.

Hypothesis: the same query-side AABB filtering that helps active-set broad phase would reduce self swept PT/EE candidate-vector pressure.

Result: Focused max-step tests passed, but the 20-step smoke moved `contact.adapter.max_step` to 0.102-0.116s versus the retained/broad-only 0.093-0.095s range, and a full run measured 23.32s / 4685 calls versus the #5-#10 retained 21.87s / 4701-call reference. The retained #7 only uses the new query API in active-set broad phase.

Exact coordinate-key spatial hashing was also tried and replaced by #8.

Hypothesis: storing exact `(ix,iy,iz)` cell coordinates would remove all spatial-hash key collisions and lower raw candidate pressure.

Result: The candidate counters improved, but the direct `GridCoord` unordered-map key and a hash-bucket/equality design both regressed or stayed above the retained #11 short-run timing. The retained packed key captures the useful low-coordinate collision reduction while preserving the cheaper single-`uint64_t` map shape.

Sorted bulk hash build was also tried and reverted.

Hypothesis: collect `(key, primitiveId)` references during `SpatialHashGrid::build()`, sort by key, and fill each cell vector in grouped order to reduce repeated `unordered_map` lookup and vector growth.

Result: It regressed the 20-step smoke. `self_ee_hash_insert` moved from the packed-key range of about 0.10-0.12s to 0.17-0.19s, `self_pt_hash_insert` moved from about 0.10-0.11s to 0.20-0.21s, and `contact.surface.build_active_set` moved to 0.75-0.79s. Sorting/reference materialization cost was not repaid.

Visitor/callback overlap queries were also tried and reverted.

Hypothesis: replace `queryOverlapping(..., candidates)` with a callback-style scan so active-set broad phase can run distance tests directly without materializing and re-looping over a temporary candidate vector.

Result: Focused tests passed, but the full profile did not show a per-call win. `pair_build.static` measured 118.05s / 24160 calls (4.89ms/call) versus #8's 117.87s / 25290 calls (4.66ms/call), and `self_ee_query` moved to 1.86ms/call versus #8's 1.68ms/call. The lower total run time was due to fewer active-set calls/path drift, not a safer retained improvement.

Per-call surface vertex caches were also tried and reverted.

Hypothesis: build `std::vector<V3d>` caches for dynamic positions, dynamic displacements, and overlapping obstacle positions once per broad-phase / max-step call, then use references in candidate loops to remove candidate-side `segment<3>()` extraction.

Result: Focused broad-phase, max-step, and `runIPCSim` tests passed, and candidate counters stayed unchanged, but the 20-step smoke did not improve. Baseline `/private/tmp/libpgo-vertex-cache-baseline-smoke.log`: `build_active_set=0.3153s`, `self_ee_query=0.0789s`, `self_pt_query=0.0367s`, `adapter.max_step=0.0511s`. Best sequential retained sample `/private/tmp/libpgo-vertex-cache-retained-smoke-4.log`: `build_active_set=0.3150s`, `self_ee_query=0.0816s`, `self_pt_query=0.0357s`, `adapter.max_step=0.0507s`. The cache construction overhead offsets the saved segment loads in the early no-contact phase, so this was not retained.

Persistent obstacle-pose vertex caching was also tried and reverted.

Hypothesis: materialize sampled obstacle vertices once in `ObstaclePoseCache` and reuse them in external broad phase, external max-step, and external barrier assembly.

Result: Focused tests passed, and the 20-step smoke was neutral because there was no external overlap (`/private/tmp/libpgo-obstacle-vertex-cache-smoke.log`: `build_active_set=0.3173s / 140`, `pair_build.external=0.0053s / 140`). The full 2000-step profile regressed versus the #9/#13 retained snapshot: `/private/tmp/libpgo-obstacle-vertex-cache-full.log` measured `build_active_set=90.06s / 24555`, `pair_build.external=32.44s / 24555`, and `active_set_combined.external=32.13s / 6697`, versus `/private/tmp/libpgo-query-primitive-cache-full.log` at `79.87s / 24003`, `28.67s / 24003`, and `28.21s / 6659`. External EE barrier pair volume moved only from 23.00M to 23.16M, so the extra cached-storage/update path was not a retained improvement. Production code was reverted; the surface-bounds cache test was kept.

Direct external dynamic-block assembly was also tried and reverted.

Hypothesis: instead of calling the generic 12-DOF kernel and extracting dynamic sub-blocks, assemble the external dynamic contribution directly while still applying the full 12-DOF PSD projection before extracting the dynamic block.

Result: Correctness tests passed, but the 600-step profile regressed. The wrapper retained run measured `external_ee=6.74s / 2.28M pairs`; the direct dynamic assembly run measured `external_ee=10.51s / 2.29M pairs` and `active_set_combined.external=16.52s`. The extra local expression/project path was not optimized as well as the generic wrapper path.

External EE pair sorting by dynamic edge was also tried and reverted.

Hypothesis: sort external EE pairs by dynamic edge before barrier assembly to improve gradient/Hessian scatter locality.

Result: Pair-set correctness tests passed, but the 600-step profile regressed in the target section. The sorted run measured `active_set_combined.external=17.08s`, `external_ee=10.72s / 2.28M pairs`, versus the retained wrapper run at `13.21s` and `6.74s`. Sorting slightly reduced pair-build timings in that run but did not compensate for worse barrier execution.

Self broad-phase topology-aware hash-query filtering was also tried and reverted.

Hypothesis: add predicate-filtered `SpatialHashGrid::queryOverlapping*()` helpers so self PT can drop incident triangles and self EE can drop adjacent edges before appending candidates to the hot vectors. Raw hash-candidate accounting would stay conservative, but candidate vectors and post-query loops would shrink.

Result: The red API tests failed first, then focused grid/self-broad-phase/core tests passed after implementation. The 600-step profile did not improve: `self_ee_query` moved from 5.754s / 1733 calls to 6.140s / 1740 calls, `self_pt_query` from 2.278s to 2.301s, and `pair_build.static` from 11.938s to 12.078s. Hash and exact-test counters stayed at the same scale (`self_ee.distance_tests` about 9.0M, `self_pt.distance_tests` about 1.1M), so the extra predicate check on every raw hash candidate was not repaid. Production code and the temporary query API were reverted.

External EE contact-edge union AABB early-out was also tried and reverted.

Hypothesis: cache a union AABB for each obstacle's retained contact-edge boxes, compute a union AABB for the dynamic edge boxes, and skip external EE hash queries when those two boxes do not overlap.

Result: The cache-bounds test failed first, then focused core/external broad-phase/external max-step tests passed after implementation. The 600-step profile regressed: `pair_build.external_ee` moved from 0.535s / 967 calls to 1.149s / 965 calls, `pair_build.external` from 2.636s to 5.638s, and `build_active_set` from 14.579s / 1733 calls to 24.178s / 1731 calls. External EE hash, distance, and accepted counters stayed at zero, meaning the global dynamic-edge union and obstacle contact-edge union were too coarse to skip this scene. Production code and the temporary test were reverted.

External max-step direct swept-surface union was also tried and reverted.

Hypothesis: because the #15/#19 600-step profile has zero external max-step obstacle overlaps, compute only a dynamic swept-surface union box before the obstacle-overlap test, and materialize per-vertex swept boxes only if an obstacle overlaps that union.

Result: Focused max-step tests passed and the 600-step profile converged, but timings regressed: `contact.adapter.max_step` moved from 7.457s / 1131 calls to 14.492s / 1135 calls, `max_step_ee` from 4.320s to 7.541s, `max_step_pt` from 2.004s to 4.424s, and `pair_build.swept` from 0.839s to 1.888s. External max-step hash/CCD counters stayed at zero. The extra direct-union path was not a retained no-overlap optimization, and production code was reverted.

Cached obstacle-vertex hashing for external TP was also tried and reverted.

Hypothesis: store an obstacle vertex hash in `ObstaclePoseCache` and query dynamic triangle boxes against cached obstacle vertex boxes, avoiding the per-overlap dynamic triangle hash rebuild used by external TP broad phase.

Result: The cache test failed first, then focused core/external broad-phase tests passed and the 600-step profile converged. The target timing did not improve: `pair_build.external_tp` moved from 1.152s / 967 calls to 1.470s / 1021 calls, `pair_build.external` from 2.636s to 4.522s, and `build_active_set` from 14.579s / 1733 calls to 25.041s / 1787 calls. The inverted query shape plus cached hash did not pay off, and production code plus the temporary cache test were reverted.

Identity embedding sparse-pullback bypass in `MappedSurfacePotentialEnergy` was also tried and reverted.

Hypothesis: detect an identity `surfaceFromSimulationDispMap` and bypass sparse surface-map, gradient-pullback, and Hessian-pullback products for shell-style identity embeddings.

Result: The red profiling test first confirmed identity adapters still recorded pullback profile sections, then focused adapter tests passed after implementation. The 600-step target profile at `/private/tmp/libpgo-identity-map-run.log` showed the run was phase2 volume IPC with a barycentric embedding, not the shell identity path, so `pullback_gradient` and `pullback_hessian` still recorded 1731 calls. The profile regressed versus the #15/#19 baseline: `adapter.func` moved from 9.986s to 22.854s, `adapter.max_step` from 7.457s to 18.251s, and `build_active_set` from 14.579s to 32.517s. The fast path was irrelevant to this target workload, and production/test changes were reverted.

Direct `Eigen::Ref` gradient output for combined IPC assembly was also tried and reverted.

Hypothesis: change `SurfaceIPCCore::computeAll()` plus the combined self/external barrier helpers to accept `Eigen::RefVecXd`, allowing `EmbeddedSurfaceIPCPotentialEnergy::computeSurfaceAll()` to write directly into the adapter's surface-gradient output instead of allocating a temporary `VectorXd` and copying it back.

Result: The compile-red core test first proved the old `VectorXd&` API could not accept a gradient ref, then focused core/adapter/barrier tests passed after implementation. The 600-step target profile at `/private/tmp/libpgo-ref-gradient-run.log` converged but did not improve: `build_active_set` moved from 14.579s / 1733 calls to 38.858s / 2015 calls, `adapter.hessian_direct` from 19.920s / 1731 calls to 27.082s / 1742 calls, and `adapter.max_step` from 7.457s / 1131 calls to 17.558s / 1144 calls. A fresh post-revert retained-code run under the same host state (`/private/tmp/libpgo-post-revert-baseline-run.log`) was noisy but still lower at `build_active_set=29.509s / 1877`, `adapter.hessian_direct=21.193s / 1735`, and `adapter.max_step=14.620s / 1136`. The temporary/copy removal was not a validated end-to-end win, and production/test changes were reverted.

## Verification

Primary commands:

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

Profiling smoke:

- Config: `examples/ipc/cubic/box-with-sphere-lite/box-ipc.json`
- Temporary overrides: `num-timestep=20`, `dump-interval=1000`, `profiling=true`
- Build: `build/base_no_mkl`

Full retained-code profile:

- Config: original 2000-step `examples/ipc/cubic/box-with-sphere-lite/box-ipc.json`
- Temporary copy: `profiling=true`, profile-only output folder
- Final code timing:
  - `contact.surface.build_active_set`: 197.22s / 24560 calls
  - `contact.surface.pair_build.static`: 124.57s / 24560 calls
  - `contact.surface.pair_build.external`: 72.54s / 24560 calls
  - `contact.adapter.max_step`: 21.87s / 4701 calls
  - `contact.surface.active_set_combined`: 82.91s / 6676 calls
  - `contact.surface.active_set_combined.external`: 82.43s / 6676 calls
  - `contact.surface.active_set_combined.external_ee`: 51.47s / 6676 calls
  - `contact.surface.active_set_combined.external_pt`: 13.70s / 6676 calls
  - `contact.surface.active_set_combined.external_tp`: 9.61s / 6676 calls
  - `contact.surface.active_set_combined.self`: 0.48s / 6676 calls
- Key counters:
  - Static external obstacle overlap: 22015 / 24560 calls, so the early-out skips 2545 early calls in the full run.
  - External CCD obstacle overlap: 0 / 4701 calls, so external max-step is fully skipped by the global AABB early-out.
  - Self EE static: 4.20B hash candidates -> 126.6M exact tests -> 35.9k accepted pairs.
  - Self PT static: 1.52B hash candidates -> 15.5M exact tests -> 17.5k accepted pairs.
  - External EE static: 1.91B hash candidates -> 247.7M exact tests -> 86.3M accepted pairs.
  - External barrier combined: 23.05M EE pairs, 3.99M PT pairs, 2.91M TP pairs.
  - Self EE swept CCD: 644.8M hash candidates -> 5.27M CCD tests.

Full broad-only overlap-query profile:

- Config: temporary 2000-step copy with `profiling=true`, output under `/private/tmp/libpgo-overlap-query-full`
- Wall-clock log timestamps: 10:26:22.564 -> 10:33:02.268 (~400s)
- Timing:
  - `contact.surface.build_active_set`: 196.03s / 25054 calls
  - `contact.surface.pair_build.static`: 124.28s / 25054 calls
  - `contact.surface.pair_build.external`: 71.64s / 25054 calls
  - `contact.adapter.max_step`: 22.63s / 4727 calls
  - `contact.surface.active_set_combined`: 78.54s / 6704 calls
  - `contact.surface.active_set_combined.external`: 78.09s / 6704 calls
  - `contact.surface.active_set_combined.external_ee`: 48.08s / 6704 calls
  - `solver.linear_solve`: 6.72s / 4727 calls
- Key counters:
  - Static external obstacle overlap: 22509 / 25054 calls.
  - Self EE static: 4.28B hash candidates -> 129.2M exact tests -> 38.5k accepted pairs.
  - Self PT static: 1.55B hash candidates -> 15.8M exact tests -> 18.3k accepted pairs.
  - External EE static: 1.95B hash candidates -> 250.7M exact tests -> 87.2M accepted pairs.
  - External barrier combined: 23.10M EE pairs, 4.00M PT pairs, 2.92M TP pairs.

Full packed-hash profile:

- Config: temporary 2000-step copy with `profiling=true`, output under `/private/tmp/libpgo-packed-hash-full`
- Wall-clock log timestamps: 10:53:11.448 -> 10:59:36.294 (~385s)
- Timing:
  - `contact.surface.build_active_set`: 189.72s / 25290 calls
  - `contact.surface.pair_build.static`: 117.87s / 25290 calls
  - `contact.surface.pair_build.external`: 71.72s / 25290 calls
  - `contact.adapter.max_step`: 23.73s / 4735 calls
  - `contact.surface.active_set_combined`: 63.88s / 6710 calls; barrier code was unchanged, so treat this as run-to-run variation
  - `solver.linear_solve`: 8.36s / 4735 calls
- Key counters:
  - Static external obstacle overlap: 22745 / 25290 calls.
  - Self EE static: 3.45B hash candidates -> 130.3M exact tests -> 33.9k accepted pairs.
  - Self PT static: 1.29B hash candidates -> 16.0M exact tests -> 16.0k accepted pairs.
  - External EE static: 1.39B hash candidates -> 255.4M exact tests -> 88.9M accepted pairs.
  - Self EE swept CCD: 533.8M hash candidates -> 5.29M CCD tests.
  - Self PT swept CCD: 198.1M hash candidates -> 595k CCD tests.

Full query-side extraction-hoist profile:

- Config: temporary 2000-step copy with `profiling=true`, output under `/private/tmp/libpgo-query-primitive-cache-full`
- Run start around 18:42:40, profile summary at 18:45:27.407 (~167s)
- Timing:
  - `contact.surface.build_active_set`: 79.87s / 24003 calls (3.33ms/call)
  - `contact.surface.pair_build.static`: 51.14s / 24003 calls (2.13ms/call)
  - `contact.surface.pair_build.external`: 28.67s / 24003 calls (1.19ms/call)
  - `contact.surface.pair_build.self_ee_query`: 12.36s / 24003 calls
  - `contact.surface.pair_build.self_pt_query`: 5.96s / 24003 calls
  - `contact.adapter.max_step`: 11.18s / 4680 calls (2.39ms/call)
  - `contact.surface.max_step_ee`: 5.53s / 4680 calls
  - `contact.surface.max_step_pt`: 4.06s / 4680 calls
  - `contact.surface.active_set_combined`: 28.49s / 6659 calls; barrier code was unchanged, so treat this as run-to-run/path variation
  - `solver.linear_solve`: 5.61s / 4680 calls
- Key counters:
  - Static external obstacle overlap: 21458 / 24003 calls.
  - Self EE static: 3.28B hash candidates -> 123.6M exact tests -> 33.3k accepted pairs.
  - Self PT static: 1.23B hash candidates -> 15.2M exact tests -> 15.3k accepted pairs.
  - External EE static: 1.31B hash candidates -> 241.8M exact tests -> 84.2M accepted pairs.
  - External barrier combined: 23.00M EE pairs, 3.98M PT pairs, 2.90M TP pairs.
  - Self EE swept CCD: 527.6M hash candidates -> 5.25M CCD tests.
  - Self PT swept CCD: 195.8M hash candidates -> 591k CCD tests.
- Attribution note: the full profile improved more than #9 can explain by itself and also reduced untouched sections. Use the same-environment 20-step A/B as the direct evidence for the hoist; use this full profile as the retained-code snapshot.

## Remaining Optimization Space

- Candidate-count instrumentation is now in place. #8 reduces raw hash candidate enumeration and #9 reduces per-candidate coordinate-load overhead, but the #15 profile still enumerates about 139k self EE and 52k self PT raw hash candidates per static call before exact overlap/distance pruning. Self broad phase is now a clearer remaining structural target than external EE for the flat-obstacle scene.
- After #11/#12/#13/#14, the obvious duplicated Newton/residual/diagnostic evaluations in the default IPC path were removed for the profiled implicit-Euler run. #20 then tested the remaining high-risk line-search sharing idea directly: the same-host 600-step A/B moved `build_active_set` from 29.509s / 1877 calls to 21.951s / 1734 calls and `contact.surface.energy` from 20.857s / 1277 calls to 14.884s / 1134 calls, with all 600 timesteps converged. The remaining evaluation-count wins are now more likely to require broader Newton policy changes rather than another local duplicate-call fix.
- In the flat-box profile, #15 eliminates the previous external EE candidate/barrier workload by removing coplanar obstacle triangulation edges from the external EE contact edge set: `active_set_combined.external_ee.pairs` moved from 2.281M to 0. External PT/TP remain active and are now the dominant external barrier work (`external_pt=2.83s`, `external_tp=2.19s` in the 600-step #15 run).
- External CCD max-step is no longer a meaningful target in this scene because the global AABB early-out skips all external max-step calls. Remaining CCD work is self swept EE/PT.
- Barrier assembly remains visible, but its shape changed: self barrier work is still small, external EE is no longer a target for this flat obstacle, and the remaining obvious barrier work is external PT/TP plus the broader question of reducing pair counts. Broader smooth-feature filtering is still high risk and not done; #15 only removes coplanar interior obstacle edges while keeping boundary, nonmanifold, sharp, and degenerate edges.

## Concrete Future Optimization Candidates

This section is a backlog for the next optimization pass. It intentionally avoids repeating ideas already rejected above: per-call vertex caches, persistent obstacle vertex caches, exact coordinate hash keys, sorted hash builds, visitor queries, and CCD-side overlap filtering should not be revisited unless a new profile shows a different workload shape.

### 1. External-only dynamic-block barrier kernels (completed as #10)

- **Target:** `src/core/contact/ipc/core/surfaceIPCBarrierKernels.h`, the corresponding kernel implementation, `src/core/contact/ipc/core/surfaceIPCExternalBarrierAssembler.cpp`, and `tests/src/core/contact/surfaceIPCBarrierAssembler_gtest.cpp`.
- **Idea:** external obstacles are fixed with respect to the solve DOFs, but the current external assembler still calls generic `pointTriangle()` / `edgeEdge()` kernels that return a full `V12d` gradient and `M12d` Hessian. The assembler then scatters only dynamic blocks. Add external-specialized kernels that return only dynamic DOF blocks:
  - external PT: dynamic point vs fixed obstacle triangle -> energy, `V3d` gradient, `M3d` Hessian.
  - external TP: fixed obstacle point vs dynamic triangle -> energy, `V9d` gradient, `M9d` Hessian.
  - external EE: dynamic edge vs fixed obstacle edge -> energy, `V6d` gradient, `M6d` Hessian.
- **Why it may work:** the #9 full profile still processed about 23.00M external EE pairs in `computeExternalAll()`. EE currently allocates/fills/scatters from 12-DOF local objects even though the obstacle 6 DOFs are discarded. Reducing the local math and Hessian scatter width targets the largest retained barrier hotspot without changing pair generation or contact semantics.
- **Risk:** medium. The energy must remain identical, and the dynamic gradient/Hessian blocks must exactly match the corresponding sub-blocks of the generic 12-DOF kernel. A wrong block extraction will silently corrupt Newton steps.
- **Validation:** first add focused tests that compare each new external kernel against the generic kernel over deterministic PT/TP/EE samples, including active and inactive pairs. For Hessians, compare the dynamic sub-blocks and verify symmetry/PSD projection behavior where relevant. Then run `surfaceIPCBarrierAssembler_gtest`, `surfaceIPCCore_gtest`, `embeddedSurfaceIPCPotentialEnergy_gtest`, and a 20-step plus full profile. Keep only if `active_set_combined.external_ee` improves on a per-pair or per-call basis.
- **Status:** completed as retained change #10 using exact-subblock wrappers plus lazy kernel temporaries. A deeper direct dynamic assembly was tested but rejected because it regressed the same-environment 600-step profile.

### 2. Line-search active-set superset reuse (retained as #20 for default line search)

- **Target:** `src/core/contact/ipc/core/surfaceIPCCore.*`, active-set storage/diagnostics, and the adapter path used by `embeddedSurfaceIPCPotentialEnergy`.
- **Idea:** line search evaluates `func()` multiple times along `x + alpha * dx`. Instead of rebuilding active sets for each trial alpha, build one conservative superset active set for the whole segment `alpha in [0, 1]` using swept or enlarged AABBs, then evaluate barrier energy for each alpha against that superset. The narrower exact accepted-state reuse case is already completed as #14; this candidate is only for sharing across different trial alphas before acceptance.
- **Why it may work:** `buildActiveSet()` remains a frequent call even after within-evaluation fusion. Line-search calls are geometrically related, so they are more plausible to share than arbitrary Newton iterations. Unlike cross-Newton reuse, the motion path is explicit: every trial point lies on the same segment, which gives a cleaner conservative-bound story and a clearer debug oracle.
- **Risk:** high. The superset must never miss a pair that can become active at any line-search alpha. If the superset is too large, barrier evaluation may become slower than rebuilding smaller active sets.
- **Validation:** add a debug mode that builds both the proposed superset and the exact per-alpha active set, then asserts exact pairs are a subset of the superset over sampled alpha values. Profile line-search-heavy runs and report both active-set build savings and extra barrier-pair cost. Keep it behind an option until the superset inflation ratio and end-to-end timing are stable across multiple cases.
- **Status:** retained as #20 for the default Newton backtracking/simple line-search paths. The implementation builds swept-AABB self/external supersets for the full `alpha in [0, 1]` segment, reuses the pair list across trial energy calls, and lets barrier kernels return zero for inactive `d2 >= dhat^2` pairs. Golden/Brent searches can expand the bracket past `alpha=1`, so they keep the exact per-alpha path. Focused tests check exact per-alpha energy equality and exact-active-set subset coverage over sampled alphas.
- **Result:** same-host 600-step A/B against `/private/tmp/libpgo-post-revert-baseline-run.log` improved the target sections: `build_active_set` 29.509s / 1877 calls -> 21.951s / 1734 calls, `contact.surface.energy` 20.857s / 1277 calls -> 14.884s / 1134 calls, and `contact.adapter.func` 21.083s / 1277 calls -> 15.092s / 1134 calls. All 600 timesteps converged and the failure-pattern search returned no matches.
- **Caveat:** pair-list counters can include conservative superset pairs after this change, especially when the accepted line-search state feeds the next fused Newton evaluation. Those counters remain workload diagnostics, not always exact `dhat` active-pair counts. Keep validating on non-flat and line-search-heavy scenes before treating this as universally low risk.

### 3. External EE pair ordering and scatter locality

- **Target:** `src/core/contact/ipc/broadPhase/surfaceIPCBroadPhase.cpp`, external pair containers in `src/core/contact/ipc/broadPhase/surfaceIPCBroadPhase.h`, `src/core/contact/ipc/core/surfaceIPCExternalBarrierAssembler.cpp`, and pair-ordering tests.
- **Idea:** after external EE pairs are generated, group or sort them by dynamic edge, or by a compact dynamic edge block range, before the combined external barrier pass. The goal is to process pairs touching nearby dynamic DOFs together and make gradient scatter / Hessian triplet writes more cache-friendly.
- **Why it may work:** the rejected TLS dense-gradient experiment showed that a broad reduction over all dynamic vertices is too expensive. A narrower locality strategy may still help external EE, where pair count is high and the same dynamic edge can appear against many obstacle edges.
- **Risk:** low to medium for correctness, medium for performance. Pair order should not change mathematical results beyond floating-point summation order, but sorting millions of pairs can itself dominate if done every active-set build. It must be measured separately from barrier execution.
- **Validation:** add a golden pair-set test that verifies sorted/grouped output has the same pair multiset and weights as the current output. Profile both `pair_build.external_ee` and `active_set_combined.external_ee`; keep only if the barrier gain exceeds sorting/build overhead. Include a determinism check if exact output ordering is consumed anywhere else.
- **Status:** tested and rejected for the pre-#15 workload. Sorting by dynamic edge preserved pair-set correctness but regressed the 600-step target section (`active_set_combined.external=17.08s`, `external_ee=10.72s / 2.28M pairs` versus the retained wrapper run at 13.21s and 6.74s). After #15, external EE pairs are 0 in the flat-obstacle profile, so this is not an obvious target for that workload.
- **Priority:** low unless a future scene has many retained sharp/boundary obstacle EE pairs.

### 4. Feature-edge filtering for obstacle EE pairs (completed narrowly as #15)

- **Target:** `src/core/contact/ipc/external/obstacleSurface.*`, `src/core/contact/ipc/external/obstaclePoseCache.*`, external edge construction/query code in `surfaceIPCBroadPhase.cpp`, and new external broad-phase regression tests.
- **Idea:** classify obstacle edges as boundary, nonmanifold, sharp-feature, coplanar interior, or degenerate. Build the external EE obstacle hash from the conservative contact-edge set, while leaving PT/TP triangle interactions unchanged.
- **Status:** completed as retained change #15 for the narrow coplanar-interior case. The implementation removes only coplanar two-face interior obstacle edges by default. Boundary, nonmanifold, sharp, and degenerate edges are retained and recomputed on obstacle pose updates.
- **Why it worked here:** the profiled `bottom.1.obj` obstacle has 12480 topological edges, of which 12160 are coplanar interior tessellation edges and only 320 are sharp feature edges. The 600-step profile moved external EE barrier pairs from 2.281M to 0 and `contact.surface.active_set_combined.external_ee` from 8.124s to 0.000098s.
- **Residual risk:** broader "smooth feature" filtering is still not implemented. Do not generalize #15 to curved smooth interiors without additional collision/penetration tests and an option gate; EE constraints can be necessary for robustness outside flat triangulation diagonals.
- **Priority:** completed for flat coplanar obstacle tessellation. Revisit only for scenes with many non-coplanar retained feature edges or for a deliberately gated broader smooth-surface policy.

### 5. Hierarchical external broad phase for obstacle edges/triangles

- **Target:** a new broad-phase helper under `src/core/contact/ipc/broadPhase/`, obstacle cache storage in `obstaclePoseCache.*`, external pair build in `surfaceIPCBroadPhase.cpp`, and `spatialHashGrid_gtest`-style correctness tests for the new query structure.
- **Idea:** build a BVH or other hierarchical structure over obstacle triangles/edges, refit/rebuild it when the obstacle pose changes, and query dynamic vertex/edge AABBs against that hierarchy instead of using only a uniform spatial hash. The current hash is simple and fast, but inflated AABBs can enumerate many same-cell candidates after overlap begins.
- **Why it may work:** packed hash keys reduced raw cell-collision noise, but candidate volume remains structural after obstacle overlap. After #15, a hierarchy is more likely to help external PT/TP for the flat-box profile; external EE would matter mainly in scenes with many retained sharp/boundary obstacle edges.
- **Risk:** high implementation cost and moderate correctness risk. The query must be conservative, parallel-friendly, and must preserve pair-set coverage. A poorly tuned BVH can be slower than the current hash on small or uniformly dense meshes.
- **Validation:** implement behind an option or strategy enum. First compare pair-set equality against the existing spatial hash on deterministic external PT/TP/EE tests. Then profile candidate counters and per-call timings on `box-with-sphere-lite` and at least one non-flat obstacle scene. Do not remove the spatial hash path until the BVH wins on multiple profiles.
- **Priority:** high effort, likely only worth doing after kernel/scatter work.

### 6. Incremental active-set / spatial-hash updates

- **Target:** `SpatialHashGrid`, self/external broad phase builders, active-set lifecycle in `surfaceIPCCore.*`, and topology/pose change tracking.
- **Idea:** update only primitives whose AABBs moved across grid cells or whose local neighborhood changed, instead of rebuilding all dynamic AABBs, hashes, and pairs each active-set call.
- **Why it may work:** the surface topology is fixed and per-iteration motion is often small. In principle, many primitives could keep the same grid membership over consecutive Newton steps.
- **Risk:** very high. Self pairs need correct de-duplication, deleted/inserted cell references must be exact, and parallel mutation of hash cells is tricky. A missed stale pair or missing new pair is a contact correctness bug.
- **Validation:** start with an offline checker that compares incremental output against full rebuild for every active-set call on a short run. Keep full rebuild as a fallback and only enable incremental mode under exhaustive debug assertions until it survives multiple cases.
- **Priority:** long-term only. Do not start here unless the goal is a larger broad-phase rewrite.

### 7. Solver / Newton-path evaluation-count reduction

- **Target:** solver integration around `runIPCSim`, `embeddedSurfaceIPCPotentialEnergy`, and any line-search / Newton policy configuration.
- **Idea:** reduce the number of times IPC energy/gradient/hessian are requested rather than making each request faster. Candidates include better line-search initialization, contact-aware damping, reusing material/contact components when positions are unchanged, or exposing diagnostics that identify line-search-heavy frames.
- **Why it may work:** full-run timings can shift significantly with Newton path and line-search counts. Reducing evaluations attacks all expensive sections simultaneously.
- **Risk:** medium to high. This changes convergence behavior and can affect robustness more than localized kernel changes.
- **Validation:** track per-timestep Newton iterations, line-search `func()` calls, active-set rebuilds, accepted step sizes, and final residuals. Require identical or improved convergence status across a batch, not just a faster single run.
- **Status:** partially completed as #11/#12/#13/#14 for exact duplicate evaluation removal: alpha=1 backtracking energy reuse, deferred initial Newton gradient normalization, timestep residual reuse of Newton's final gradient diagnostics, cached timestep energy-component printing, and exact accepted-state active-set reuse from line search into the next fused Newton evaluation. Larger Newton-path changes remain high-risk and should still require per-timestep convergence evidence.
