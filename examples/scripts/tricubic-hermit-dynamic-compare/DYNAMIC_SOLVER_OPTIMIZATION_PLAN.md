# Dynamic Solver Optimization Spec and Implementation Plan

## Profiling Context

The `cubic_hermite` dynamic run shows two low-parallelism hot paths:

- `newton.solver.symbolic_analyze`: about 40-50 s/frame before reuse, about 3-4 average cores.
- `EnergySet::func_grad_hessian` dynamic sparse assembly:
  - `energy_set.fgh.dynamic.set_from_triplets`: about 15-20 s/frame, about 1 core.
  - `energy_set.fgh.dynamic.add_global_hessian`: about 11-15 s/frame, about 1 core.

MKL Pardiso numerical factorization and solve already use about 31 cores, so thread
tuning is not the main lever.

## Phase 1: Pardiso Symbolic Pattern Reuse

Goal: rebuild the Newton sparse solver only when `activeSystemMatrix()` changes
sparsity pattern.

Pattern equality must compare:

- rows
- cols
- nonzero count
- `outerIndexPtr()[0..outerSize]`
- `innerIndexPtr()[0..nonZeros-1]`

Matrix values must not participate in pattern equality.

Implementation:

- Add a `NewtonSolver` pattern cache with dimensions, nonzero count, hash, and exact
  copies of outer/inner index arrays.
- Use hash only as a fast negative check; exact arrays still decide reuse.
- Update the cache only after successful solver construction.
- Invalidate the cache when fixed DOFs/topology changes, solver reset happens, or
  solve failure leaves solver state uncertain.
- Keep numeric `factorize()` and `solve()` on every Newton iteration.

Verification:

- Build `pypgo_core`.
- Run Python import/profiling smoke.
- Resume `cubic_hermite` for at least 2 accepted frames with `PGO_PROFILE_DYNAMIC=1`.
- Confirm `newton.solver.symbolic_analyze` drops from every Newton iteration to
  pattern-change only, while `solver.factorize` and `solver.solve` still run every
  iteration.

Observed Phase 1 result:

- Frames 91-94 before reuse: `symbolic_analyze` averaged 4 calls/frame and 39.8 s/frame.
- Frames 95-96 after reuse: `symbolic_analyze` averaged 1 call/frame and 9.8 s/frame.
- Frame wall time averaged 132.9 s before and 104.8 s after.

## Phase 2: EnergySet Dynamic Hessian Assembly Cache

Goal: avoid this dynamic-term path when local dynamic Hessian pattern is unchanged:

```cpp
KiGlobal.setFromTriplets(entries.begin(), entries.end());
hess = hess + KiGlobal;
```

Implementation plan:

- Implement this first as a verification-only cache. The old assembly path remains
  authoritative until verification passes.

Detailed worker plan:

1. Keep the write scope narrow:
   - `src/core/nonlinearOptimization/energySet.cpp`
   - `src/core/nonlinearOptimization/energySet.h` only if required.
   - The workspace may already contain partial profiling/cache edits. Continue from
     the current file state; do not revert unrelated edits from the main agent or
     user.
2. Add private cache state to the `EnergySetBuffer` implementation in
   `energySet.cpp`.
   - `PatternSnapshot`: `valid`, `rows`, `cols`, `nnz`, `hash`, `outerIndices`,
     `innerIndices`.
   - `DynamicAssemblyCache`: `valid`, `SpMatD hessianTemplate`,
     `std::vector<SpMatI> termMappings`, `std::vector<PatternSnapshot> termPatterns`.
   - Keep this per `EnergySet` instance through `buffer_`; do not use global/static
     mutable cache.
   - Add temporary dynamic Hessian buffers if useful, but keep ownership inside
     `EnergySetBuffer`.
3. Add pattern helpers in the `energySet.cpp` anonymous namespace.
   - Hash dimensions plus outer/inner indices.
   - Hash is only a fast check; exact compare of outer/inner arrays is required.
   - Values never participate in pattern equality.
4. Split the current dynamic non-fixed path into reusable helpers.
   - Old/reference path: current `Ki -> triplets -> KiGlobal -> hess = hess + KiGlobal`.
   - New/cache path:
     - ensure cache has a full template containing fixed term patterns plus current
       dynamic term patterns,
     - rebuild all mappings whenever any dynamic term pattern changes,
     - clear `hess.valuePtr()`,
     - add fixed terms and dynamic terms with `ES::addSmallToBig(..., parallel=1)`.
5. Initial implementation must run in verify mode, controlled by env var:
   - `PGO_VERIFY_ENERGY_SET_HESSIAN_CACHE=1`
   - When enabled, compute both old and new assembly for the same input and compare:
     - dimensions,
     - sparse pattern,
     - value difference norm or max absolute difference.
   - On mismatch, throw with enough diagnostics: dimensions, nnz, first mismatch or
     value error magnitude.
6. Fast path should remain disabled unless explicitly enabled by a separate env var:
   - `PGO_ENABLE_ENERGY_SET_HESSIAN_CACHE=1`
   - With this unset, production result remains the old/reference path even if the
     cache is built and verified.
   - With this set, cache hits may return the new direct-add result; cache misses
     rebuild mappings and then direct-add.
7. Preserve profiling section names already present.
   - Keep the old sections around the reference path.
   - Add new sections with names like:
     - `energy_set.fgh.cache.rebuild_template`
     - `energy_set.fgh.cache.rebuild_mappings`
     - `energy_set.fgh.cache.add_dynamic`
     - `energy_set.fgh.cache.add_fixed`
     - `energy_set.fgh.cache.verify`
8. Handle nested `EnergySet`.
   - Each instance owns its own buffer/cache.
   - Do not assume term count or term type beyond `isHessianTopologyFixed()`.
9. Keep fallback conservative.
   - Any cache uncertainty invalidates and rebuilds.
   - If verification env is enabled and comparison fails, fail loudly.
   - If fast env is disabled, return the old path result.

Execution checklist:

1. Snapshot/evaluate all terms exactly once per `func_grad_hessian` call.
   - For fixed terms, keep the existing `func_grad`, `hessianInPlace`, and gradient
     accumulation behavior.
   - For dynamic terms, call `func_grad_hessian` once and store the returned local
     Hessian in a per-term buffer for later assembly. Do not call dynamic terms again
     for the reference/candidate comparison.
2. Build the reference Hessian only when needed.
   - Needed when `PGO_ENABLE_ENERGY_SET_HESSIAN_CACHE` is unset.
   - Needed when `PGO_VERIFY_ENERGY_SET_HESSIAN_CACHE=1`.
   - It must use the current reference route so existing profiling sections still
     measure `build_global_triplets`, `set_from_triplets`, and `add_global_hessian`.
3. Build the cached candidate when verify or fast mode is enabled.
   - Construct a full global template from the existing fixed template plus current
     dynamic term patterns.
   - Important: when any dynamic term pattern changes, rebuild the full template and
     all fixed/dynamic mappings together. Existing fixed mappings target `hessianAll`,
     not the expanded full template, so they cannot be reused against the new template.
   - For each dynamic local Hessian, create/update the mapping with
     `ES::small2Big(Ki, cache.hessianTemplate, energyDOFs[i], mapping)`.
   - Initialize candidate Hessian from `cache.hessianTemplate`, zero values, add fixed
     terms, then add dynamic terms with `ES::addSmallToBig(..., parallel=1)`.
4. Select the returned Hessian explicitly.
   - If fast mode is disabled: `hess` must be the reference Hessian.
   - If fast mode is enabled: `hess` may be the cached candidate.
   - If verify mode is enabled: compare reference and candidate regardless of which one
     is returned.
5. Compare sparse matrices strictly in verify mode.
   - Same dimensions.
   - Same compressed sparse pattern: `outerIndexPtr` and `innerIndexPtr`.
   - Values equal within a small numerical tolerance; report max absolute difference
     and index on mismatch.
6. Keep `EnergySet::hessian` and `gradient_hessian` unchanged unless the worker can
   prove they are on the same hot path. Phase 2 target is `func_grad_hessian` first.
7. Add no broad refactors. Helper functions are fine if they reduce risk in
   `func_grad_hessian`; avoid changing public API unless absolutely necessary.

Review checklist for the main agent:

1. Confirm no dynamic energy is evaluated more than once per term per
   `func_grad_hessian` call.
2. Confirm the old/reference path is still the default without env vars.
3. Confirm all mappings are rebuilt when the full template changes.
4. Confirm verify mode cannot silently pass on pattern mismatch.
5. Confirm the running tmux experiment was not touched by the worker.

Expected signal:

- `energy_set.fgh.dynamic.set_from_triplets`,
  `energy_set.fgh.dynamic.add_global_hessian`, and
  `energy_set.fgh.dynamic.build_global_triplets` should disappear or become rare on
  stable-pattern frames when `PGO_ENABLE_ENERGY_SET_HESSIAN_CACHE=1`.

Worker verification:

1. Build:
   ```bash
   conda run -n libpgo-mkl cmake --build build/base --target pypgo_core -j 8
   ```
2. Smoke import:
   ```bash
   conda run -n libpgo-mkl python -c 'import pypgo, pypgo.profiling as pr; pr.set_enabled(True); pr.reset(); print(pr.is_enabled())'
   ```
3. If feasible without disturbing the running tmux job, run a focused unit/smoke that
   exercises `EnergySet::func_grad_hessian` with
   `PGO_VERIFY_ENERGY_SET_HESSIAN_CACHE=1`.
4. Do not restart the long `dynamic_cubic_hermite` tmux job from the worker. The main
   agent will do runtime verification/profile after reviewing the patch.

Observed Phase 2 result:

- Baseline frames 106-110: 108.0 s/frame.
- Phase 2 fast frames 112-114: 86.8 s/frame including first cache rebuild.
- Stable cache-hit frames 113-114: about 81.5 s/frame.
- Old dynamic assembly sections disappeared on fast-path frames:
  - `energy_set.fgh.dynamic.build_global_triplets`
  - `energy_set.fgh.dynamic.set_from_triplets`
  - `energy_set.fgh.dynamic.add_global_hessian`
- First cache rebuild frame 112 still spent about:
  - `energy_set.fgh.cache.rebuild_template`: 5.7 s
  - `energy_set.fgh.cache.rebuild_mappings`: 7.8 s

## Phase 3: EnergySet Cache Rebuild Optimization

Goal: reduce the cost of Phase 2 cache misses, especially when contact active sets
change frequently. Phase 3 must not change the mathematical assembly result. It only
changes how the cached full-template mappings are rebuilt.

Target hot paths on cache miss:

- `energy_set.fgh.cache.rebuild_template`
- `energy_set.fgh.cache.rebuild_mappings`

Non-goals:

- Do not change the old/reference assembly path.
- Do not change when Phase 2 cache is enabled.
- Do not replace the full template builder with a custom row-merge sparse builder in
  this phase. Keep `setFromTriplets()` unless profiling after Phase 3 proves it is
  still the main rebuild cost.

Implementation plan:

1. Keep the write scope narrow:
   - Prefer `src/core/nonlinearOptimization/energySet.cpp`.
   - Modify `src/core/eigenSupport/EigenSupport.h/.cpp` only if adding a reusable
     helper is cleaner than a local anonymous-namespace helper.
   - Continue from the current workspace state; do not revert unrelated edits.
2. Cache fixed-template triplets or fixed row/column entries.
   - `hessianAll` contains all fixed-topology Hessian pattern entries and never changes
     for an `EnergySet` instance.
   - Store fixed template pattern entries once in `EnergySetBuffer` or
     `DynamicAssemblyCache`.
   - On cache rebuild, initialize the triplet list from the cached fixed entries, then
     append only current dynamic entries.
   - Keep the existing `hessianTemplate.setFromTriplets()` call for now.
3. Add a fast mapping builder for known local pattern.
   - Existing `ES::small2Big()` builds a triplet list and then calls
     `mapping.setFromTriplets()`.
   - Add a helper that constructs `mapping` with the exact sparse pattern of `Asmall`
     and fills only `mapping.valuePtr()[k]` with the global offset in `Abig`.
   - Keep exact topology validation: if a local nonzero cannot be found in `Abig`,
     throw `std::domain_error`.
   - It is acceptable to use serial offset lookup first; parallel value fill can be
     added if the implementation stays simple and safe.
4. Avoid recomputing fixed term mappings from local row/column search.
   - Existing fixed mappings `hessianMatrixMappings[i]` map each fixed local Hessian
     nonzero to an offset in `hessianAll`.
   - On cache rebuild, build one base mapping from `hessianAll` to the current
     `hessianTemplate`.
   - Compose fixed mappings:
     - `oldOffset = hessianMatrixMappings[i].valuePtr()[k]`
     - `newOffset = hessianAllToTemplateMapping.valuePtr()[oldOffset]`
     - `cache.termMappings[i].valuePtr()[k] = newOffset`
   - The composed mapping must keep the same sparse pattern as the fixed local Hessian.
   - Do not reuse `hessianMatrixMappings[i]` directly against `hessianTemplate`.
5. Dynamic mappings still rebuild from current dynamic Hessian patterns.
   - Use the new fast mapping helper for dynamic term mappings.
   - Dynamic term patterns are still the cache invalidation trigger.
6. Add fine-grained profiling around rebuild work:
   - `energy_set.fgh.cache.rebuild_template.fixed_entries`
   - `energy_set.fgh.cache.rebuild_template.dynamic_entries`
   - `energy_set.fgh.cache.rebuild_template.set_from_triplets`
   - `energy_set.fgh.cache.rebuild_mappings.base`
   - `energy_set.fgh.cache.rebuild_mappings.fixed_compose`
   - `energy_set.fgh.cache.rebuild_mappings.dynamic`
7. Preserve Phase 2 verify mode.
   - `PGO_VERIFY_ENERGY_SET_HESSIAN_CACHE=1` must still compare old/reference and
     cached Hessians.
   - Any mapping-composition bug should be caught by this verification.

Expected signal:

- On a first-rebuild frame similar to frame 112, `cache.rebuild_mappings` should drop
  materially from the previous 7.8 s baseline.
- `cache.rebuild_template` may improve modestly from avoiding repeated fixed-entry
  collection, but `setFromTriplets()` remains and may still dominate.
- Stable cache-hit frames should remain unchanged; Phase 3 targets cache misses.

Worker verification:

1. Build:
   ```bash
   conda run -n libpgo-mkl cmake --build build/base --target pypgo_core -j 8
   ```
2. Smoke import:
   ```bash
   conda run -n libpgo-mkl python -c 'import pypgo, pypgo.profiling as pr; pr.set_enabled(True); pr.reset(); print(pr.is_enabled())'
   ```
3. Run focused tests in default, verify, and fast modes:
   ```bash
   ctest --test-dir build/base -R '^EnergySetFused\\.SingleActiveSetBuildAndCorrectValues$' --output-on-failure
   PGO_VERIFY_ENERGY_SET_HESSIAN_CACHE=1 ctest --test-dir build/base -R '^EnergySetFused\\.SingleActiveSetBuildAndCorrectValues$' --output-on-failure
   PGO_ENABLE_ENERGY_SET_HESSIAN_CACHE=1 ctest --test-dir build/base -R '^EnergySetFused\\.SingleActiveSetBuildAndCorrectValues$' --output-on-failure
   ```
4. If cheap, run one IPC/contact focused test with verify mode.
5. Do not restart or kill the long `dynamic_cubic_hermite` tmux job. The main agent
   will run real-workload correctness and performance verification after patch review.

## Risks

- Hash-only reuse is unsafe; exact pattern comparison is required.
- Same `nnz` does not imply same sparse pattern.
- EnergySet mappings are invalid whenever the full template pattern changes.
- Fixed local patterns are stable, but their value offsets in the expanded full
  template are not stable. Fixed mappings must be composed through a
  `hessianAll -> hessianTemplate` mapping, not reused directly.
- Nested EnergySets produce inclusive timing, so section totals can double-count.
- Reduced fixed-DOF matrices must be compared after reduction and damping/diagonal
  insertion, not on the unreduced full Hessian.

## Rollback Criteria

Rollback Phase 1 if accepted frame behavior changes, `factorize/solve` fails after a
cache hit, or exact pattern comparison cannot be maintained.

Rollback Phase 2 if old/new Hessian verification fails or cached templates materially
increase memory.

Rollback Phase 3 if Phase 2 verify mode fails, fixed mapping composition cannot
preserve exact sparse patterns, or cache miss performance does not improve on a
representative rebuild frame.

## Phase 4: Row-Wise EnergySet Full Template Builder

Goal: reduce `energy_set.fgh.cache.rebuild_template`, whose remaining dominant
subsection after Phase 3 is `rebuild_template.set_from_triplets`.

Observed Phase 3 rebuild result on real `cubic_hermite` frames:

- Phase 3 cold rebuild frame 136:
  - `energy_set.fgh.cache.rebuild_template`: about 5.9 s
  - `energy_set.fgh.cache.rebuild_template.fixed_entries`: about 0.5 s
  - `energy_set.fgh.cache.rebuild_template.dynamic_entries`: about 0.8 s
  - `energy_set.fgh.cache.rebuild_template.set_from_triplets`: about 4.4 s
  - `energy_set.fgh.cache.rebuild_mappings`: about 4.0 s

This is the same optimization class as the solid deformation model DOF-layout
Hessian path:

- `src/core/solidDeformationModel/formulations/dof/dofLayout.h`
  documents `collectHessianBlockPairs -> buildCompressedHessianTemplate ->
  buildAllHessianBlockOffsets`.
- `dofLayout.cpp::buildCompressedHessianTemplate()` builds a compressed sparse
  template row-wise by collecting columns, sorting/uniquing per row, then calling
  `reserve/startVec/insertBackByOuterInner/finalize`, avoiding `setFromTriplets()`.

EnergySet cannot directly reuse that helper because its input is not block
structured. Its input is:

- fixed global pattern from `hessianAll`, already compressed and stable;
- current dynamic local Hessian patterns mapped through each term's DOF list.

Implementation plan:

1. Keep write scope narrow:
   - Primary file: `src/core/nonlinearOptimization/energySet.cpp`.
   - Do not modify public EnergySet APIs.
   - Do not modify or restart running tmux jobs from the worker.
2. Add a local row-wise template builder in `energySet.cpp`.
   - Input:
     - `nAll`
     - fixed template `hessianAll`
     - current dynamic term Hessians `buffer_->dynamicHessianMatrices`
     - `energyDOFs`
     - `potentialEnergies`
     - `energyCoeffs`
   - Output:
     - `ES::SpMatD hessianTemplate`, row-major, compressed.
   - Algorithm:
     - Allocate `std::vector<std::vector<StorageIndex>> rowColumns(nAll)`.
     - For each row in `hessianAll`, append existing fixed columns.
     - For each active dynamic term, append mapped global columns:
       - local row `it.row()` maps to `energyDOFs[i][it.row()]`
       - local col `it.col()` maps to `energyDOFs[i][it.col()]`
     - For each row, `sort` and `unique`.
     - Count total nnz.
     - Build template with `resize`, `reserve(totalNnz)`, `startVec(row)`,
       `insertBackByOuterInner(row, col) = 1.0`, `finalize`, and
       `makeCompressed`.
3. Preserve an optional reference builder for safety.
   - Keep the existing triplet + `setFromTriplets()` implementation as a local helper,
     at least behind a verification env var.
   - Add env var:
     - `PGO_VERIFY_ENERGY_SET_TEMPLATE_BUILDER=1`
   - When enabled, build both templates and compare exact sparse pattern:
     - rows, cols, nnz,
     - `outerIndexPtr`,
     - `innerIndexPtr`.
   - Values are not important for the template, but they should be initialized
     deterministically to `1.0`; if value comparison is cheap, allow it.
   - Throw `std::runtime_error` with diagnostics on mismatch.
4. Replace Phase 3 `setFromTriplets()` template construction on the fast path.
   - `energy_set.fgh.cache.rebuild_template` should use the new row-wise builder.
   - Keep profiling sections, but rename/add detailed sections so profile remains
     interpretable:
     - `energy_set.fgh.cache.rebuild_template.row_collect_fixed`
     - `energy_set.fgh.cache.rebuild_template.row_collect_dynamic`
     - `energy_set.fgh.cache.rebuild_template.row_sort_unique`
     - `energy_set.fgh.cache.rebuild_template.row_insert`
     - `energy_set.fgh.cache.rebuild_template.verify_triplets`
   - It is acceptable to leave old Phase 3 subsection names absent once the new
     builder replaces them.
5. Keep Phase 2/3 Hessian verification intact.
   - `PGO_VERIFY_ENERGY_SET_HESSIAN_CACHE=1` must still compute old/reference Hessian
     values and compare with cached assembly.
   - `PGO_VERIFY_ENERGY_SET_TEMPLATE_BUILDER=1` only verifies template pattern
     construction.
   - For a high-confidence real-workload check, run both verify env vars together
     once.
6. Do not parallelize first unless the serial row-wise builder is simple and correct.
   - Correctness first.
   - If the serial row-wise builder is correct and still costly, later work can
     parallelize row collection/sort/insert.
7. Preserve Phase 3 mapping optimizations.
   - Fixed mapping composition and fast dynamic mapping should continue to work against
     the row-wise template.
   - Existing Phase 3 verify should catch bad offsets.

Expected signal:

- On a cold cache rebuild frame, `cache.rebuild_template` should drop materially from
  the Phase 3 value of about 5.9 s if `setFromTriplets()` was the dominant cost.
- `cache.rebuild_mappings` should remain close to Phase 3 levels.
- Cache-hit frames should remain unchanged.

Worker verification:

1. Build:
   ```bash
   conda run -n libpgo-mkl cmake --build build/base --target pypgo_core -j 8
   ```
2. Smoke import:
   ```bash
   conda run -n libpgo-mkl python -c 'import pypgo, pypgo.profiling as pr; pr.set_enabled(True); pr.reset(); print(pr.is_enabled())'
   ```
3. Focused tests:
   ```bash
   ctest --test-dir build/base -R '^EnergySetFused\\.SingleActiveSetBuildAndCorrectValues$' --output-on-failure
   PGO_VERIFY_ENERGY_SET_TEMPLATE_BUILDER=1 PGO_VERIFY_ENERGY_SET_HESSIAN_CACHE=1 ctest --test-dir build/base -R '^EnergySetFused\\.SingleActiveSetBuildAndCorrectValues$' --output-on-failure
   PGO_VERIFY_ENERGY_SET_TEMPLATE_BUILDER=1 PGO_ENABLE_ENERGY_SET_HESSIAN_CACHE=1 ctest --test-dir build/base -R '^EnergySetFused\\.SingleActiveSetBuildAndCorrectValues$' --output-on-failure
   ```
4. If cheap, run one IPC/contact focused test with both verify env vars enabled.
5. Do not restart the long `dynamic_cubic_hermite` tmux job. The main agent will run
   real-workload correctness and performance verification.

Rollback Phase 4 if the row-wise builder fails exact pattern comparison against the
triplet builder, Phase 2 Hessian verify fails, or cold rebuild performance regresses
materially on the real workload.

## Phase 5: Parallel EnergySet Row-Wise Rebuild

Goal: parallelize the remaining serial pieces of the Phase 4 row-wise cache rebuild
without changing sparse patterns or cached Hessian values.

Phase 5A status:

- Implemented low-risk per-row parallel `sort + unique` in:
  - `src/core/nonlinearOptimization/energySet.cpp`
  - `src/core/solidDeformationModel/formulations/dof/dofLayout.cpp`
- Real contact-heavy frame 439 showed:
  - `energy_set.fgh.cache.rebuild_template.row_sort_unique`: 8 calls, 0.276 s,
    about 30 average cores.
- This confirmed the parallel row pass is correct and effective.

Observed remaining Phase 5A bottlenecks on frame 439:

- `energy_set.fgh.cache.rebuild_template.row_collect_dynamic`: 8 calls, 3.70 s
- `energy_set.fgh.cache.rebuild_template.row_insert`: 8 calls, 4.82 s
- `energy_set.fgh.cache.rebuild_mappings`: 8 calls, 12.56 s

Phase 5B targets:

1. Parallelize dynamic row collection in the row-wise template builder.
2. Parallelize mapping offset fill/composition in cache rebuild mappings.

Non-goals:

- Do not parallelize `row_insert` in this phase. Eigen's
  `startVec/insertBackByOuterInner` path should remain serial for now.
- Do not directly write Eigen sparse internals.
- Do not weaken Phase 2/4 verification modes.

Implementation plan:

1. Keep write scope narrow:
   - Primary file: `src/core/nonlinearOptimization/energySet.cpp`.
   - Do not modify public EnergySet APIs.
   - Do not restart or kill running tmux jobs from the worker.
2. Parallel dynamic row collection without locks.
   - Current serial path appends dynamic columns directly to `rowColumns[globalRow]`.
   - Replace it with a deterministic multi-pass approach:
     1. Count dynamic columns per global row.
     2. Prefix-sum row counts into row offsets.
     3. Fill a flat `dynamicColumns` array in parallel.
     4. Append each row's flat dynamic slice into `rowColumns[row]`.
   - Avoid per-row locks and avoid a full `vector<vector<...>>(nAll)` per thread.
   - It is acceptable to use per-row atomic counters for fill offsets if simpler, but
     the final row contents must be deterministic after sort/unique.
   - Keep/adjust profiling:
     - `energy_set.fgh.cache.rebuild_template.row_collect_dynamic`
     - optionally add:
       - `energy_set.fgh.cache.rebuild_template.row_collect_dynamic.count`
       - `energy_set.fgh.cache.rebuild_template.row_collect_dynamic.fill`
       - `energy_set.fgh.cache.rebuild_template.row_collect_dynamic.append`
3. Parallelize fast mapping offset fill.
   - Current helper builds a mapping with `Asmall`'s sparse pattern and fills
     `mapping.valuePtr()[k]` using `lower_bound` in `Abig`.
   - Keep the same mapping pattern and topology checks.
   - Build the sparse mapping pattern serially if that is simplest and safe.
   - Fill `mapping.valuePtr()` in parallel by row/nonzero.
   - This helper is used for:
     - `hessianAll -> hessianTemplate` base mapping;
     - dynamic term mappings.
4. Parallelize fixed mapping composition.
   - Current composition:
     - `smallToTemplate = smallToAll`
     - for each `k`, `smallToTemplate.valuePtr()[k] =
       allToTemplate.valuePtr()[smallToAll.valuePtr()[k]]`
   - Parallelize the value loop.
   - Preserve bounds checks.
5. Preserve verification:
   - `PGO_VERIFY_ENERGY_SET_TEMPLATE_BUILDER=1` must still exact-compare row-wise
     template vs triplet reference.
   - `PGO_VERIFY_ENERGY_SET_HESSIAN_CACHE=1` must still compare old/reference Hessian
     vs cached Hessian.
6. Preserve existing profiling section names so before/after comparisons remain easy.
   - Additional subsection names are fine.

Expected signal:

- `row_collect_dynamic` should drop materially from frame 439's 3.70 s / 8 rebuilds.
- `rebuild_mappings` should drop materially from frame 439's 12.56 s / 8 rebuilds.
- `row_insert` should remain roughly unchanged.
- Template/Hessian verify must pass on a real contact-heavy frame.

Worker verification:

1. Build:
   ```bash
   conda run -n libpgo-mkl cmake --build build/base --target pypgo_core -j 8
   ```
2. Smoke import:
   ```bash
   conda run -n libpgo-mkl python -c 'import pypgo, pypgo.profiling as pr; pr.set_enabled(True); pr.reset(); print(pr.is_enabled())'
   ```
3. Focused tests:
   ```bash
   PGO_VERIFY_ENERGY_SET_TEMPLATE_BUILDER=1 PGO_VERIFY_ENERGY_SET_HESSIAN_CACHE=1 ctest --test-dir build/base -R '^EnergySetFused\\.SingleActiveSetBuildAndCorrectValues$' --output-on-failure
   PGO_VERIFY_ENERGY_SET_TEMPLATE_BUILDER=1 PGO_ENABLE_ENERGY_SET_HESSIAN_CACHE=1 ctest --test-dir build/base -R '^EnergySetFused\\.SingleActiveSetBuildAndCorrectValues$' --output-on-failure
   PGO_VERIFY_ENERGY_SET_TEMPLATE_BUILDER=1 PGO_VERIFY_ENERGY_SET_HESSIAN_CACHE=1 ctest --test-dir build/base -R '^IPCContactEnergyGTest\\.AggregatedGradientHessianPreservesIPCFusion$' --output-on-failure
   ```
4. Do not run or restart the long `dynamic_cubic_hermite` tmux job. The main agent
   will run real-workload verify/profile.

Rollback Phase 5B if template/Hessian verification fails, parallel mapping fill
produces nondeterministic patterns or offsets, or real contact-heavy rebuild sections
regress materially.

## Phase 6: Newton Solver Diagnostics for Step Quality and Pattern Cost

Problem statement:

Recent contact-heavy `cubic_hermite` frames show many `STEP_TOO_SMALL` exits. The
new frame diagnostics already show that these frames are not contact/material
max-step clamps:

- `min_feasible_alpha = 1.0`
- `contact_clamp_count = 0`
- `min_line_search_alpha = 0.5^100`

This strongly suggests the line search is shrinking a poor Newton direction to
the simple line-search floor. However, the current profile does not directly
record the Newton direction quality, step scale, energy delta, line-search
iteration count, or damping value. Without these fields it is hard to distinguish:

- Hessian/linear-system direction is not descent.
- Direction is descent but too aggressive for the local nonlinear energy.
- Raw Newton step is already tiny.
- Damping is too weak or too strong.
- Sparse pattern rebuild/symbolic analyze cost is driven by pattern size/change.

Goal:

Add structured, per-solve diagnostics that are written into
`profile_dynamic.jsonl` through the existing `solver_diagnostics` payload. Keep
section timing and integer counters in the profiling module, but keep floating
point solver diagnostics in `SolveDiagnostics`.

Non-goals:

- Do not optimize contact assembly or EnergySet assembly in this phase.
- Do not change solver behavior except for recording diagnostics.
- Do not change default solver damping behavior. Existing config/env controls
  may remain available, but this phase should not tune damping.
- Do not restart the long `dynamic_cubic_hermite` tmux job from the worker.

Diagnostics to add:

1. Direction quality:
   - `last_grad_dot_dx`
     - `grad.dot(deltax)` after the linear solve and full-step expansion.
     - `>= 0` means the Newton direction is not a descent direction.
   - `last_raw_step_max_norm`
     - `||deltax||_inf` before feasible/max-step and line-search scaling.
   - `last_raw_step_norm`
     - `||deltax||_2` before scaling.

2. Accepted step and line search:
   - `last_accepted_step_max_norm`
     - Actual accepted step max norm after feasible alpha and line-search alpha.
   - `last_line_search_iterations`
     - Number of trial evaluations/iterations in the most recent line search.
   - `max_line_search_iterations`
     - Max line-search iterations in the solve.
   - `total_line_search_iterations`
     - Sum of line-search iterations in the solve.
   - `last_current_energy`
   - `last_accepted_energy`
   - `last_energy_delta = last_accepted_energy - last_current_energy`
     - Nonnegative or near-zero deltas explain line-search shrink/failure.

3. Damping and system scale:
   - `last_damping_value`
     - Actual diagonal shift added in `prepareReducedSystem`.
   - `last_active_system_nnz`
     - `activeSystemMatrix().nonZeros()` after compression.
   - `last_active_system_rows`
   - `last_active_system_cols`

4. Sparse pattern reuse:
   - `last_active_system_nnz`
   - `last_active_system_rows`
   - `last_active_system_cols`
     - Shape and nonzero count of the compressed active system.
   - `linear_solver_symbolic_rebuild_count`
   - `linear_solver_symbolic_reuse_count`
     - Counts per solve from `ensureLinearSolver()`.

Implementation plan:

1. Extend `SolveDiagnostics`.
   - File: `src/core/nonlinearOptimization/solver/common/solveDiagnostics.h`.
   - Add scalar fields listed above.
   - Add small record helpers:
     - `recordNewtonDirection(...)`
     - `recordLineSearch(..., iterations, currentEnergy, acceptedEnergy, acceptedStepMaxNorm)`
     - `recordDampingValue(double)`
     - `recordActiveSystemPattern(rows, cols, nnz, hash)`
     - `recordLinearSolverSymbolicReuse(bool reused)`
   - Preserve existing fields and reset semantics.

2. Record direction and step information.
   - File: `src/core/nonlinearOptimization/solver/newton/NewtonSolver.cpp`.
   - After `expandReducedStep()` succeeds and before `takeStep()`, record:
     - raw step norms,
     - `grad.dot(deltax)`.
   - In `runLineSearchStep()`, pass `ret.iterations` and energy/step values into
     diagnostics.
   - In `prepareReducedSystem()`, record damping value even when damping is off
     (`0.0`).
   - In `ensureLinearSolver()`, compute/record active pattern hash and whether
     the existing solver was reused or symbolic analysis was rebuilt.

3. Expose diagnostics to Python.
   - Files:
     - `src/python/pypgo/solver/core.cpp`
     - `src/python/pypgo/simulation/dynamic/core.cpp`
     - `pypgo/solver/result.py`
   - Add the new fields to the diagnostics dictionaries and dataclass.
   - Existing unknown-key filtering in `SolveDiagnostics.from_dict()` should
     continue to make this backward compatible.

4. Preserve profile output.
   - File: `pypgo/tools/sim/_runners.py`.
   - No new profile schema is needed if `_solver_diagnostics()` uses
     `asdict(result.diagnostics)`.
   - Confirm `profile_dynamic.jsonl` rows include the new fields under:
     - `solver_diagnostics.diagnostics`
     - `stage_diagnostics[*].diagnostics`

5. Verification:
   - Build:
     ```bash
     cmake --build build/base -j 8
     ```
   - Smoke import:
     ```bash
     LD_LIBRARY_PATH=/root/miniconda3/envs/libpgo-mkl/lib:${LD_LIBRARY_PATH:-} \
       /root/miniconda3/envs/libpgo-mkl/bin/python - <<'PY'
     import pypgo
     import pypgo.profiling as pr
     pr.set_enabled(True)
     pr.reset()
     print(pr.is_enabled())
     PY
     ```
   - Focused dynamic/solver tests if available:
     ```bash
     ctest --test-dir build/base -R 'Newton|dynamicStepper' --output-on-failure
     ```
   - Real workload profile, run by the main agent:
     - Use an existing `cubic_hermite` checkpoint.
     - Run one frame with `PGO_PROFILE_DYNAMIC=1`.
     - Confirm JSON includes:
       - `last_grad_dot_dx`
       - `last_raw_step_max_norm`
       - `last_accepted_step_max_norm`
       - `last_line_search_iterations`
       - `last_energy_delta`
       - `last_damping_value`
       - `linear_solver_symbolic_rebuild_count`
       - `linear_solver_symbolic_reuse_count`

Expected interpretation:

- `STEP_TOO_SMALL` with `last_grad_dot_dx >= 0`:
  Hessian/direction quality problem. Investigate adaptive damping,
  modified Cholesky, or trust-region logic.
- `STEP_TOO_SMALL` with `last_grad_dot_dx < 0` and
  `last_line_search_iterations` at the max:
  Direction is descent but too aggressive or energy is highly non-smooth near
  active contact changes. Investigate line search policy or frozen active-set
  assumptions.
- `STEP_TOO_SMALL` with tiny `last_raw_step_max_norm`:
  Linear solve produced a near-zero step; check gradient scale, damping scale,
  and system conditioning.
- High symbolic rebuild count with low pattern hash reuse:
  Pattern changes are structural; LRU solver cache may not help.
- High symbolic rebuild count with repeated pattern hash:
  Pattern LRU is a good candidate.

Rollback:

Rollback Phase 6 if diagnostics change solver behavior, introduce measurable
runtime overhead with profiling disabled, or break Python result deserialization.

## Phase 7: Simple Line Search Floating-Point Plateau Acceptance

Problem statement:

Phase 6 diagnostics on real `cubic_hermite` frames show repeated
`STEP_TOO_SMALL` exits with:

- `last_grad_dot_dx < 0`
- `min_feasible_alpha = 1.0`
- `contact_clamp_count = 0`
- `material_clamp_count = 0`
- `last_line_search_iterations = 100`
- `last_energy_delta = 0.0`

This means the Newton direction is a descent direction and is not being clamped
by contact/material max-step logic. The simple line search is shrinking because
it requires strict `trialEnergy < currentEnergy`. On large-magnitude energies,
the predicted decrease can be below the floating-point resolution of the
reported total energy, so a numerically equivalent descent trial is treated as a
failure and the accepted step collapses to the line-search floor.

Goal:

Make `SimpleLineSearchPolicy` robust to floating-point energy plateaus for true
descent directions, while preserving rejection of non-descent directions and
actual energy increases.

Non-goals:

- Do not tune damping or change the Hessian/linear solve.
- Do not change contact/material max-step logic.
- Do not change golden, Brent, or Armijo backtracking policies.
- Do not hide real non-descent directions.

Implementation plan:

1. Update `SimpleLineSearchPolicy::search()`.
   - File:
     `src/core/nonlinearOptimization/solver/newton/newtonLineSearchPolicy.cpp`.
   - Compute `gradDotDirection = ctx.gradient.dot(ctx.direction)`.
   - Continue accepting strict decreases immediately.
   - If `gradDotDirection < 0`, also accept
     `trialEnergy <= currentEnergy + energyTolerance`, where
     `energyTolerance` is a small relative floating-point tolerance scaled by
     `max(1, abs(currentEnergy))`.
   - Never accept non-finite trial energies.
   - Use `ctx.maxIterations` when it is positive; otherwise fall back to
     `params_.maxIterations`.
   - Keep `result.alpha` and `result.energy` from the same evaluated trial.
     Do not return a post-shrink alpha paired with the previous trial energy.

2. Add focused regression tests.
   - Prefer the existing Newton solver gtest target to avoid new build plumbing.
   - Cover:
     - Descent direction with equal total energy accepts `alpha = 1` in one
       iteration.
     - Non-descent direction with equal total energy does not use the plateau
       tolerance and exhausts the configured iteration limit.
     - Failure path returns the last evaluated alpha and corresponding energy,
       not one extra shrink.

3. Verification:
   - Build:
     ```bash
     cmake --build build/base -j 8
     ```
   - Focused tests:
     ```bash
     ctest --test-dir build/base -R 'Newton|newton' --output-on-failure
     ```
   - Real workload:
     - Run one frame from a recent `cubic_hermite` checkpoint.
     - Confirm the previous failure signature changes from:
       - `last_line_search_iterations = 100`
       - `min_line_search_alpha = 0.5^100`
       - `last_accepted_step_max_norm ~= 0`
     - To:
       - small line-search iteration count, ideally 1
       - finite/non-collapsed accepted step
       - no `STEP_TOO_SMALL` caused by line-search floor on the same frame.

Risks:

- The tolerance must be small enough not to accept meaningful energy increases.
- Equal-energy acceptance can still produce slow progress if the Newton step is
  genuinely too small; diagnostics should distinguish that by raw step size and
  gradient norm.
- If active contact changes make the energy discontinuous, this phase only fixes
  floating-point plateau handling; it is not a replacement for trust-region or
  adaptive damping.

Rollback:

Rollback Phase 7 if tests show non-descent directions being accepted, real
workload energy increases beyond the tolerance are accepted, or the fix masks a
different solver failure mode.

## Phase 8: Explicit Line Search Result Status

Problem statement:

Phase 7 fixed the immediate floating-point plateau bug, but it still relies on
the Newton solver re-checking line-search success from returned scalar values.
That makes the contract between a `NewtonLineSearchPolicy` and
`NewtonSolver::takeStep()` implicit:

- The policy returns `alpha`, `energy`, and `iterations`.
- The solver infers whether the policy succeeded by checking the returned
  energy again.

This is fragile because the policy owns the method-specific acceptance logic.
After Phase 7, the same floating-point plateau rule had to be duplicated in the
outer solver guard. Future line-search policies can easily drift from that
outer inference.

Goal:

Make line-search success explicit in `NewtonLineSearchResult`, so each policy
declares whether the returned trial was accepted, failed to find an acceptable
step, or hit a non-finite result. The outer Newton solver should consume this
status instead of reverse-engineering policy success from energy alone.

Non-goals:

- Do not change the numerical acceptance rule from Phase 7.
- Do not tune damping, sparse solves, contact max-step, or Hessian assembly.
- Do not remove the outer solver's responsibility for `StepTooSmall`,
  `NonFinite`, diagnostics, and fallback convergence.
- Do not change Python result schemas unless a new diagnostic field is needed.

Design:

Add an explicit status enum:

```cpp
enum class NewtonLineSearchStatus
{
  Accepted,
  NoAcceptableStep,
  NonFiniteEnergy,
  NonFiniteAlpha,
  EvaluationFailed,
};
```

Extend `NewtonLineSearchResult`:

```cpp
struct NewtonLineSearchResult
{
  double alpha = 1.0;
  double energy = std::numeric_limits<double>::quiet_NaN();
  int iterations = 0;
  NewtonLineSearchStatus status = NewtonLineSearchStatus::NoAcceptableStep;

  bool accepted() const;
  bool nonFinite() const;
};
```

Semantic contract:

- `Accepted`
  - `alpha` and `energy` correspond to the accepted trial.
  - The outer solver may apply the step after its own step-size checks.
- `NoAcceptableStep`
  - The policy exhausted its search without finding an accepted trial.
  - `alpha` and `energy` are diagnostic values from the last evaluated trial,
    when available.
- `NonFiniteEnergy`
  - A trial energy was non-finite.
  - The outer solver maps this to `SolveStatus::NonFinite`.
- `NonFiniteAlpha`
  - The policy/helper returned a non-finite alpha.
  - The outer solver maps this to `SolveStatus::NonFinite`.
- `EvaluationFailed`
  - Reserved for evaluators that report an error code instead of only writing
    energy. If the current evaluator interface does not expose failures in a
    useful way, keep this status available but unused.

Implementation plan:

1. Update line-search result types.
   - File:
     `src/core/nonlinearOptimization/solver/newton/newtonLineSearchPolicy.h`.
   - Add `NewtonLineSearchStatus`.
   - Extend `NewtonLineSearchResult` with `status`, `accepted()`, and
     `nonFinite()`.
   - Keep Phase 7 helper functions:
     - `lineSearchEnergyFpTolerance`
     - `lineSearchEnergyWithinFpTolerance`
     - `lineSearchAcceptsEnergy`

2. Update policy implementations.
   - File:
     `src/core/nonlinearOptimization/solver/newton/newtonLineSearchPolicy.cpp`.
   - `SimpleLineSearchPolicy`:
     - Set `Accepted` only when `lineSearchAcceptsEnergy(...)` returns true.
     - Set `NoAcceptableStep` when the loop exhausts.
     - Set `NonFiniteEnergy` on non-finite trial energy.
     - Preserve alpha/energy pairing for every evaluated trial.
   - `BacktrackingLineSearchPolicy`, `GoldenLineSearchPolicy`,
     `BrentsLineSearchPolicy`:
     - Wrap helper results through a shared classification helper.
     - If alpha is non-finite, return `NonFiniteAlpha`.
     - If energy is non-finite, return `NonFiniteEnergy`.
     - If accepted by `lineSearchAcceptsEnergy(...)`, return `Accepted`.
     - Otherwise return `NoAcceptableStep`.

3. Update Newton solver consumption.
   - File:
     `src/core/nonlinearOptimization/solver/newton/NewtonSolver.cpp`.
   - `runLineSearchStep()` should copy `ret.status` into `StepAcceptance`.
   - `StepAcceptance::nonFinite()` should include line-search result
     non-finite status, or map it through the existing `NonFiniteReason`.
   - `LineSearchStrategy::takeStep()` should:
     - Reject non-finite status as `SolveStatus::NonFinite`.
     - Reject `NoAcceptableStep` as `SolveStatus::LineSearchFailed`.
     - Apply the step only for `Accepted`.
     - Keep the existing `StepTooSmall` path after accepted status.
   - The outer solver should no longer infer line-search failure using
     `acceptedEnergy > currentEnergy`, except possibly as an assertion/logging
     sanity check in debug-only code.

4. Tests:
   - Existing Phase 7 tests should continue to pass.
   - Add focused coverage for status semantics:
     - Simple descent plateau returns `Accepted`.
     - Simple non-descent plateau returns `NoAcceptableStep`.
     - Simple non-finite trial returns `NonFiniteEnergy`.
     - Outer Newton solver maps `NoAcceptableStep` to
       `SolveStatus::LineSearchFailed`.
     - Outer Newton solver still maps accepted but tiny step to
       `SolveStatus::StepTooSmall`.
   - Keep the real bug regression:
     - FP-equivalent descent accepted by policy is not rejected by `takeStep`.
     - Non-descent FP-equivalent energy is rejected.

5. Verification:
   - Build:
     ```bash
     cmake --build build/base -j 8
     ```
   - Focused tests:
     ```bash
     /root/miniconda3/envs/libpgo-mkl/bin/ctest --test-dir build/base \
       -R 'SimpleLineSearchPolicy|NewtonSolverGTest|NewtonOptimizer' --output-on-failure
     ```
   - Real workload:
     - Re-run one frame from
       `cubic_hermite/checkpoints/state0483.npz` to reproduce frame 484.
     - Expected:
       - `solver_status = CONVERGED`
       - `last_line_search_iterations` stays small, not 100.
       - accepted step remains non-collapsed.

Risks:

- Existing tests or callers may aggregate-initialize `NewtonLineSearchResult`;
  update them carefully so field order changes do not silently alter meaning.
- Golden/Brent helper methods may return energies that do not satisfy the
  Newton acceptance helper even if historically they were tolerated. Preserve
  current behavior only if a test demonstrates that behavior is intentional.
- Diagnostics should still record the last evaluated alpha/energy for failed
  searches so profiling remains useful.

Rollback:

Rollback Phase 8 if line-search statuses disagree with existing solver results
on focused tests, if accepted steps are applied for non-accepted statuses, or if
the real frame 484 regression returns to `STEP_TOO_SMALL`/`LINE_SEARCH_FAILED`.

## Phase 9A: Contact Hessian Pullback Profiling Drill-Down

Problem statement:

After the line-search correctness fix, recent `cubic_hermite` frames converge
reliably. The dominant non-Pardiso contact cost is now:

- `contact.adapter.hessian_direct`
- `contact.adapter.pullback_hessian`

However, `contact.adapter.hessian_direct` is an inclusive section around the
whole IPC adapter Hessian path, and `contact.adapter.pullback_hessian` is almost
as large as it. The current implementation of
`SurfaceDofMap::pullbackHessian()` is:

```cpp
simulationHessian = W.transpose() * surfaceHessian * W;
```

where `W = surfaceFromSimulationDispMap_`. This is likely dominated by Eigen
sparse matrix multiplication and sparse allocation/compression, not by the IPC
pair kernels themselves. Before replacing it with a custom parallel pullback,
we need finer evidence:

- Is the first multiply `surfaceHessian * W` slow?
- Is the second multiply `W^T * tmp` slow?
- Is allocation/compression the slow part?
- How large is the expansion implied by each surface scalar DOF row of `W`?
- Does the resulting simulation Hessian pattern repeat enough to justify a
  pullback pattern/value-fill cache?

Goal:

Add low-risk profiling and counters inside `SurfaceDofMap::pullbackHessian()`
without changing numerical behavior. Use the new data to choose between:

- a simple cached transpose/buffer optimization,
- a custom parallel triplet pullback,
- or a cached pullback pattern/value-fill plan.

Non-goals:

- Do not optimize the IPC pair kernels in this phase.
- Do not change `surfaceHessian` or `simulationHessian` values/patterns.
- Do not replace Eigen sparse multiplication yet.
- Do not change contact active set generation.
- Do not restart the long tmux job from the worker; the main agent will handle
  real workload profiling.

Implementation plan:

1. Add profiling section names.
   - Preferred file:
     `src/core/contact/ipc/profiling/surfaceIPCProfiling.h`.
   - Add sections:
     - `contact.adapter.pullback_hessian.validate`
     - `contact.adapter.pullback_hessian.multiply_surface_hessian_map`
     - `contact.adapter.pullback_hessian.multiply_transpose_tmp`
   - Add counters:
     - `contact.adapter.pullback_hessian.map_rows`
     - `contact.adapter.pullback_hessian.map_cols`
     - `contact.adapter.pullback_hessian.map_nnz`
     - `contact.adapter.pullback_hessian.surface_hessian_nnz`
     - `contact.adapter.pullback_hessian.tmp_nnz`
     - `contact.adapter.pullback_hessian.simulation_hessian_nnz`
     - `contact.adapter.pullback_hessian.map_row_nnz_min`
     - `contact.adapter.pullback_hessian.map_row_nnz_max`
     - `contact.adapter.pullback_hessian.map_row_nnz_total`
     - `contact.adapter.pullback_hessian.map_row_nnz_nonzero_rows`
   - If there is already a naming pattern for counters, follow it.

2. Instrument `SurfaceDofMap::pullbackHessian()`.
   - File: `src/core/contact/surfaceDofMap.cpp`.
   - Preserve current semantics exactly.
   - Split:
     ```cpp
     tmp = surfaceHessian * surfaceFromSimulationDispMap_;
     simulationHessian = surfaceFromSimulationDispMap_.transpose() * tmp;
     ```
   - Wrap each multiply with the new profiling sections.
   - Record counters for dimensions and nnz after each step.
   - Compute map row-nnz stats once in the constructor and store them in
     `SurfaceDofMap`, because `W` is fixed:
     - min row nnz,
     - max row nnz,
     - total row nnz,
     - nonzero row count.
   - Report those cached stats from each pullback call.

3. Optional debug-only verification.
   - If a cheap local test exists, compare the split multiply result against the
     old single-expression result on a small matrix.
   - Do not run full real workload from the worker.

4. Tests:
   - Existing `SurfaceDofMapGTest.PullsBackSurfaceHessian` should still pass.
   - If practical, extend it to cover a non-identity sparse map with multiple
     entries per surface row.
   - Run:
     ```bash
     cmake --build build/base --target surfaceDofMap_gtest -j 8
     /root/miniconda3/envs/libpgo-mkl/bin/ctest --test-dir build/base \
       -R 'SurfaceDofMapGTest|IPCContactEnergyGTest' --output-on-failure
     ```

5. Real workload validation, run by the main agent:
   - Stop/restart `dynamic_cubic_hermite` around rebuild.
   - Run one recent `cubic_hermite` frame or let the tmux job produce 1-2 rows.
   - Confirm profile rows include the new sections/counters.
   - Summarize:
     - first multiply wall/core,
     - second multiply wall/core,
     - `W` row nnz distribution,
     - `surfaceHessian/tmp/simulationHessian` nnz,
     - whether custom pullback or pattern cache is the best Phase 9B.

Expected interpretation:

- If `multiply_surface_hessian_map` dominates and `W` row nnz is modest:
  custom parallel pullback over surface Hessian nonzeros is a good candidate.
- If `multiply_transpose_tmp` dominates and output pattern repeats:
  cached simulation Hessian pattern/value-fill plan is the best candidate.
- If `W` row nnz is high and triplet expansion would explode:
  avoid naive thread-local triplets; build a planned sparse accumulator.
- If both multiply sections are small but parent section is large:
  allocation/compression or hidden Eigen temporaries need separate measurement.

Rollback:

Rollback Phase 9A if the split multiply changes Hessian results, if profiling
overhead is measurable with profiling disabled, or if existing contact Hessian
tests fail.

## Phase 9B: Experimental Parallel Surface Hessian Pullback

Phase 9A result:

One real `cubic_hermite` frame from `state0500.npz` to frame 501 produced:

- `contact.adapter.pullback_hessian`: 40.047s wall, avg core 1.00, 17 calls.
- `contact.adapter.pullback_hessian.multiply_surface_hessian_map`: 2.649s wall.
- `contact.adapter.pullback_hessian.multiply_transpose_tmp`: 37.383s wall.
- `W` shape: 12642 x 72816, nnz 808944.
- `W` row nnz: min 16, max 64, total 808944.
- `surfaceHessian` nnz: 194796 per call max.
- `tmp = surfaceHessian * W` nnz: 3132864 per call max.
- `simulationHessian` nnz: 34056576 per call max.

The second multiply `W^T * tmp` accounts for nearly all pullback time and runs
single-threaded. Optimizing IPC pair kernels first would miss the current
bottleneck.

Goal:

Add an opt-in custom pullback path for the second multiply:

```cpp
tmp = surfaceHessian * W;
simulationHessian = customParallelTransposeMapMultiply(tmp);
```

The default path must remain the Eigen sparse multiply until the custom path is
verified on real frames. Enable the custom path only with:

```bash
PGO_CONTACT_PULLBACK_CUSTOM=1
```

Non-goals:

- Do not change the first multiply `surfaceHessian * W` in this phase.
- Do not change IPC active set generation.
- Do not make the custom path default yet.
- Do not add a persistent output-pattern cache yet; this phase tests the row
  parallel builder first.

Algorithm:

1. Precompute fixed `W` row data in `SurfaceDofMap`.
   - Existing `W = surfaceFromSimulationDispMap_` is fixed for the map.
   - Keep per-surface-row entries:
     `(simulation_col, weight)`.
   - Build the inverse adjacency:
     `simulation_col -> [(surface_row, weight)]`.
   - This lets each output row of `W^T * tmp` be assembled independently.

2. For each simulation output row `i`, in parallel:
   - Look up all `(surface_row r, weight w_ri)` entries from the inverse
     adjacency.
   - For every nonzero `(r, j, tmp_rj)` in `tmp` row `r`, append
     `(j, w_ri * tmp_rj)` into a local row buffer.
   - Sort the local row buffer by column.
   - Reduce duplicate columns by summing values.
   - Drop exact zeros or near-zero values only if the existing Eigen path does
     so. Prefer no numerical pruning in the first implementation.

3. Build the output sparse matrix from sorted row buffers.
   - Keep row construction parallel.
   - Fill the Eigen row-major sparse matrix after row nnz is known.
   - Avoid global triplets if practical, because the output can have more than
     34M nonzeros and triplets add substantial peak memory.

4. Profiling:
   - Add sections:
     - `contact.adapter.pullback_hessian.custom_row_build`
     - `contact.adapter.pullback_hessian.custom_output_fill`
   - Add counters:
     - `contact.adapter.pullback_hessian.custom_enabled`
     - `contact.adapter.pullback_hessian.custom_output_nnz`
     - `contact.adapter.pullback_hessian.custom_contribution_count`
     - `contact.adapter.pullback_hessian.custom_active_output_rows`

Implementation plan:

1. Update profiling names.
   - File:
     `src/core/contact/ipc/profiling/surfaceIPCProfiling.h`.

2. Extend `SurfaceDofMap`.
   - Files:
     - `src/core/contact/surfaceDofMap.h`
     - `src/core/contact/surfaceDofMap.cpp`
   - Add small structs for fixed map entries:
     - simulation column,
     - surface row,
     - interpolation weight.
   - Build:
     - `surfaceFromSimulationDispMapRows_`
     - `simulationToSurfaceDispMapRows_`
   - Validate this once in tests with the existing non-identity W fixture.

3. Add env-gated custom path.
   - In `pullbackHessian()`, keep the Phase 9A first multiply and counters.
   - If `PGO_CONTACT_PULLBACK_CUSTOM=1`, call the custom second multiply.
   - Otherwise keep:
     `simulationHessian = surfaceFromSimulationDispMap_.transpose() * tmp`.
   - Keep the existing Phase 9A profiling section for
     `multiply_transpose_tmp`; either nest custom sections inside it or record
     a sibling custom section so real profile rows stay comparable.

4. Correctness tests.
   - Extend `SurfaceDofMapGTest.PullsBackSurfaceHessian`:
     - compute Eigen reference,
     - enable `PGO_CONTACT_PULLBACK_CUSTOM`,
     - compute custom result,
     - compare dense values with `isApprox(..., 1e-12)`,
     - compare dimensions and nnz sanity.
   - Add a second fixture with multiple surface rows contributing to the same
     simulation output row, so duplicate-column reduction is tested.

5. Verification.
   - Build:
     ```bash
     cmake --build build/base --target surfaceDofMap_gtest -j 8
     ```
   - Tests:
     ```bash
     /root/miniconda3/envs/libpgo-mkl/bin/ctest --test-dir build/base \
       -R 'SurfaceDofMapGTest|IPCContactEnergyGTest' --output-on-failure
     ```
   - Real frame A/B:
     - Stop the long tmux job around rebuild.
     - Run one frame from `state0500.npz` with default path.
     - Run the same one frame with `PGO_CONTACT_PULLBACK_CUSTOM=1`.
     - Compare:
       - solver status and iteration count,
       - final gradient max norm,
       - accepted energy,
       - `simulation_hessian_nnz`,
       - pullback wall time,
       - avg core.

Expected result:

The custom path should reduce `multiply_transpose_tmp` wall time and increase
its avg core above 1.0. If output fill becomes the new serial bottleneck, Phase
9C should replace Eigen insert fill with a lower-level compressed sparse fill or
a reusable output pattern/value buffer.

Risks:

- Row-local sort/reduce may allocate heavily. The implementation must avoid a
  second global triplet copy if possible.
- If a few simulation rows receive most contributions, row-parallel load
  balance may be uneven. Use TBB auto partitioning first; tune grain size only
  after profiling.
- Floating-point summation order will differ from Eigen. Tests should use
  approximate comparison, not bytewise equality.
- Peak memory is already high on the real workload. Watch memory before making
  this path default.

Rollback:

Disable `PGO_CONTACT_PULLBACK_CUSTOM` or revert Phase 9B if the custom path
changes solver results beyond tolerance, increases peak memory excessively, or
does not improve the real-frame pullback wall time.

## Phase 9C: K-Way Merge Pullback Row Builder and Reusable Buffers

Phase 9B result:

The opt-in custom pullback path reduced the real frame 501 contact Hessian
pullback from 40.047s to 18.460s:

- `contact.adapter.pullback_hessian.multiply_transpose_tmp`: 37.383s -> 15.831s.
- `contact.adapter.pullback_hessian.custom_row_build`: 10.031s, avg core 16.90.
- `contact.adapter.pullback_hessian.custom_output_fill`: 4.846s, avg core 1.01.
- `custom_contribution_count`: 200,441,088 max per pullback.
- `custom_output_nnz`: 34,056,576 max per pullback.

This means the current row builder creates about 5.9x more intermediate
contributions than final nonzeros and then uses `std::sort` to reduce duplicate
columns. The next low-risk optimization is to avoid the per-row append/sort
phase.

Goal:

Optimize the Phase 9B custom row builder while preserving the opt-in
`PGO_CONTACT_PULLBACK_CUSTOM=1` behavior:

1. Replace per-row `append all contributions -> sort by column -> reduce` with
   a k-way merge over already sorted `tmp` rows.
2. Reuse per-row buffers/workspace across pullback calls to reduce allocator
   overhead and heap retention.

Non-goals:

- Do not make the custom pullback default.
- Do not change `tmp = surfaceHessian * W`.
- Do not add a persistent output pattern cache in this phase.
- Do not change solver/contact numerical behavior.
- Do not optimize `custom_output_fill` here except for small mechanical changes
  needed by the new row buffer representation.

Algorithm:

For each simulation output row `i`, the current custom builder computes:

```text
H(i, j) = sum_r W(r, i) * tmp(r, j)
```

where the inverse adjacency gives all surface rows `r` with nonzero `W(r, i)`.
Because `tmp` is row-major compressed, every `tmp` row is already sorted by
column. Therefore each adjacent surface row contributes a sorted stream of
weighted `(j, value)` entries.

Use a row-local k-way merge:

1. For output row `i`, initialize one cursor per adjacent surface row with a
   nonempty `tmp` row.
2. Keep cursors ordered by current column.
3. Repeatedly consume the smallest current column `j`.
4. Sum all cursors whose current column is `j`:

   ```text
   value += W(r, i) * tmp(r, j)
   ```

5. Advance consumed cursors and reinsert them if they still have entries.
6. Append only the reduced `(j, value)` to the output row buffer.

Expected effect:

- Intermediate row buffer size should fall from contribution count toward final
  output nnz.
- Per-row `std::sort` disappears.
- Peak temporary memory should decrease.
- `custom_row_build` wall time should improve, especially on contact-heavy
  frames.

Implementation plan:

1. Add profiling names.
   - File:
     `src/core/contact/ipc/profiling/surfaceIPCProfiling.h`.
   - Add sections:
     - `contact.adapter.pullback_hessian.custom_row_merge`
     - `contact.adapter.pullback_hessian.custom_workspace_prepare`
   - Add counters:
     - `contact.adapter.pullback_hessian.custom_merge_stream_count`
     - `contact.adapter.pullback_hessian.custom_merge_output_nnz`
     - `contact.adapter.pullback_hessian.custom_workspace_reused_rows`

2. Refactor custom row build.
   - Files:
     - `src/core/contact/surfaceDofMap.h`
     - `src/core/contact/surfaceDofMap.cpp`
   - Keep `customParallelTransposeMapMultiply(...)` as the top-level custom
     path.
   - Replace the append/sort/reduce body with a helper such as:

     ```cpp
     buildMergedOutputRow(tmp, outputRow, rowBuffer, scratchStreams)
     ```

   - Use a small row-local cursor struct:
     - current column,
     - current index/pointer in `tmp` row,
     - end index/pointer,
     - weight.
   - Prefer a simple binary heap (`std::push_heap`/`std::pop_heap`) or sorted
     vector depending on local simplicity. The heap comparator should make the
     smallest column pop first.
   - Special-case:
     - zero adjacent rows: clear row buffer,
     - one adjacent row: copy weighted sorted entries directly,
     - small number of streams: the heap path is fine if simpler.

3. Reuse buffers/workspace.
   - Add mutable workspace to `SurfaceDofMap`, guarded by a mutex only if
     `pullbackHessian()` can be called concurrently on the same map. If there is
     no existing concurrent use contract, keep the workspace local in the custom
     function but reuse allocation with a `thread_local` scratch vector inside
     each TBB worker.
   - Preferred low-risk approach:
     - Keep `rowBuffers` as one vector per output row for now, but do not store
       unreduced contributions.
     - Use `tbb::enumerable_thread_specific` scratch streams/heaps so every
       worker reuses its scratch vectors across rows within one pullback call.
     - Reserve row output buffers using the previous capacity if available; do
       not call `shrink_to_fit`.
   - Avoid adding a long-lived 72k-row mutable workspace unless tests or code
     inspection confirm `SurfaceDofMap` is not used concurrently.

4. Preserve correctness and diagnostics.
   - Keep `custom_contribution_count` as the number of conceptual weighted
     contributions scanned from `tmp`.
   - Keep `custom_output_nnz` equal to final reduced output nnz.
   - Add `custom_merge_output_nnz` equal to reduced row buffer total.
   - Existing `custom_output_nnz` and `simulation_hessian_nnz` must still match.
   - Floating-point order will differ from append/sort and Eigen; compare with
     approximate tolerance.

5. Tests:
   - Existing `CustomPullbackMatchesEigenAndReducesDuplicateColumns` must pass.
   - Add or extend a fixture where:
     - multiple adjacent surface rows contribute to the same output column,
     - one output row has no adjacent streams,
     - one output row has exactly one adjacent stream.
   - Check new counters are recorded when `PGO_CONTACT_PULLBACK_CUSTOM=1`.

6. Verification:
   - Build:
     ```bash
     cmake --build build/base --target surfaceDofMap_gtest -j 8
     ```
   - Tests:
     ```bash
     /root/miniconda3/envs/libpgo-mkl/bin/ctest --test-dir build/base \
       -R 'SurfaceDofMapGTest|IPCContactEnergyGTest' --output-on-failure
     ```
   - Real frame profile, run by main agent:
     - Stop the long tmux job around rebuild.
     - Run frame 501 from `state0500.npz` with
       `PGO_CONTACT_PULLBACK_CUSTOM=1`.
     - Compare against Phase 9B:
       - solver status and iterations,
       - final gradient max norm and accepted energy,
       - `custom_row_build`,
       - `custom_row_merge`,
       - `custom_output_fill`,
       - `custom_contribution_count`,
       - `custom_output_nnz`,
       - peak/current memory.

Expected result:

`custom_row_build` should improve from 10.031s and temporary row-buffer memory
should decrease. If it does not improve, the likely cause is heap overhead or
memory-bandwidth limits; then Phase 9D should skip further row-builder tuning
and focus on output pattern/value cache.

Risks:

- K-way merge changes floating-point summation order.
- A heap per output row may be slower for very small stream counts; special
  cases for 0/1 streams reduce this risk.
- Long-lived mutable workspaces can introduce thread-safety bugs. Prefer
  call-local row buffers plus TBB thread-local scratch unless a stronger
  concurrency contract is established.
- If `tmp` rows are not compressed/sorted, the merge assumption fails. Keep
  `tmp.makeCompressed()` before the custom path.

Rollback:

Rollback Phase 9C or fall back to Phase 9B append/sort behind an env switch if
the k-way merge path changes results beyond tolerance, increases wall time, or
raises memory use.

## Phase 9D: Parallel Direct CSR Fill for Custom Pullback Output

Phase 9C result:

The k-way merge builder reduced the real frame 501 pullback substantially:

- `custom_row_build`: 10.031s -> 3.768s.
- `pullback_hessian`: 18.460s -> 11.134s.
- `multiply_transpose_tmp`: 15.831s -> 8.527s.
- `custom_output_fill`: still 4.704s, avg core 1.01.

At this point, `custom_output_fill` is the largest remaining single-core block
inside the custom pullback. It is not doing mathematical work; it serially
materializes already sorted/reduced row buffers into `Eigen::SparseMatrix` via:

```cpp
simulationHessian.startVec(row);
simulationHessian.insertBackByOuterInner(row, col) = value;
simulationHessian.finalize();
```

Goal:

Add an opt-in direct compressed-row fill path that bypasses Eigen's sequential
insert API and writes the row-major sparse matrix arrays directly:

- `outerIndexPtr()`,
- `innerIndexPtr()`,
- `valuePtr()`.

Enable this path only when both environment variables are set:

```bash
PGO_CONTACT_PULLBACK_CUSTOM=1
PGO_CONTACT_PULLBACK_DIRECT_FILL=1
```

Default behavior remains:

- no custom pullback unless `PGO_CONTACT_PULLBACK_CUSTOM=1`,
- no direct fill unless `PGO_CONTACT_PULLBACK_DIRECT_FILL=1`.

Non-goals:

- Do not change the k-way merge row builder.
- Do not add output pattern/value cache in this phase.
- Do not make custom pullback or direct fill default.
- Do not change `surfaceHessian * W`.
- Do not change solver/contact behavior.

Algorithm:

After Phase 9C, `rowBuffers[row]` is already sorted by column and reduced. Build
the sparse output in compressed row-major form:

1. Compute per-row nnz:

   ```text
   rowNnz[row] = rowBuffers[row].size()
   ```

2. Prefix-sum row nnz to obtain `outer`:

   ```text
   outer[0] = 0
   outer[row + 1] = outer[row] + rowNnz[row]
   totalNnz = outer[n]
   ```

3. Resize and allocate the output matrix:

   ```cpp
   simulationHessian.resize(n, n);
   simulationHessian.resizeNonZeros(totalNnz);
   ```

4. Fill `outerIndexPtr()` serially or with a small copy.
5. Parallel over rows:

   ```text
   offset = outer[row]
   for k in rowBuffers[row]:
     innerIndexPtr[offset + k] = rowBuffers[row][k].col
     valuePtr[offset + k] = rowBuffers[row][k].value
   ```

This is safe to parallelize because each row writes a disjoint range
`[outer[row], outer[row + 1])`.

Implementation plan:

1. Add profiling names.
   - File:
     `src/core/contact/ipc/profiling/surfaceIPCProfiling.h`.
   - Add sections:
     - `contact.adapter.pullback_hessian.custom_direct_fill_prepare`
     - `contact.adapter.pullback_hessian.custom_direct_fill_values`
     - Keep wrapping both direct and Eigen fill under existing
       `contact.adapter.pullback_hessian.custom_output_fill`.
   - Add counters:
     - `contact.adapter.pullback_hessian.custom_direct_fill_enabled`
     - `contact.adapter.pullback_hessian.custom_direct_fill_nnz`
     - `contact.adapter.pullback_hessian.custom_direct_fill_rows`

2. Add env-gated direct fill helper.
   - File:
     `src/core/contact/surfaceDofMap.cpp`.
   - Add helper:

     ```cpp
     bool customDirectFillEnabled();
     ```

   - Add helper:

     ```cpp
     fillSimulationHessianDirect(rowBuffers, numOutputRows, outputNnz, simulationHessian);
     ```

   - Keep the existing Eigen insert fill helper/path as fallback.
   - The direct fill helper must:
     - use `EigenSupport::SpMatD::StorageIndex` for outer/inner arrays,
     - check/cast `outputNnz` and row offsets safely,
     - preserve sorted inner columns,
     - support empty rows,
     - call the correct Eigen methods to leave the matrix compressed and valid.

3. Retain correctness checks through tests.
   - Existing custom pullback test should run direct fill by setting
     `PGO_CONTACT_PULLBACK_DIRECT_FILL=1` and compare against Eigen reference.
   - Add a second test, or extend the existing one, to run both:
     - custom k-way + Eigen insert fill,
     - custom k-way + direct fill,
     and compare both to the same `W.transpose() * surfaceHessian * W`
     reference.
   - Check direct fill counters are recorded only when direct fill is enabled.

4. Verification:
   - Build:
     ```bash
     cmake --build build/base --target surfaceDofMap_gtest -j 8
     ```
   - Tests:
     ```bash
     /root/miniconda3/envs/libpgo-mkl/bin/ctest --test-dir build/base \
       -R 'SurfaceDofMapGTest|IPCContactEnergyGTest' --output-on-failure
     ```
   - Run:
     ```bash
     git diff --check
     ```
   - Real frame A/B, run by main agent:
     - Stop the long tmux job around rebuild.
     - Run frame 501 from `state0500.npz` with:

       ```bash
       PGO_CONTACT_PULLBACK_CUSTOM=1
       ```

       as the Phase 9C baseline.
     - Run the same frame with:

       ```bash
       PGO_CONTACT_PULLBACK_CUSTOM=1
       PGO_CONTACT_PULLBACK_DIRECT_FILL=1
       ```

     - Compare:
       - solver status and iterations,
       - final gradient max norm,
       - accepted energy,
       - `simulation_hessian_nnz`,
       - `custom_output_fill`,
       - `custom_direct_fill_prepare`,
       - `custom_direct_fill_values`,
       - current/peak memory.

Expected result:

`custom_output_fill` should drop from about 4.704s and average core should rise
above 1.0. Because this phase is mostly memory bandwidth and sparse array
materialization, the expected improvement is moderate rather than dramatic.

If direct fill works, the remaining pullback costs should be:

- `multiply_surface_hessian_map`,
- `custom_row_merge`,
- direct sparse array fill.

Risks:

- Eigen sparse matrix internal invariants must be respected. A matrix with
  inconsistent `outerIndexPtr`, `innerIndexPtr`, `valuePtr`, or compressed state
  can fail later in subtle ways.
- `Eigen::Index` and `StorageIndex` conversions must not overflow. The real
  frame has about 34M output nnz, which fits 32-bit signed storage, but this
  should still be checked before casting.
- Empty rows must produce repeated outer offsets.
- If `resizeNonZeros` leaves the matrix in an uncompressed state for this Eigen
  version, the helper must call the appropriate compression/finalization method
  or fall back to Eigen insert fill.
- Directly touching Eigen internals is less future-proof than the public insert
  API; keep it behind `PGO_CONTACT_PULLBACK_DIRECT_FILL`.

Rollback:

Disable `PGO_CONTACT_PULLBACK_DIRECT_FILL` or revert Phase 9D if direct fill
produces invalid sparse matrices, changes solver results beyond tolerance,
increases memory, or does not improve `custom_output_fill` on the real frame.

## Phase 10: Newton Iteration Trace + Termination/Damping Policy Framework

Phase 9D result:

The direct sparse output fill moved contact pullback off the top of the profile:

- `custom_output_fill`: 4.704s -> 1.261s.
- `pullback_hessian`: 11.134s -> 7.593s.
- Newton solve: 200.0s -> 193.5s.

The next useful optimization target is no longer one obvious sparse assembly
block. We need better visibility into Newton iteration value:

- which iterations spend time in factorize/solve/line search,
- which iterations make little gradient or energy progress,
- whether tiny steps are limited by contact feasibility, line search, or
  damping,
- whether repeated symbolic rebuilds correlate with poor progress.

Goal:

Add a structured per-Newton-iteration trace and lay the design groundwork for
pluggable termination and damping policies.

Non-goals for the first implementation:

- Do not change default solver behavior.
- Do not enable adaptive termination by default.
- Do not enable reactive damping by default.
- Do not replace line search.
- Do not change Pardiso or sparse solver behavior.
- Do not change convergence status for existing default runs.

Phase sequence:

1. `10A Trace Only`: add observability and prove default behavior is unchanged.
2. `10B Policy Interfaces`: move existing termination/damping decisions behind
   default policies without changing results.
3. `10C Adaptive Termination Experiment`: add opt-in early-stop policy based on
   low-value iteration traces.
4. `10D Reactive Damping Experiment`: add opt-in damping adjustment driven by
   line-search and progress traces.

Only 10A should be implemented first. 10B-10D depend on 10A trace quality and
must remain disabled until separate validation proves they are safe.

### Phase 10A: Trace Only

Scope:

Add diagnostics only. The Newton loop should make the same decisions as before.
The trace should be emitted through the existing `SolveDiagnostics` object and
therefore appear in Python profile JSONL without introducing a new logging
path.

Per-iteration trace fields:

```text
iteration
energy_before
energy_after
energy_delta

grad_norm_before
grad_norm_after
grad_max_before
grad_max_after
grad_reduction_ratio

grad_dot_dx
raw_step_norm
raw_step_max_norm
accepted_alpha
accepted_step_norm
accepted_step_max_norm

line_search_iterations
line_search_status
min_feasible_alpha
feasible_alpha
line_search_alpha
contact_clamp_count
material_clamp_count

damping_value
hessian_rows
hessian_cols
hessian_nnz
symbolic_rebuilt

func_grad_hessian_seconds
symbolic_analyze_seconds
factorize_seconds
solve_seconds
line_search_seconds
iteration_wall_seconds

low_value
```

Frame summary fields:

```text
newton_low_value_iteration_count
newton_tiny_step_count
newton_small_alpha_count
newton_symbolic_rebuild_count
newton_worst_progress_iteration
newton_worst_progress_ratio
newton_total_factorize_seconds
newton_total_solve_seconds
```

Initial low-value heuristic:

```text
accepted_alpha < 1e-2 && grad_reduction_ratio > 0.9
or line_search_iterations > 8 && abs(energy_delta) is near floating tolerance
or accepted_step_max_norm < step_tol && grad_max_after is still unresolved
or iteration_wall_seconds is high && grad_reduction_ratio > 0.95
```

Implementation plan:

1. Add trace data structures.
   - File:
     `src/core/nonlinearOptimization/solver/common/solveDiagnostics.h`.
   - Add `NewtonIterationTrace`.
   - Add `SolveDiagnostics::newtonIterations`.
   - Add aggregate counters and timers.
   - Add helpers:
     - `recordNewtonIteration`.
     - `completeLastNewtonIterationAfterState`.
   - Reset all new fields through the existing `SolveDiagnostics::reset()`.

2. Instrument the Newton loop.
   - Files:
     - `src/core/nonlinearOptimization/solver/newton/NewtonSolver.h`.
     - `src/core/nonlinearOptimization/solver/newton/NewtonSolver.cpp`.
   - Measure:
     - current state evaluation,
     - symbolic analyze,
     - factorize,
     - solve,
     - line search,
     - whole iteration wall time.
   - Fill trace before the linear solve and update it after the accepted step.
   - On the next iteration, complete the previous trace with
     post-step energy and gradient.
   - Keep all existing stop/failure decisions unchanged.

3. Expose diagnostics to Python.
   - Files:
     - `src/python/pypgo/solver/core.cpp`.
     - `src/python/pypgo/simulation/dynamic/core.cpp`.
     - `pypgo/solver/result.py`.
   - Add `newton_iterations` as a JSON-serializable list of dictionaries.
   - Add summary fields to the diagnostics dictionary/dataclass.
   - Convert non-finite optional values to `None` only at the Python boundary
     if needed; keep native C++ trace numeric for low-overhead recording.

4. Add focused tests.
   - File:
     `tests/src/core/solver/NewtonSolver_gtest.cpp`.
   - Check that a simple quadratic solve records at least one Newton trace.
   - Check important fields:
     - iteration index,
     - energy decrease,
     - gradient reduction,
     - accepted alpha,
     - line-search status,
     - Hessian dimensions/nnz,
     - nonnegative timing fields.
   - Extend diagnostics reset tests so trace vectors and aggregate counters
     reset correctly.

5. Validation.
   - Build:
     ```bash
     cmake --build build/base --target NewtonSolver_gtest -j 8
     ```
   - Tests:
     ```bash
     /root/miniconda3/envs/libpgo-mkl/bin/ctest --test-dir build/base \
       -R 'NewtonSolverGTest|SolveDiagnosticsGTest' --output-on-failure
     ```
   - Full compile smoke:
     ```bash
     cmake --build build/base -j 8
     ```
   - Style:
     ```bash
     git diff --check
     ```
   - Real-frame trace-only smoke:
     - Run the same frame 501 setup used for Phase 9D.
     - Confirm:
       - solve status and iteration count match the Phase 9D baseline,
       - `newton_iterations` exists in `profile_dynamic.jsonl`,
       - trace count matches completed Newton step attempts,
       - factorize/solve totals are plausible,
       - trace overhead is small relative to existing solver wall time.

Sub-agent execution contract:

- Worker scope is limited to:
  - `src/core/nonlinearOptimization/solver/common/solveDiagnostics.h`
  - `src/core/nonlinearOptimization/solver/newton/NewtonSolver.h`
  - `src/core/nonlinearOptimization/solver/newton/NewtonSolver.cpp`
  - `src/python/pypgo/solver/core.cpp`
  - `src/python/pypgo/simulation/dynamic/core.cpp`
  - `pypgo/solver/result.py`
  - `tests/src/core/solver/NewtonSolver_gtest.cpp`
- The worker may adjust an existing partial 10A skeleton, but must not revert
  unrelated Phase 9 changes or user edits.
- The worker must not restart the long tmux run. Main-agent validation owns
  real-frame profiling after review.
- Any behavior change is a failure for 10A. Instrumentation must report what
  happened; it must not change what happens.
- The worker report must include:
  - files changed,
  - tests run and results,
  - behavior-risk notes,
  - whether real-frame profiling remains for the main agent.

Success criteria for 10A:

- Default behavior is unchanged on tests and the real frame smoke.
- Python profile output contains enough per-iteration data to classify
  low-value Newton iterations.
- The trace answers whether poor progress is dominated by:
  - tiny accepted step,
  - small feasible alpha,
  - line-search shrink,
  - symbolic rebuild,
  - factorize/solve cost,
  - weak gradient reduction.

Rollback:

Revert Phase 10A if trace-only changes convergence behavior, introduces
noticeable runtime overhead on frame 501, or produces incomplete diagnostics
that cannot be serialized through Python.

### Phase 10B: Policy Interfaces

Scope:

Introduce interfaces only after 10A proves the trace is stable.

Termination interface:

```cpp
class NewtonTerminationPolicy
{
public:
  virtual NewtonTerminationDecision beforeLinearSolve(const NewtonIterationContext &ctx);
  virtual NewtonTerminationDecision afterStep(const NewtonIterationContext &ctx);
};
```

Damping interface:

```cpp
class NewtonDampingPolicy
{
public:
  virtual double dampingForIteration(const NewtonIterationContext &ctx);
  virtual void updateAfterStep(const NewtonIterationContext &ctx);
};
```

Default implementations:

- `DefaultNewtonTerminationPolicy`: exactly reproduces current termination.
- `NoDampingPolicy`: damping disabled.
- `FixedDampingPolicy`: reproduces current fixed damping behavior when enabled.

10B success criteria:

- Default policy path matches pre-policy behavior.
- Existing tests still pass without enabling experimental policies.
- New policy classes do not make the Newton loop harder to read or debug.

### Phase 10C: Adaptive Termination Experiment

Scope:

Add an opt-in experimental policy only after 10A/10B are stable.

Proposed switch:

```bash
PGO_NEWTON_TERMINATION=adaptive_progress
```

Initial parameters:

```text
step_tol
energy_tol
grad_reduction_tol
patience
```

Initial rules:

- Stop after K consecutive low-value iterations.
- Stop if accepted step is tiny, energy is unchanged within tolerance, and
  gradient progress is poor.
- Never mark success unless the existing convergence criteria are met; early
  adaptive stop should have its own diagnostic status/reason.

### Phase 10D: Reactive Damping Experiment

Scope:

Add an opt-in damping policy after the trace shows whether line-search shrink
is a dominant failure mode.

Proposed switch:

```bash
PGO_NEWTON_DAMPING_POLICY=line_search_reactive
```

Initial rule:

```text
alpha < 1e-2 or line_search_iterations > 8:
  lambda *= 10
alpha == 1 and progress is good:
  lambda *= 0.5
lambda = clamp(lambda, lambda_min, lambda_max)
```

10D success criteria:

- Fewer severe line-search shrinks.
- Fewer tiny accepted steps caused by non-SPD or too-aggressive Newton
  directions.
- Final residual and frame trajectory remain acceptable in A/B tests.

---

## Implemented Results (Phases 10A – 10B)

### Phase 10A Completed: Newton Iteration Trace

Real workload validation on frame 500 (`state0499.npz` → timestep 500):

- Frame 500: CONVERGED, 11 Newton iterations, ~249s Newton solve.
- `newton_iterations`: 11 per-iteration traces in `profile_dynamic.jsonl`.
- All trace fields populated: energy, gradient, step norms, line search, factorize/
  solve timings, symbolic rebuild flags, `low_value` classification.
- All 11 iterations: `low_value=False` (monotonic convergence, α=1.0, smooth gradient
  reduction). No STEP_TOO_SMALL or line-search shrink observed on this frame.
- Summary counters: `newton_low_value_iteration_count=0`, `newton_symbolic_rebuild_count=2`,
  `newton_total_factorize_seconds=107.5s`, `newton_total_solve_seconds=2.4s`.
- Verify: solve status and iteration count match Phase 9D baseline. Trace overhead
  is unmeasurable relative to ~20s/iteration factorize cost.

### Phase 10B Completed: Policy Interfaces

New files:

- `src/core/nonlinearOptimization/solver/newton/newtonTerminationPolicy.h/.cpp`
- `src/core/nonlinearOptimization/solver/newton/newtonDampingPolicy.h/.cpp`

Interfaces:

```
NewtonTerminationPolicy              NewtonDampingPolicy
├─ FixedNewtonTerminationPolicy      ├─ NoDampingPolicy
│   (absolute + relative tol)        │   (always returns 0)
│                                   └─ FixedDampingPolicy
└─ afterStep (default: Continue)        (dampingScale × lambdaScale × lambda0)
```

`SolverParam` gains `termination` and `damping` shared_ptr fields (null defaults to
correct policies). `isConverged()` and `updateDampingScale()` removed from
`NewtonSolver`; convergence is delegated to `beforeLinearSolve()`, damping to
`dampingForIteration()`. `afterStep` hook added after trace recording for future
adaptive policies.

Hooks in the Newton loop:
1. `evaluateCurrentState()` → `terminationPolicy->beforeLinearSolve(ctx)` replaces `isConverged()`
2. `prepareReducedSystem()` → `dampingPolicy->dampingForIteration(ctx)` replaces `addDamping` flag
3. After `recordNewtonIteration(trace)` → `terminationPolicy->afterStep(ctx)` (default: Continue)

Tests: 35→36 all pass including 8 new policy tests.

### Phase 10B Extension: Python API for Policies

New Python modules:

- `pypgo/solver/damping.py`: `NoDamping()`, `FixedDamping(damping_scale)` factory functions.
- `pypgo/solver/termination.py`: `FixedTermination()` factory function.
- `PyDampingPolicy` / `PyNoDamping` / `PyFixedDamping` / `PyTerminationPolicy` /
  `PyFixedTermination` C++ wrapper classes in `core.h` / `bindings.cpp`.
- `NewtonOptimizer.__init__` now takes `damping=` and `termination=` policy objects
  (same pattern as `line_search=`) instead of `damping`/`damping_scale` scalars.
- `NewtonOptimizer.Options::dampingScale` replaced by `damping` and `termination`
  shared_ptr fields; the old scalar field is removed rather than mapped for
  backwards compatibility.
- `_runners.py::_make_optimizer` builds `FixedDamping` / `NoDamping` from config.

### Phase 10 Trace Read: Frame 579

Current long-run profile:

- File:
  `examples/outputs/bunny-dynamic-drop-compare-conservative-r15/cubic_hermite/profile_dynamic.jsonl`
- Frame 579: `CONVERGED`, 18 Newton iterations, 6 low-value iterations, final
  `grad_max=8.17e-05`.
- Total Newton linear work on this frame:
  `newton_total_factorize_seconds=214.4s`, `newton_total_solve_seconds=4.9s`.
- Low-value iterations: 1, 4, 5, 7, 12, 13.
- The run uses `NoDamping()` (`damping_value=0.0` for all iterations).

Important interpretation:

- `low_value` alone is not a safe termination signal.
  - Iterations 4 and 5 are consecutive low-value steps, but iteration 6 reduces
    `grad_max` from `1.45e-01` to `2.46e-02`.
  - Iterations 12 and 13 are consecutive low-value steps, but iteration 14
    recovers strongly, reducing `grad_max` from `9.40e-02` to `2.13e-03`.
- A `patience=2` adaptive termination rule would therefore stop this frame far
  too early. Even `patience=3` is risky on nearby frames; frame 582 has a
  low-value sequence from iterations 7 through 11 before recovering.
- The most plausible termination opportunity in frame 579 is not one of the
  flagged low-value iterations. It is iteration 17: before solving it,
  `grad_max=1.08e-04`, just above the default `1e-04` absolute tolerance, and
  the final solve only brings it to `8.17e-05`. A near-converged adaptive stop
  could save one expensive factorization here, but it would need an explicit
  non-default status/reason because it does not satisfy the fixed criterion.

Frame 579 iteration-level actionability:

| Iteration | Observation | Recommended experimental action |
| --- | --- | --- |
| 1 | `alpha=1`, energy decreases, but `grad_max` grows `5.04e-02 -> 1.16e-01`. | Do not terminate. Consider damping only after repeated early gradient spikes across frames. |
| 4-5 | Consecutive low-value gradient spikes with moderate line search (`alpha=0.5`). | Do not terminate. A reactive damping policy could raise damping for iteration 6, but this needs A/B validation because iteration 6 already recovers. |
| 7 | Mild low-value step (`ratio=1.11`, `alpha=0.25`). | Ignore for termination. Too weak as a damping trigger by itself. |
| 12 | Severe shrink (`alpha=1/64`, 7 line-search iterations), near-zero/slightly positive energy delta, weak progress. | Strong damping trigger for the next iteration. |
| 13 | Severe shrink (`alpha=1/16`, 5 line-search iterations), positive energy delta, `grad_max` spike `8.92e-03 -> 9.40e-02`. | Strong damping trigger; this is the clearest 10D target. |
| 17 | Very small final step; starts just above tolerance and ends below it. | Candidate for opt-in near-converged termination, not low-value termination. |

Suggested Phase 10C/10D direction:

1. Keep Phase 10C conservative.
   - Do not stop on low-value count alone.
   - Add an opt-in `NearConvergedTerminationPolicy` only if the caller accepts
     a diagnostic early-stop status/reason, e.g. stop when
     `grad_max < 1.1 * epsilon` and recent accepted steps are tiny.
   - For the current 76 traced frames, a `1.1 * epsilon` near-converged rule
     would have applied to 16 frames and saved about 20 Newton iterations. Frame
     579 would save only iteration 17.
2. Prioritize Phase 10D reactive damping.
   - The useful trigger should be weaker than the original draft
     (`alpha < 1e-2` or `line_search_iterations > 8`), because frame 579's
     actionable bad steps are `alpha=1/64` and `alpha=1/16` with 7 and 5 line
     search iterations.
   - Candidate trigger:
     `low_value && (accepted_alpha <= 1.0/16.0 || line_search_iterations >= 5 || energy_delta >= 0.0 || grad_reduction_ratio >= 2.0)`.
   - Increase damping for the following iteration; decay only after a full
     `alpha=1` step with strong gradient reduction.
3. Extend the policy interface before implementing 10D.
   - `NewtonDampingPolicy::dampingForIteration()` is currently `const` and has
     no `updateAfterStep()` hook, so it cannot implement stateful reactive
     damping cleanly.
   - Add `updateAfterStep(const NewtonIterationTrace &trace)` or an equivalent
     post-step hook, then expose an opt-in Python factory.

---

## Phase 5B Extension: Parallel Row Collection and Direct CSR Template Fill

After Phase 5A parallel sort/unique, three serial bottlenecks remained in
`buildEnergySetHessianTemplateRowWise()`:

### Parallel `row_collect_fixed` and `row_collect_dynamic`

Both sections replaced the serial `push_back` loop with a count→prefix-sum→fill flat
array→append pattern:

1. **Count pass** (parallel): iterate over input columns, atomically increment per-row counters.
2. **Prefix sum** (serial): O(nRows), negligible.
3. **Fill pass** (parallel): iterate over columns again, write to flat array using
   atomically-incremented per-row write positions.
4. **Append** (serial): copy flat slices into `rowColumns[row]`.

For `row_collect_dynamic`, the outer loop iterates terms serially (only 1–2 terms)
with inner `tbb::parallel_for` over columns (thousands of work items). For
`row_collect_fixed`, `tbb::blocked_range` with grain size 256 is used.

### Direct CSR fill for `row_insert`

Replaced Eigen's sequential `startVec`/`insertBackByOuterInner`/`finalize` with
direct writes to `outerIndexPtr`, `innerIndexPtr`, `valuePtr`:

1. Prefix-sum `rowNonZeros` → `outerIndexPtr`.
2. `resizeNonZeros(totalNonZeros)`.
3. `tbb::parallel_for` rows: write `innerIndexPtr[offset+k]` and `valuePtr[offset+k]`
   directly (each row writes a disjoint range, no contention).
4. Serial fallback (`isCompressed()` guard) in case Eigen's `resizeNonZeros` leaves
   the matrix uncompressed.

### A/B Comparison on Frame 500

Strict A/B on the same frame (`state0499.npz`, 4 cache rebuilds):

| Section | Serial (baseline) | Parallel |
|--------|-------------------|----------|
| `row_collect_fixed` | 0.75s・cores=1.00 | 0.34s・cores=8.20 |
| `row_collect_dynamic` | 1.87s・cores=1.00 | 0.73s・cores=7.57 |
| `row_sort_unique` | 0.13s・cores=27.85 | 0.13s・cores=29.35 |
| `row_insert` | 0.34s・cores=28.76 | 0.36s・cores=29.43 |
| **rebuild_template total** | **3.11s** | **1.58s（↓49%）** |

On contact-heavy frames with 8 rebuilds (e.g., frame 439), estimated improvement
from ~9.3s → ~1.5s.

---

## Phase 11: Legacy Code Path Cleanup

After all optimization phases above were verified in production, the env-var-gated
verification and fallback paths became dead weight. This phase removes them,
keeping only the optimized path as the default (and only) implementation.

### Phase 11A: EnergySet — Remove Verification and Reference Assembly

Scope:

- `src/core/nonlinearOptimization/energySet.cpp`

Removed functions:

- `buildEnergySetHessianTemplateTripletReference()` — the old triplet-based
  reference template builder.
- `assertSameTemplateBuilderResult()` — only used by verify mode.
- `assertSparseMatricesNear()` — only used by verify mode.
- `patternMismatchDetail()` — only used by assertion helpers.
- `addDynamicHessianReference()` — the old reference assembly path for dynamic
  terms (build global triplets -> setFromTriplets -> add_global_hessian).
- `envEnabled()` — no remaining env-var usage.

Removed env-var-gated paths from `func_grad_hessian()`:

- `PGO_ENABLE_ENERGY_SET_HESSIAN_CACHE` — always enabled; the caching path is
  now the only path.
- `PGO_VERIFY_ENERGY_SET_HESSIAN_CACHE` — removed; verification against the
  old path is no longer needed.
- `PGO_VERIFY_ENERGY_SET_TEMPLATE_BUILDER` — removed; the row-wise template
  builder is the only builder.

Simplifications:

- `hess = hessianAll; memset(...)` initialization removed; `assembleCachedHessian()`
  creates the output matrix directly.
- Dynamic-term reference assembly (`addDynamicHessianReference`) removed.
- Fixed-term `addSmallToBig` into `hess` outside the cache removed (cache handles it).
- The `useCacheAssembly`/`verifyCache`/`enableCache` guard variables removed.
- `assembleCachedHessian()` is always called after term evaluation.

Removed profiling section strings:
- `energy_set.fgh.dynamic.build_global_triplets`
- `energy_set.fgh.dynamic.set_from_triplets`
- `energy_set.fgh.dynamic.add_global_hessian`
- `energy_set.fgh.fixed.add_hessian`
- `energy_set.fgh.cache.verify`
- `energy_set.fgh.cache.rebuild_template.verify_triplets`

All `energy_set.fgh.cache.*` sections are preserved (rebuild_template,
rebuild_mappings, add_fixed, add_dynamic, etc.).

Removed unused include: `<sstream>`.

Parallel row collection preserved in `buildEnergySetHessianTemplateRowWise`:

- Fixed-term row collection uses a parallel two-pass pattern: atomic count per row,
  prefix-sum into offsets, parallel fill into a flat array, then append slices into
  `rowColumns`. (Profiling: `energy_set.fgh.cache.rebuild_template.row_collect_fixed`)
- Dynamic-term row collection uses the same parallel pattern over dynamic Hessian
  matrices, mapping local coordinates through DOF lists. (Profiling:
  `energy_set.fgh.cache.rebuild_template.row_collect_dynamic`)
- Row sort and unique runs in parallel over all rows. (Profiling:
  `energy_set.fgh.cache.rebuild_template.row_sort_unique`)
- CSR array fill runs in parallel when the matrix is already compressed after
  `resizeNonZeros`, writing `innerIndexPtr` and `valuePtr` directly with disjoint
  per-row offsets. Otherwise falls back to serial Eigen insert. (Profiling:
  `energy_set.fgh.cache.rebuild_template.row_insert`)

These optimizations were introduced in Phases 4/5 and remain the only path.

### Phase 11B: SurfaceDofMap — Remove Eigen Fallback and Env Gating

Scope:

- `src/core/contact/surfaceDofMap.h`
- `src/core/contact/surfaceDofMap.cpp`
- `src/core/contact/ipc/profiling/surfaceIPCProfiling.h`

Removed functions:

- `customPullbackEnabled()` — env check `PGO_CONTACT_PULLBACK_CUSTOM`.
- `customDirectFillEnabled()` — env check `PGO_CONTACT_PULLBACK_DIRECT_FILL`.
- `fillSimulationHessianWithEigenInsert()` — the Eigen serial-insert fallback.

Simplifications:

- `fillSimulationHessianDirect()`: removed the Eigen insert fallback when
  `resizeNonZeros` does not produce a compressed matrix; always uses direct CSR
  fill.
- `customParallelTransposeMapMultiply()`: removed the
  `if (customDirectFillEnabled())` branch; always calls
  `fillSimulationHessianDirect()`.
- `pullbackHessian()`: removed the `if (customPullbackEnabled())` check and the
  old `simulationHessian = W.transpose() * tmp` Eigen path; always calls
  `parallelTransposeMapMultiply(tmp)` directly.

Naming cleanup:

- Renamed `customParallelTransposeMapMultiply` -> `parallelTransposeMapMultiply`
  (no longer "custom" since it is the only implementation).
- All profiling section constants renamed to drop `Custom` from the name:
  - `kAdapterPullbackHessianCustomRowBuild` -> `kAdapterPullbackHessianRowBuild`
  - `kAdapterPullbackHessianCustomRowMerge` -> `kAdapterPullbackHessianRowMerge`
  - `kAdapterPullbackHessianCustomWorkspacePrepare` -> `kAdapterPullbackHessianWorkspacePrepare`
  - `kAdapterPullbackHessianCustomOutputFill` -> `kAdapterPullbackHessianOutputFill`
  - `kAdapterPullbackHessianCustomDirectFillPrepare` -> `kAdapterPullbackHessianDirectFillPrepare`
  - `kAdapterPullbackHessianCustomDirectFillValues` -> `kAdapterPullbackHessianDirectFillValues`
  - `kAdapterPullbackHessianCustomEnabled` -> `kAdapterPullbackHessianEnabled`
  - `kAdapterPullbackHessianCustomOutputNnz` -> `kAdapterPullbackHessianOutputNnz`
  - `kAdapterPullbackHessianCustomContributionCount` -> `kAdapterPullbackHessianContributionCount`
  - `kAdapterPullbackHessianCustomActiveOutputRows` -> `kAdapterPullbackHessianActiveOutputRows`
  - `kAdapterPullbackHessianCustomMergeStreamCount` -> `kAdapterPullbackHessianMergeStreamCount`
  - `kAdapterPullbackHessianCustomMergeOutputNnz` -> `kAdapterPullbackHessianMergeOutputNnz`
  - `kAdapterPullbackHessianCustomWorkspaceReusedRows` -> `kAdapterPullbackHessianWorkspaceReusedRows`
  - `kAdapterPullbackHessianCustomDirectFillEnabled` -> `kAdapterPullbackHessianDirectFillEnabled`
  - `kAdapterPullbackHessianCustomDirectFillNnz` -> `kAdapterPullbackHessianDirectFillNnz`
  - `kAdapterPullbackHessianCustomDirectFillRows` -> `kAdapterPullbackHessianDirectFillRows`
- Corresponding string values updated (`.custom_` -> `.`).
- Removed `kAdapterPullbackHessianMultiplyTransposeTmp` (no longer used).
- Removed unused include: `<cstdlib>`.

### Phase 11C: NewtonSolver — Policy-Only API (Remove addDamping/dampingScale)

Scope:

- `src/core/nonlinearOptimization/solver/newton/NewtonSolver.h`
- `src/core/nonlinearOptimization/solver/newton/NewtonSolver.cpp`
- `src/core/nonlinearOptimization/solver/newton/NewtonOptimizer.cpp`
- `src/core/nonlinearOptimization/solver/newton/NewtonOptimizer.h`
- `src/python/pypgo/solver/core.h`
- `src/python/pypgo/solver/core.cpp`
- `src/python/pypgo/solver/bindings.cpp`
- `pypgo/solver/optimizer.py`
- `pypgo/solver/damping.py` (new)
- `pypgo/solver/termination.py` (new)

Removed fields from `SolverParam`:

- `int addDamping = 0;`
- `double dampingScale = 1.0;`

Removed backward-compat from `NewtonOptimizer::Options`:

- `std::optional<double> dampingScale` removed.
- Replaced with `std::shared_ptr<const NewtonDampingPolicy> damping` and
  `std::shared_ptr<const NewtonTerminationPolicy> termination`, identical
  in shape to the existing `lineSearch` field.

C++ `NewtonOptimizer::solve()` now passes policies directly:

```cpp
sp.damping = options_.damping;
sp.termination = options_.termination;
```

Python API updated:

- `damping`/`damping_scale` parameters removed from `NewtonOptimizer.__init__()`.
- Damping is now configured via `damping=ps.FixedDamping(2.0)` or
  `damping=ps.NoDamping()` (matches `line_search=` pattern).
- Termination is configured via `termination=ps.FixedTermination()`.
- New modules: `pypgo/solver/damping.py`, `pypgo/solver/termination.py`.

### Test Updates

Refactored tests to match the removed legacy paths:

- `AddDampingConvergesOnQuadratic` -> `DampingPolicyConvergesOnQuadratic`
  (uses explicit `FixedDampingPolicy`).
- `AddDampingFlagStillWorks` -> `DampingPolicyConvergesOnQuadraticBackwardCompat`
  (uses explicit `FixedDampingPolicy`).
- `ExplicitNoDampingOverridesAddDamping` -> `NoDampingPolicyWithQuadratic`
  (simplified, no addDamping flag).
- `ZeroFixedFastPathAvoidsReducedState...` uses explicit
  `FixedDampingPolicy(Params{1.0})` via `solverParam.damping`.
- `CustomPullbackEigenInsertFillMatchesEigenAndReducesDuplicateColumns` removed
  (merged into direct-fill test).
- `CustomPullbackDirectFillMatchesEigen...` -> `PullbackDirectFillMatchesEigen...`
  (no env-var gating, always direct fill).
- `PullsBackSurfaceHessian` updated to check profiling sections for the parallel
  path instead of the old Eigen path.

### Verification

All 70 relevant tests pass:

```text
100% tests passed, 0 tests failed out of 70
```

- Newton solver tests: 49 tests pass.
- SurfaceDofMap tests: 12 tests pass.
- IPCContactEnergy tests: 9 tests pass.

### Risk Assessment

Low risk. All removed code paths were env-var-gated verification or fallback
paths. The production path has been the optimized path since the previous phases.
Removing dead code reduces maintenance burden and eliminates the risk of
accidentally running the slow path.

Removed profiling section names are a non-breaking change: they were only
visible in profile JSONL output, and the frames that triggered them had
disappeared after earlier phases enabled the cache by default.

---

## Phase 12: Self IPC Direct Row Assembly

Problem statement:

After Phases 9–11 cleaned up the contact pullback and EnergySet dynamic
assembly, the profile showed self IPC Hessian assembly still spending
significant time in Eigen's `setFromTriplets()`:

- `contact.surface.active_set_combined.self`: median ~16.84 ms/call.
- `contact.surface.active_set_combined.self.set_from_triplets`: ~14.34 ms/call
  (85% of the self IPC section).

The existing path pre-allocated a fixed-size triplet array
(`144 * totalPairs` entries), had each pair kernel write its local 12×12
Hessian into a fixed stride (including structurally-zero entries for inactive
vertices), then called `hess.setFromTriplets()` which requires Eigen to
globally sort and hash-deduplicate all triplets.

Goal:

Replace the triplet-based assembly with a row-local direct CSR assembly that
bypasses Eigen's global sort/dedup and reduces memory traffic by skipping
structurally-zero contributions.

Non-goals:

- Do not change self energy or gradient computation.
- Do not change the barrier kernel math (`pointTriangle`, `edgeEdge`).
- Do not introduce env-var gating; the direct path replaces the triplet path
  unconditionally.

Algorithm:

The old path:

```
parallel_for each pair:
  kernel → 12×12 local Hessian
  scatterSelfHessian: write 144 triplets at fixed stride (pairIdx * 144)
hess.setFromTriplets(triplets)  // Eigen global sort + dedup
```

The new path:

```
parallel_for each pair (per-thread buffers, no locks):
  kernel → 12×12 local Hessian
  appendSelfHessianRows: for each (i, di) → global row r,
    for each (j, dj) → global col c (skip if idx[j] < 0):
      threadRows[r].push_back({col: c, value: localH(i, j)})

buildSelfHessianFromThreadRows:
  1. Thread row merge (parallel_reduce):
     merge all thread-specific row vectors into one global row buffer
     per row. Count contributions and active rows.
  2. Row sort + reduce (parallel_for):
     per row: sort by column, merge duplicate columns by summing values.
  3. Direct CSR fill:
     prefix-sum row sizes → outerIndexPtr.
     parallel_for rows: write innerIndexPtr and valuePtr directly
     (disjoint per-row ranges, no contention).
```

Key implementation details:

- `RowValue`: `{Eigen::Index col; double value;}` — lightweight struct
  replacing `Eigen::Triplet<double>`.

- `SelfHessianThreadRows`: wraps `std::vector<std::vector<RowValue>> rows`
  (one vector per global DOF row). Created per thread via
  `tbb::enumerable_thread_specific`, so parallel pair kernels append without
  locks or atomics.

- `appendSelfHessianRows`: replaces `scatterSelfHessian`. Skips inactive
  vertices (`idx[j] < 0`) instead of writing zero-valued triplets. The 12×12
  block produces at most 12×12 = 144 entries but typically far fewer.

- `buildSelfHessianFromThreadRows`: orchestrates merge → sort/reduce → fill.
  Merge and sort/reduce are both parallel over rows.

- Three safe conversion helpers (`checkedStorageIndexFromUint64`,
  `checkedStorageIndexFromEigenIndex`, `checkedEigenIndexFromUint64`) guard
  64→32-bit index casts for large models.

- The gradient path (`scatterSelfGrad`) is unchanged; it already uses atomic
  fetch-add and does not go through triplets.

Files changed:

- `src/core/contact/ipc/core/surfaceIPCSelfBarrierAssembler.cpp`
  - Removed: `scatterSelfHessian`.
  - Added: `RowValue`, `SelfHessianThreadRows`, `RowMergeStats`,
    `appendSelfHessianRows`, `fillSparseRowsDirect`,
    `buildSelfHessianFromThreadRows`, three checked-conversion helpers.
  - Modified: `computeSelfHessian` and `computeSelfAll` to use thread-local
    row buffers instead of pre-allocated triplets.

- `src/core/contact/ipc/profiling/surfaceIPCProfiling.h`
  - Added profiling sections:
    - `kActiveSetSelfDirectRowAssembly` — total direct row assembly time.
    - `kActiveSetSelfThreadRowMerge` — merge phase.
    - `kActiveSetSelfRowSortReduce` — sort + reduce phase.
    - `kActiveSetSelfDirectSparseFill` — CSR fill phase.
  - Added profiling counters:
    - `kActiveSetSelfDirectRowContributions` — total entries before reduce.
    - `kActiveSetSelfDirectActiveRows` — number of non-empty rows.

- `tests/src/core/contact/surfaceIPCBarrierAssembler_gtest.cpp`
  - Updated `ComputeSelfAllRecordsBarrierBreakdownProfiling` to verify new
    profiling sections and counters instead of the old triplet-based sections.
  - Added assertions for `direct_row_contributions` (equals old
    `triplet_slots` count) and `direct_active_rows` (> 0).

Removed profiling sections (no longer emitted):

- `contact.surface.active_set_combined.self.triplet_alloc`
- `contact.surface.active_set_combined.self.set_from_triplets`

Observed result:

- Old path: `self` median ~16.84 ms/call, with `set_from_triplets` ~14.34
  ms/call.
- New path: `self` median ~10.97 ms/call (~35% reduction).

Verification:

- Build:
  ```bash
  cmake --build build/base --target surfaceIPCBarrierAssembler_gtest \
    ipcContactEnergy_gtest pypgo_core -j 8
  ```
- Tests:
  - `surfaceIPCBarrierAssembler_gtest`: 11/11 passed.
  - `ipcContactEnergy_gtest`: 13/13 passed.
- Python import smoke: `pypgo import ok`.
- Real workload: running in tmux `dynamic_cubic_hermite` with log at
  `examples/outputs/bunny-dynamic-drop-compare-conservative-r15/cubic_hermite/logs/contact_direct_rows_20260702T045413Z.log`.

Rollback:

Revert commit `17bcb17` if the direct row assembly produces incorrect Hessian
patterns or values, or if the real workload shows regression in solver
convergence or energy correctness.

---

## Phase 13: Multi-Subsystem Profiling and Serial-Bottleneck Cleanup

Commit `3d9f868`. This phase targeted five remaining serial/low-parallelism
bottlenecks across the contact IPC and EnergySet/NewtonSolver subsystems that
became visible after Phases 9–11.

### 13A: Self and External IPC Profiling Drill-Down

Problem:

The contact profile had inclusive sections (`active_set_combined.self`,
`active_set_combined.external`) but no visibility into where time was spent
inside them: triplet allocation vs `setFromTriplets` vs `hessian_add`.

Changes:

- `surfaceIPCSelfBarrierAssembler.cpp`: wrapped `triplets.resize(...)` and
  `hess.setFromTriplets(...)` with scoped profiling sections. Added counter
  for triplet slot count and output Hessian nnz.
- `surfaceIPCExternalBarrierAssembler.cpp`: same treatment for external path,
  plus a new section for `hess += hessExt` (the sparse add of external into
  the accumulated Hessian).
- `surfaceIPCProfiling.h`: added 10 new section/counter names:
  - Self: `triplet_alloc`, `set_from_triplets`, `triplet_slots`, `hessian_nnz`
  - External: `triplet_alloc`, `set_from_triplets`, `hessian_add`,
    `triplet_slots`, `hessian_nnz`
- `surfaceIPCBarrierAssembler_gtest.cpp`: extended profiling assertions to
  cover all new sections and counters.

This data directly motivated Phase 12 (self `set_from_triplets` was 85% of
self time).

### 13B: External IPC Empty-Pair Early Exit

Problem:

`computeExternalAll` was called even when there were zero active external
pairs, wasting a triplet allocation and `setFromTriplets` on an empty matrix.

Changes:

- Added early return in `computeExternalAll` when `nPT == 0 && nTP == 0 &&
  nEE == 0`. Properly initializes `hess` as an `n × n` zero sparse matrix.
  Preserves existing gradient and energy output (already at origin).
- Added `EmptyExternalAllPreservesExistingOutputsAndSizesFreshOutputs` test
  to verify correctness for both pre-existing and fresh output matrices.

### 13C: Parallel First Pullback Multiply (`surfaceHessian * W`)

Problem:

Phase 9B parallelized only the second multiply (`W^T * tmp`). The first
multiply `tmp = surfaceHessian * W` was still Eigen's serial sparse-sparse
multiplication.

Changes:

- Added `parallelSurfaceHessianMapMultiply` in `surfaceDofMap.cpp`. Algorithm:
  1. For each surface row, iterate over `surfaceHessian` nonzeros and the
     corresponding `W` row entries, producing weighted `(col, value)` pairs.
  2. Sort per row by column, reduce duplicates by summing.
  3. Direct CSR fill into `tmp` via the existing `fillSparseRowsDirect`.
  - Profiling: `workspace_prepare`, `row_build`, `output_fill`.
  - Counters: `tmp_contribution_count`, `tmp_active_rows`.
- Added reusable `fillSparseRowsDirect` helper (separate from the
  `fillSimulationHessianDirect` used for the second multiply; the helper
  generalizes to any row-major output matrix).
- `pullbackHessian()` now calls `parallelSurfaceHessianMapMultiply` instead
  of `tmp = surfaceHessian * surfaceFromSimulationDispMap_`.

### 13D: Parallel Map-Vector Multiplies

Problem:

`surfaceDisplacements()` and `pullbackGradient()` used Eigen sparse
matrix-vector multiply (`W * x` and `W^T * y`), both serial.

Changes:

- Added `parallelSurfaceMapVectorMultiply` (`W * x`): `parallel_for` over
  rows of `W`, each row computes `sum(weight * x[simulationCol])`.
- Added `parallelTransposeMapVectorMultiply` (`W^T * y`): `parallel_for` over
  columns of `W` (rows of `W^T`), each row computes
  `sum(weight * y[surfaceRow])`.
- Both use the precomputed `surfaceFromSimulationDispMapRows_` and
  `simulationToSurfaceDispMapRows_` from Phase 9B.
- `surfaceDisplacements()` and `pullbackGradient()` now call the parallel
  helpers instead of Eigen sparse MV.

### 13E: Pattern Matching Optimization with memcmp

Problem:

Both `NewtonSolver::activeSystemPatternMatches` and
`EnergySet::patternEqual` computed a hash via `sparsePatternHash()` (iterating
all outer/inner indices) and then did element-by-element comparison. The hash
was redundant work in the common case (pattern matches), and element-wise
comparison is slower than `memcmp` on raw `StorageIndex` arrays.

Changes in `NewtonSolver`:

- Removed `mixPatternHashValue`, `sparsePatternHash`, and the `hash` field
  from `LinearSolverPatternCache`.
- `activeSystemPatternMatches` now uses `memcmp` on outer and inner index
  arrays directly, skipping the hash entirely.
- Added profiling sections `make_compressed` and `pattern_match` around the
  compression and comparison calls.

Changes in `EnergySet`:

- Replaced `patternEqual(PatternSnapshot, PatternSnapshot)` with
  `patternMatches(SpMatD, PatternSnapshot)` — compares a live compressed
  matrix against a cached snapshot using `memcmp`.
- Added `emptyPatternMatches` for zero-nnz terms, avoiding snapshot creation.
- Added `zeroSparseValues` with a parallel `std::fill` path for large nnz
  (>100k), serial `memset` for small nnz.
- On cache hit, the existing output matrix pattern is checked against the
  full template via `patternMatches`; only copied from `cache.hessianTemplate`
  when the pattern has changed (output-reuse optimization, avoids an
  unnecessary sparse matrix copy on every call).
- Added profiling sections: `check_patterns`, `snapshot_patterns`,
  `prepare_output`, `dynamic.make_compressed`.

### Files Changed

- `src/core/contact/ipc/core/surfaceIPCSelfBarrierAssembler.cpp`
- `src/core/contact/ipc/core/surfaceIPCExternalBarrierAssembler.cpp`
- `src/core/contact/ipc/profiling/surfaceIPCProfiling.h`
- `src/core/contact/surfaceDofMap.cpp` / `.h`
- `src/core/nonlinearOptimization/energy/energySet.cpp`
- `src/core/nonlinearOptimization/solver/newton/NewtonSolver.cpp` / `.h`
- `tests/src/core/contact/surfaceIPCBarrierAssembler_gtest.cpp`

### Verification

- `surfaceIPCBarrierAssembler_gtest`: all tests pass, including new
  `EmptyExternalAllPreservesExistingOutputsAndSizesFreshOutputs`.
- `SurfaceDofMapGTest`: custom pullback and direct fill tests pass.
- `NewtonSolverGTest`: pattern cache and solve tests pass.
- `EnergySetFused`: all tests pass.
- Real workload: the Phase 12 profile run (`contact_direct_rows_*`) includes
  these changes as they were already merged before Phase 12.
