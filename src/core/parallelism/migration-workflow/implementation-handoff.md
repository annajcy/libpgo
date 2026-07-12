# V1 Implementation Handoff

## Scope

- Plan: `src/core/parallelism/PARALLEL_FOR_MIGRATION_PLAN_V1.md`
- Implemented phases: Phase 1 through Phase 5
- Validation budget: planned and actual `full-suite`
- Behavior policy: every migrated call explicitly uses `NestedKernelPolicy::Inherit`

## Result

- Original baseline: 159 active production calls plus one dead commented reference.
- Migrated: 75 active calls across 10 modules.
- Deferred: 84 active calls in 25 files, with zero remaining `migrate-v1` entries.
- Deferred categories: 49 `partitioner`, 23 `typed-index`, and 12 `tls-coupled`.
- Removed the dead commented reference.

| Module | Migrated | Deferred |
|---|---:|---:|
| `c` | 2 | 0 |
| `constraintPotentialEnergies` | 4 | 15 |
| `contact` | 43 | 27 |
| `eigenSupport` | 2 | 8 |
| `geometryPotentialEnergies` | 7 | 6 |
| `interpolationCoordinates` | 1 | 4 |
| `mesh` | 8 | 4 |
| `nonlinearOptimization` | 1 | 8 |
| `solidDeformationModel` | 3 | 12 |
| `volumetricMesh` | 4 | 0 |

## Main changes

- Replaced eligible direct-index and simple `blocked_range<int>` loops with `pgo::parallel::parallelFor` without changing index types, algorithms, reduction order, or scratch ownership.
- Added a machine-readable inventory, a human-readable deferred report, and a comment/literal-aware CTest guard with a hermetic self-test.
- Added C API analytical parity tests built with runtime concurrency 1 and 4.
- Added explicit `parallelism` dependencies to migrated targets.
- Made direct TBB dependencies explicit for every target that still owns a TBB primitive; `contact` exposes TBB types in a public header and therefore propagates TBB publicly, while implementation-only users link it privately.
- Removed `parallel_for.h` from translation units that no longer call it.

## Plan drift and safety decisions

- The initial inventory classified 77 calls as V1 candidates. Two loops in `tetMeshOccupation.cpp` were reclassified as `tls-coupled` after implementation inspection found a shared `std::mt19937` used concurrently. Migrating them mechanically would preserve an existing race rather than establish safe ownership, so they remain deferred with an explicit reason.
- No index narrowing, explicit partitioner removal, chunk-scratch granularity change, public ABI change, or nested-kernel policy change was introduced.
- No benchmark-before dataset exists for the migrated call sites. The Accelerate benchmark was used as a runtime smoke test, not as a claimed performance delta.

## Validation

- `conda run -n libpgo cmake --preset base`: passed.
- `conda run -n libpgo cmake --build --preset base -j 6`: passed.
- `conda run -n libpgo ctest --test-dir build/base --output-on-failure`: 514/514 passed; one test self-skipped by its runtime condition.
- `conda run -n libpgo python -m pytest tests/pypgo`: 401 passed, 22 skipped.
- Contact suite: 109/109 passed; 43 race-sensitive tests also passed 20 repetitions each during Phase 4.
- C API concurrency 1 and 4 analytical tests: passed.
- `parallelism_accelerate_nested_benchmark --benchmark_min_time=0.01s --benchmark_repetitions=1`: passed as a macOS Accelerate runtime smoke test.
- Inventory guard, guard self-test, dependency visibility audits, 75-call `Inherit` pairing audit, JSON validation, and `git diff --check`: passed.

## V2 input

- `partitioner` (49): the API needs an explicit scheduling/partitioner decision before these can move.
- `typed-index` (23): the API needs non-narrowing support for `Eigen::Index`, `size_t`, and repository index aliases.
- `tls-coupled` (12): migration needs worker-local state and merge-lifecycle design; the two shared-RNG loops additionally need explicit RNG ownership.
