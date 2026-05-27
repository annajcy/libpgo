# Broad Phase Prepared Pairs Baseline

## Case

- Command: `cmake --build build/base_no_mkl --target cipcProfiling_gtest -j2`; `./build/base_no_mkl/tests/src/core/contact/cipcProfiling_gtest`
- Input/config: in-test IPC fixtures from `tests/src/core/contact/cipcProfiling_gtest.cpp`
- Date: 2026-04-24 12:58:44 +08
- Git status summary: `HEAD 3ed6329`; no tracked changes reported before implementation in the checked files.

## Baseline Observation

- `contact.surface.pair_build.static` count: present in existing profiling assertions for `func`; same-state `func/gradient/hessianDirect` aggregate count is not emitted by the current baseline test binary.
- `contact.surface.pair_build.static` total time: not emitted by the current baseline test binary.
- `contact.surface.energy` time: present in existing profiling assertions for `func`; total time is not emitted.
- `contact.surface.gradient` time: not covered by the current baseline profiling test output.
- `contact.surface.hessian` time: not covered by the current baseline profiling test output.
- `contact.adapter.func` time: not covered by `cipcProfiling_gtest`; covered by embedded adapter profiling tests separately, but not emitted as a timing report.
- `contact.adapter.gradient` time: not covered by `cipcProfiling_gtest`; covered by embedded adapter profiling tests separately, but not emitted as a timing report.
- `contact.adapter.hessian_direct` time: not covered by `cipcProfiling_gtest`; covered by embedded adapter profiling tests separately, but not emitted as a timing report.

## Notes

- Same-state `func/gradient/hessianDirect` repeated pair build observed: yes, by code path inspection before implementation. `func`, `gradient`, and `hessianDirect` each call the legacy `SurfaceIPCCore::compute*` API, and each `compute*` calls `findCollisionPairs()`.
- Pair counts if visible: not printed by baseline profiling; `hessianDirect()` logs PT/EE pair counts through the normal logger.
- Caveats: Task 0 intentionally did not add a benchmark target or expand the profiling framework. The implementation tests will add explicit profiling `callCount` assertions for the optimized path.
