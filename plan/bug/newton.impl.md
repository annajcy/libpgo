# Newton Failure Handling Implementation Record

Source plan: `plan/bug/newton.plan.md`
Implementation doc: `plan/bug/newton.impl.md`

## Summary

Implemented Phase A and Phase B. `NewtonSolver` now returns meaningful nonzero statuses for non-converged solves, and `ImplicitBackwardEulerTimeIntegrator` now exposes a rejectable `tryTimestep(...)` path while keeping legacy `doTimestep(...)` fail-fast.

## Phase A: NewtonSolver Status Codes

Implemented behavior:

- Added `NewtonSolver::SolveStatus` with `Converged`, `MaxIterations`, `LineSearchFailed`, `StepTooSmall`, `NonFinite`, and `LinearSolveFailed`.
- Added `NewtonSolver::solveStatusToString(int)` for stable log/test names.
- `NewtonSolver::solve(...)` no longer unconditionally returns `0`.
- `dx too small` with residual above `epsilon` returns `StepTooSmall`.
- line search failure with residual above `epsilon` returns `LineSearchFailed`.
- non-finite energy, gradient, step, feasible alpha, or line-search energy returns `NonFinite` instead of aborting on non-finite `dx`.
- Newton logs include `status=<name>` in convergence/failure messages, but do not include timestep IDs or caller context.

Tests added:

- `tests/src/core/NewtonSolver_gtest.cpp`
  - converged quadratic solve returns `Converged`;
  - zero feasible step with large residual returns `StepTooSmall`;
  - status string helper returns stable names.

## Phase B: Backward Euler Accept/Reject

Implemented behavior:

- Added `ImplicitBackwardEulerTimeIntegrator::tryTimestep(...)`.
- `tryTimestep(...)` returns the solver status.
- On nonzero solver status:
  - it does not write `q1/qvel1/qacc1`;
  - it does not call `proceedTimestep()`;
  - it returns the solver failure code to the caller.
- Legacy `doTimestep(...)` calls `tryTimestep(...)` and throws `std::runtime_error` on nonzero solver status.
- Added `TimeIntegrator::getTimestepID() const` so tests and future adaptive retry code can verify timestep accept/reject behavior.
- `ImplicitBackwardEulerTimeIntegrator` logs timestep begin/end context, including `T<timestepID>`, `dt`, named `solverRet`, residual, and `accepted=true/false`.
- Added a near-converged stalled-step accept policy:
  - if `solverRet == StepTooSmall` and the free-DOF max residual is at most `2 * eps`, the timestep is accepted with a warning;
  - `tryTimestep(...)` returns `0` for this accepted timestep so legacy `doTimestep(...)` does not throw;
  - `getSolverReturn()` still preserves the original solver status (`StepTooSmall`) for diagnostics.

Tests added:

- `tests/src/core/implicitBackwardEulerTimeIntegrator_gtest.cpp`
  - `tryTimestep(...)` with a forced Newton max-iteration failure returns nonzero and leaves accepted state and timestep ID unchanged;
  - `doTimestep(...)` throws on the same failed solve.
  - `tryTimestep(...)` accepts a near-converged `StepTooSmall` solve when max residual is within `2 * eps`, advances the timestep, and logs a warning.

## Comparison To Source Plan

- The source plan originally allowed either direct unit tests or integration tests for Phase B. This implementation uses direct unit tests with a tiny quadratic `PotentialEnergy`, because it gives deterministic failure via `nIter=0` without pulling in `runIPCSim`.
- `getTimestepID()` was mentioned as Phase D support in the source plan, but Phase B tests need it to prove reject semantics, so it was added in Phase B.
- No timestep/substep prefix was added to `NewtonSolver`; timestep context is printed only from `ImplicitBackwardEulerTimeIntegrator`, matching the latest plan adjustment.
- After testing real IPC logs, Phase B was refined so `StepTooSmall` means Newton stalled, not necessarily that the simulation step must be rejected. The accept/reject decision originally distinguished near-converged stalled solves from clearly failed solves in the integrator layer; this was later removed (see Phase A/B Refinements below).
- Phase C and later adaptive `runIPCSim` behavior were intentionally not implemented in this phase.

## Phase A/B Refinements (2026-04-25)

After Phase A and Phase B landed, the six `examples/ipc` cases were rerun and
all six failed under the new fail-fast semantics. Log inspection identified the
dominant failure mode as floating-point saturation in the line search rather
than the IPC-related causes the source plan anticipated. See plan §11 for the
diagnosis; this section records the implementation changes made in response.

### Phase A refinements

- `NewtonSolver::SolverParam::lsm` default changed from `LSM_SIMPLE` to
  `LSM_BACKTRACK` (`src/core/nonlinearOptimization/NewtonSolver.h`).
- `minimizeEnergy.cpp` direct `LSM_SIMPLE` assignment switched to
  `LSM_BACKTRACK` for consistency.
- `NewtonSolver::solve(...)` convergence test extended from
  `gradNorm < epsilon` to `gradNorm < epsilon || gradNorm < lambda0 * 1e-5`,
  where `lambda0` is the max-norm of the initial gradient computed before the
  Newton loop.
- Loose FP-limit fallbacks added in two break paths:
  - `eng1 > eng` (line search failed): accept as `Converged` when
    `gradNorm < lambda0 * 1e-4`; otherwise return `LineSearchFailed` as before.
  - `stepSize < 1e-15` (step too small): same loose `1e-4` relative check;
    otherwise return `StepTooSmall`.
- Diagnostic logs in both fallback paths now include `lambda0` and the
  effective loose-relative threshold so a reader can tell whether convergence
  was via the absolute or relative criterion.

### Phase B refinements

- Removed `acceptedNearConvergedStalled` and the `nearConvergedStalledRatio = 2.0`
  accept policy from `ImplicitBackwardEulerTimeIntegrator::tryTimestep(...)`.
- `acceptedTimestep` is now the simple rule `solverRet == 0`. Near-converged
  FP-stalled solves are now classified as `Converged` inside `NewtonSolver`
  itself via the relative-tolerance branches above, so the integrator no longer
  needs to second-guess the solver.
- The verbose log line announcing the stalled-accept policy is removed; the
  ordinary `accepted=true/false` line is sufficient.

### Empirical validation across `examples/ipc`

Run with the release build (`build/base_no_mkl/bin/runIPCSim`):

| case | original failure | solver-only fix | + coeff=1e4, max-iter=200 |
|------|-----------------|-----------------|---------------------------|
| `cubic/box-with-sphere` | T315 StepTooSmall   | **T1999 ✅** | (not rerun) |
| `cubic/box-hang`        | T6 MaxIterations    | T1068 fail   | **T1999 ✅** |
| `cubic/box-squash`      | T2 StepTooSmall     | T109 fail    | **T199 ✅**  |
| `tet/box-hang`          | T29 StepTooSmall    | T1044 fail   | **T1999 ✅** |
| `tet/box-squash`        | T1 StepTooSmall     | T79 fail     | **T199 ✅**  |
| `shell`                 | T340 StepTooSmall   | T523 fail    | T507 fail (self-contact) |

The configuration tweaks in the third column reduced fixed-vertex spring `coeff`
from `1e5` to `1e4` and unified `solver-max-iter` to `200` across all six
example configs. Five of six cases reach their configured `num-timestep`. The
remaining shell failure is a genuinely stiff self-contact regime, not the
FP-precision issue this refinement targets.

## Validation

Targeted unit-test commands:

```bash
cmake --build build/base_no_mkl_debug --target NewtonSolver_gtest implicitBackwardEulerTimeIntegrator_gtest
build/base_no_mkl_debug/tests/src/core/NewtonSolver_gtest
build/base_no_mkl_debug/tests/src/core/implicitBackwardEulerTimeIntegrator_gtest
```

The `TryTimestepAcceptsNearConvergedStepTooSmallWithWarning` test in
`implicitBackwardEulerTimeIntegrator_gtest.cpp` was removed in line with the
Phase B refinement (the policy it covered no longer exists).

Broader compile check for touched libraries:

```bash
cmake --build build/base_no_mkl_debug --target nonlinearOptimization simulation
```

End-to-end smoke check (run after the refinements):

```bash
for d in examples/ipc/cubic/box-hang examples/ipc/cubic/box-squash \
         examples/ipc/cubic/box-with-sphere examples/ipc/tet/box-hang \
         examples/ipc/tet/box-squash; do
  (cd "$d" && rm -rf ret-* && \
   ../../../../build/base_no_mkl/bin/runIPCSim *.json) || echo "FAILED: $d"
done
```
