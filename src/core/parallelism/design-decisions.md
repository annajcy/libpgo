# Parallelism Design Decisions

## Context

`NestedKernelPolicy::Suppress` sets known BLAS kernels to a thread-local single-thread mode,
while `NestedKernelPolicy::Inherit` preserves the caller's BLAS setting. On the validated Linux
MKL-TBB host, `Inherit` reports an MKL requested maximum of 96 even though pgo's process-wide TBB
ceiling is 16 and the active executor arenas use concurrency 1, 4, or 8. The TBB limits constrain
actual participation, but the mismatched MKL request may still create excessive task decomposition
and scheduling overhead.

## Decision Index

| ID | Topic | Status |
|---|---|---|
| DD-001 | Preserve `Inherit`; prototype an executor-bounded mode before exposing it | proposed |
| DD-002 | Select the default nested-kernel policy by configured BLAS backend | rejected by current evidence |
| DD-003 | Freeze the V1 migration and nested-kernel policy scope | accepted |

## Decisions

### DD-001: Preserve `Inherit`; prototype an executor-bounded mode before exposing it

- Status: proposed
- Date: 2026-07-12
- Owner: parallelism maintainers
- Priority: low; do not include in V2 unless profiling supplies a qualifying workload
- Context: The current MKL-TBB benchmark shows that actual worker participation remains bounded by
  the current TBB arena/global control, while `mkl_get_max_threads()` under `Inherit` still reports
  the inherited host-wide value. `Suppress` is faster for the tested nested MKL DGEMM matrix, but a
  middle mode may be useful when an executor has unused capacity and only one or a few outer tasks.
- Decision: Do not silently redefine `Inherit`, and do not add a public `Bounded` enum value yet.
  Do not prioritize the prototype ahead of the typed-index and static-scheduling V2 work. Only
  prototype a benchmark-only MKL mode when a production profile shows that nested dense BLAS is a
  material bottleneck while the outer executor has unused capacity. The prototype would scope
  `mkl_set_num_threads_local(min(inheritedMax, currentExecutorConcurrency))` around the kernel.
  Keep `Suppress` as the default during the experiment.
- Rationale: `Inherit` is a compatibility promise and must continue to preserve the caller's BLAS
  setting. Clamping only to the process ceiling does not account for a smaller current arena or
  concurrent outer tasks. In addition, Accelerate exposes only single-threaded versus
  multi-threaded control, so a numeric cross-platform `Bounded` contract cannot currently be
  implemented consistently.
- Alternatives Considered:
  - Redefine `Inherit` to clamp to the runtime ceiling: rejected because it breaks the documented
    compatibility semantics and still ignores local arena capacity.
  - Add `Bounded` immediately as a public portable policy: deferred because Accelerate cannot honor
    a numeric bound and the MKL performance benefit is not yet measured.
  - Derive a dynamic per-call budget from active outer participants: deferred because participant
    counts are observational, timing-dependent, and do not provide a stable admission-control
    contract.
  - Keep only `Suppress` and `Inherit`: remains the fallback if the bounded prototype does not show
    a meaningful, stable benefit.
- Consequences: The public API remains unchanged. MKL experiments must distinguish requested MKL
  threads, executor concurrency, effective TBB ceiling, active participants, and process thread
  peak. A successful prototype would likely need an explicitly backend-aware contract rather than
  pretending all BLAS backends support the same numeric bound. V2 should spend its engineering
  budget on index-width and static-scheduling coverage unless the trigger above is observed.
- Constraints: No change may affect BLAS calls outside a pgo parallel region. Any scoped MKL setting
  must be restored on normal return and exceptions. The experiment must include warm-up and a valid
  numerical checksum.
- Spec Impact: A public policy requires a new spec only after the experiment establishes semantics
  and cross-platform behavior.
- Plan Impact: None until explicitly promoted.
- Promotion: Not promoted.
- Supersedes: None.
- Follow-ups:
  - Independently fix the DGEMM checksum counter and make process-local warm-up explicit in the
    runner; these benchmark correctness fixes do not depend on adding a third policy.
  - If the profiling trigger occurs, benchmark arena concurrency 1, 4, 8, and 16 with outer task
    counts 1, concurrency, and four times concurrency for matrix sizes 512, 1024, and 2048.
  - If triggered, compare `Suppress(1)`, raw `Inherit`, and the benchmark-only executor-bounded MKL
    mode before proposing an API.
  - Rename telemetry from `worker_mkl_max_threads` to
    `worker_mkl_requested_max_threads`.

### DD-002: Select the default nested-kernel policy by configured BLAS backend

- Status: rejected by current evidence
- Date: 2026-07-12
- Owner: parallelism maintainers
- Context: MKL-TBB and Accelerate have materially different nesting models. The corrected real-FEM
  sweep covers production CubicLinear and CubicTricubicHermite energy, gradient, Hessian, and
  combined calls. On macOS, Inherit's aggregate ratio is 0.965, which does not clear the fixed 5%
  gain threshold; the expensive Hermite Hessian and Full paths are equivalent. On MKL-TBB,
  CubicTricubicHermite Full has Inherit/Suppress medians of 1.077 for 8 elements and 1.117 for 16
  elements, with the latter crossing the fixed 10% important-regression threshold.
- Decision: Do not introduce an Accelerate=Inherit, MKL=Suppress default split based on the current
  evidence. Keep the existing portable `Suppress` default. Explicit call-site policies remain
  authoritative and platform independent. Reopen a backend-specific default only if new production
  evidence clears an explicitly accepted threshold.
- Rationale: MKL has a measured important regression under Inherit, so it must remain Suppress by
  default. macOS shows a favorable direction in some short CubicLinear cases but not the required
  aggregate gain, while its decision-critical Hermite paths are equivalent. Splitting public
  defaults without clearing the accepted macOS bar would add backend-dependent behavior without a
  demonstrated practical benefit.
- Alternatives Considered:
  - Keep `Suppress` as one portable default: remains the baseline and safest fallback.
  - Make `Inherit` the portable default: rejected by the warmed MKL-TBB evidence.
  - Add an `Auto` enum value: deferred because the current API cannot observe inner kernel size or
    predict concurrent outer work.
  - Require every call site to specify a policy: clearest semantics but too noisy for the convenience
    overloads and existing default callers.
- Consequences: Source using `Options{}` keeps one portable behavior across configured backends.
  The 75 V1 migration sites remain stable because they explicitly request `Inherit`. No generated
  public backend configuration is needed for the default at this stage.
- Constraints: Correctness and exception behavior must not vary by backend. Documentation must
  distinguish the configured default from explicit `Suppress`/`Inherit`. A backend without MKL or
  Accelerate needs a defined fallback, proposed as `Suppress`.
- Spec Impact: None unless the backend-specific proposal is reopened.
- Plan Impact: Not part of implementation until explicitly promoted.
- Promotion: Not promoted.
- Supersedes: None.
- Follow-ups:
  - Retain the real cubic FEM benchmark as a regression/decision harness.
  - Decide whether backend selection is represented by a generated public config header or by a
    non-aggregate runtime/default-options factory only if this proposal is reopened.

### DD-003: Freeze the V1 migration and nested-kernel policy scope

- Status: accepted
- Date: 2026-07-12
- Owner: parallelism maintainers
- Context: V1 migrated the repository's inventoried TBB parallel loops to `pgo::parallel`, retained
  compatibility at the migrated call sites with explicit `Inherit`, and added cross-platform policy
  correctness and performance evidence. The real CubicLinear and CubicTricubicHermite sweeps reject
  a portable Inherit default and do not clear the threshold for an Accelerate-specific default.
- Decision: Freeze V1 with one portable `Suppress` default. Keep the migrated compatibility-sensitive
  call sites explicitly `Inherit`. Preserve `Inherit` semantics without clamping it. Do not add
  `Bounded`, `Auto`, executor-derived inner budgets, or backend-specific default selection in V1.
  Retain the inventory checks, policy restoration tests, warmed/checksummed decision runner, and real
  cubic FEM benchmark as V1 verification artifacts. Reopen V1 only for a correctness or release-blocking
  defect; schedule new scheduling modes and API expansion as separately reviewed V2 work.
- Rationale: This is the smallest stable contract supported by both compatibility requirements and
  measured evidence. MKL Hermite Full crosses the important-regression threshold under Inherit,
  while macOS does not show the required aggregate gain for changing its default. Adding another
  mode or backend split now would expand public semantics without a demonstrated V1 benefit.
- Alternatives Considered:
  - Delay V1 until a bounded MKL mode exists: rejected because no production-profile trigger currently
    justifies that scope.
  - Use Inherit as the portable default: rejected by MKL DGEMM and real Hermite Full evidence.
  - Use Accelerate=Inherit and MKL=Suppress defaults: rejected by DD-002 under the fixed thresholds.
  - Remove explicit Inherit from migrated call sites: rejected because it would turn a mechanical TBB
    migration into an unmeasured behavior change.
- Consequences: V1 has a stable, backend-neutral default and preserves behavior at migrated sites.
  V2 can evaluate typed indices, static scheduling, or a profile-triggered bounded prototype without
  holding the V1 migration open. This scope freeze does not itself claim that the working tree has
  been committed, pushed, or released.
- Constraints: Explicit call-site policies remain authoritative. Suppression must stay scoped to pgo
  parallel regions and restore backend state on normal return and exceptions. Benchmark evidence must
  continue to use warm-up and numerical checksum validation.
- Spec Impact: This entry is the accepted V1 release baseline; no additional V1 policy spec is required.
- Plan Impact: Close V1 policy design. Any V2 work requires its own promoted scope and reviewed plan.
- Promotion: Promoted as the V1 release baseline; this is not a request to write a new spec.
- Supersedes: None. It incorporates the V1 outcome of DD-001 and the rejection recorded in DD-002.
- Follow-ups:
  - Commit and publish the accepted V1 implementation and verification artifacts through the normal
    release workflow.
  - Open a separate V2 decision only when its scope is explicitly selected.

## Open Questions

- Should a future public mode be named `ExecutorBound`, `RuntimeBounded`, or remain an internal MKL
  optimization hint?
- What minimum gain and maximum regression should justify a third policy? The current proposal is
  at least 5% gain in its target regime and no important regression above 10%.
- Should unsupported numeric bounding on Accelerate map to `Suppress`, map to `Inherit`, or make the
  policy unavailable on that backend?
- Can a stable remaining-capacity budget be defined without adding admission control to
  `ParallelRuntime`?
- How should the configured backend default be exposed to public-header consumers without relying
  on an OS macro?

## Promoted for Spec

- DD-003: accepted as the V1 release baseline. No new spec was requested because the implementation
  and verification artifacts already exist.

## Change Log

- 2026-07-12: Added DD-001 from the nested MKL policy discussion and cross-platform benchmark
  evidence.
- 2026-07-12: Set DD-001 to low priority and added a production-profile trigger; bounded mode is not
  part of the recommended V2 scope by default.
- 2026-07-12: Added proposed DD-002 for an Accelerate=`Inherit`, MKL=`Suppress` configured default;
  acceptance is gated on corrected warm benchmark evidence and implicit-default call-site tests.
- 2026-07-12: Rejected DD-002 under the fixed thresholds after real CubicLinear and
  CubicTricubicHermite sweeps on macOS and Linux MKL-TBB. MKL Hermite Full crossed the important
  regression threshold; macOS did not clear the minimum-gain threshold.
- 2026-07-12: Accepted DD-003 and froze the V1 migration/policy scope with a portable Suppress
  default, explicit Inherit at compatibility-sensitive migrated sites, and no bounded or
  backend-specific mode in V1.
