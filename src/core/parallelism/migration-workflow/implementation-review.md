# V1 Implementation Review

## Findings

No blocking or minor conformance findings remain.

## Plan coverage

| Requirement | Status | Evidence |
|---|---|---|
| Classify the complete production baseline | done | 159 active calls reconciled with one dead textual reference |
| Migrate every V1-eligible call | done | 75 additions of `parallelFor`; manifest contains zero `migrate-v1` entries |
| Preserve nested-kernel behavior explicitly | done | 75 migrated calls pair with 75 explicit `NestedKernelPolicy::Inherit` options |
| Avoid index narrowing and partitioner/scratch drift | done | all typed indices, explicit partitioners, and coupled local state remain deferred |
| Guard all remaining production calls | done | 84 source calls match 84 unique manifest entries and per-file counts |
| Keep include/link visibility explicit | done | direct TBB and `parallelism` target audit reports no missing dependency |
| Validate native, Python, C API, contact, and Accelerate behavior | done | all planned suites and smoke checks passed |

## Independent audit notes

- The final manifest distribution is 49 `partitioner`, 23 `typed-index`, and 12 `tls-coupled`, totaling 84.
- The migration diff adds 75 `pgo::parallel::parallelFor` calls and 75 explicit `Inherit` policies.
- `tetMeshOccupation.cpp` correctly moved from the candidate set to deferred: both loops share mutable RNG state across workers and cannot be represented safely by the current per-index API.
- Remaining TBB usage is intentional and has direct target ownership. No target is relying only on an unrelated PUBLIC dependency for TBB or `parallelism` visibility.
- Translation units that retained `parallel_for.h` still have a deferred call; three reduce-only units removed the header.

## Validation

- Fresh Release configure and full build passed.
- Native CTest: 514/514 passed, with one runtime-conditioned skip.
- Python: 401 passed, 22 skipped.
- Inventory guard and its negative/filtering self-test passed.
- macOS Accelerate nested-policy unit test and benchmark smoke passed.
- Formatting and whitespace checks passed.

## Residual risks

- V1 intentionally leaves 84 calls on TBB. Completing the repository-wide migration requires V2 API decisions; mechanically converting these calls would violate the accepted non-goals.
- The inventory guard uses stable per-file counts, so a same-file remove/add swap can preserve the count. The manifest remains the review source of truth.
- No pre-migration benchmark dataset exists for the 75 call sites, so this review establishes correctness and runtime health, not a performance claim.

## Verdict

`matches plan`
