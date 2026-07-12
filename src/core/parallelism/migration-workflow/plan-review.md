# Phase 1 Plan Review

## Pass 1

### Findings

1. **[minor] Baseline conflates textual references and active calls.** The plan reports 160 calls, while a comment-aware lexical check finds 159 active production calls plus one commented-out reference in `multiVertexPullingSoftConstraintsPOrder.cpp`. Phase 1 also says to ignore and delete dead comments, so the original done criteria could not simultaneously require all 160 entries to be classified as active calls.
2. **[minor] The guard identity and machine-readable source were left to the implementer.** The plan allowed either enclosing-function identities or per-file counts without fixing a format. Phase 1 now fixes JSON as the source of truth and per-file active counts as the non-line-number guard.
3. **[minor] CTest integration and guard failure behavior were not concrete.** The revised plan names the checker, direct invocation, CTest integration, and a hermetic self-test covering add/remove and comment/string filtering.

### Validation Budget

`static` is sufficient because Phase 1 changes inventory and test infrastructure only; it does not alter runtime code.

### Reviewer Handoff Check

- The 160/39 textual baseline was reproduced.
- A comment-aware scan reproduced 159 active calls in 39 files and one dead commented reference.
- `parallelFor` remains limited to `int` per-index callbacks and defaults to `NestedKernelPolicy::Suppress`.
- The repository has no existing production-source TBB guard; `tests/CMakeLists.txt` is the correct CTest integration point.

### Implementation Readiness

`ready with minor fixes`

The findings are local and decision-complete; apply them with `revise-plan`, then rerun `review-plan`.

## Pass 2

### Findings

No blocking or minor implementation-readiness findings remain for Phase 1.

The revised plan now distinguishes textual references from active calls, fixes the scan boundary, names the machine-readable and human-readable artifacts, chooses per-file active counts as the stable guard, defines hermetic negative testing, and names the CTest integration point. These decisions are consistent with the repository test layout and do not alter the accepted migration behavior.

### Validation Budget

`static` remains sufficient for Phase 1. Direct checker execution, its self-test, CTest registration, JSON validation, and `git diff --check` are the required evidence.

### Reviewer Handoff Check

- Source spec and non-goals remain unchanged.
- Phase 1 does not migrate runtime calls or add public API.
- Phase 2 and later validation budgets are unaffected.
- The guard deliberately detects per-file count drift; exact source location is informational and can move without breaking the guard.

### Implementation Readiness

`ready`
