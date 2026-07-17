# Python Newton phase-threading benchmark

This benchmark tests whether semantic phase-aware threading improves a real
Newton solve driven entirely through the public Python API. It does not call a
benchmark-only C++ kernel or expose Newton's internal state machine to Python.

Every timed solve uses:

- a real Vega volumetric mesh loaded by `pypgo.mesh.volume.read_veg`;
- a real `pypgo.fem.deformation_energy` using `StableNeo` by default;
- a deterministic nonzero perturbation of the deformation DOFs;
- a fixed top boundary that removes rigid-body null modes;
- `NewtonOptimizer`, `Backtrack`, `NoDamping`, and `MKLPardiso`;
- `gradient_tolerance=0.0` and `FixedTermination`.

There are two explicit modes:

| mode | default policies | repetitions | timed solves per worker | maximum Newton iterations |
| --- | --- | ---: | ---: | ---: |
| `cold_start` | `uniform_single`, `phase_aware`, `phase_single_single` | 7 | 1 | 1 |
| `steady_state` | the full phase-policy 2x2 matrix | 15 | 3 | 8 |

`cold_start` preserves the original exactly-one-iteration experiment.
`steady_state` is the primary end-to-end experiment: it requires at least two
completed Newton iterations, but does not change the public solver semantics to
force exactly eight. The existing convergence and failure exits remain active.
Every policy in one `(repetition, workload)` block must nevertheless follow the
same iteration count and complete line-search/symbolic-system path.

The public `optimizer.solve()` path includes fixed-topology initialization,
initial Hessian and symbolic analysis, every iteration's
energy/gradient/Hessian, PARDISO factorization and solve, line search, final
objective evaluation, and explicit sparse-backend cleanup.

## Workloads

The three defaults use separate mesh sizes so both FEM evaluation and sparse
linear algebra have measurable cost without making the Hermite Hessian
impractically large.

| workload | default mesh | vertices | elements | DOFs |
| --- | --- | ---: | ---: | ---: |
| `tet_linear` | `tet/dragon_big.veg` | 39979 | 186736 | 119937 |
| `cubic_linear` | `cubic/dragon-center-r16.veg` | 4080 | 2456 | 12240 |
| `cubic_tricubic_hermite` | `cubic/bunny.veg` | 711 | 436 | 17064 |

The builder computes connected components from the volume elements and fixes
every formulation DOF belonging to the top 1% slab of each component. If a
component's slab contains fewer than three non-collinear vertices, it expands
downward by the minimum number of height-sorted vertices needed to remove that
component's rigid modes. Fixed values are zero (the rest displacement), and
deterministic Gaussian noise is added only to free DOFs. The result records the
component sizes, expansion decisions, selected DOF count, and a SHA-256 digest
of the fixed indices. Component-aware anchoring is retained for custom meshes,
even though the three default workloads are intended to be connected bodies.

## Policies

With the default concurrency `C=8`, cold-start mode compares:

| policy | evaluation MKL budget | linear-solver MKL budget | execution form |
| --- | ---: | ---: | --- |
| `uniform_single` | 1 | 1 | one outer executor around the whole solve |
| `phase_aware` | 1 | C | `NewtonThreadingPolicy` |
| `phase_single_single` | 1 | 1 | two independent phase executors |

All executors use the same arena concurrency and reserved-slot setting.
`phase_single_single` versus `uniform_single` estimates the cost of the phase
dispatch abstraction while holding the effective thread budgets fixed.

Steady-state mode instead uses the complete phase-policy 2x2 matrix:

| policy | evaluation MKL budget | linear-solver MKL budget | execution form |
| --- | ---: | ---: | --- |
| `phase_single_single` | 1 | 1 | two independent phase executors |
| `phase_aware` | 1 | C | two independent phase executors |
| `phase_reversed` | C | 1 | two independent phase executors |
| `phase_multi_multi` | C | C | two independent phase executors |

`phase_single_single` is the all-single baseline, `phase_aware` is the proposed
policy, `phase_reversed` is the mechanism-negative control, and
`phase_multi_multi` exposes both phases to multi-threaded MKL. `uniform_single`
and `uniform_multi` remain opt-in controls for measuring phase-dispatch versus
one outer executor. The per-element cache fix is expected to make the
evaluation=C controls numerically valid; the benchmark still rejects the whole
block if their result or solver path differs. It never weakens correctness
tolerances to obtain a timing comparison.

## Run

Use a fresh Release build with MKL PARDISO available, then run on an otherwise
idle machine:

```bash
python benchmarks/python_solver_phase_threading/run_python_solver_phase_threading_benchmark.py \
  --mode steady_state \
  --out benchmarks/results/python-solver-phase-threading \
  --concurrency 8
```

The controller runs every case serially. Each timed sample gets a fresh worker
process; the worker environment is set before importing NumPy or `pypgo`:

```text
MKL_NUM_THREADS=C
OMP_NUM_THREADS=C
MKL_DYNAMIC=FALSE
OMP_DYNAMIC=FALSE
```

The worker creates `GlobalTbbControl(C)` before constructing its mesh,
material, energy, or optimizer. In steady-state mode it performs one full
warmup solve and three timed solves; the worker's sample is their median wall
time. Each timed solve restarts from the same deterministic perturbed DOF
vector and is independently validated. Policy order is randomized within each
`(repetition, workload)` block, and blocks are also randomized.

Quick scheduling check without loading `pypgo`:

```bash
python benchmarks/python_solver_phase_threading/run_python_solver_phase_threading_benchmark.py \
  --mode steady_state --dry-run \
  --workloads cubic_tricubic_hermite --repetitions 1
```

Six-policy diagnostic smoke:

```bash
python benchmarks/python_solver_phase_threading/run_python_solver_phase_threading_benchmark.py \
  --mode steady_state \
  --out /tmp/python-solver-phase-six-policy-smoke \
  --workloads cubic_tricubic_hermite \
  --policies uniform_single uniform_multi phase_single_single phase_aware \
    phase_reversed phase_multi_multi \
  --repetitions 1 --timed-solves 1 --bootstrap-samples 100
```

A small end-to-end smoke run after rebuilding:

```bash
python benchmarks/python_solver_phase_threading/run_python_solver_phase_threading_benchmark.py \
  --mode steady_state \
  --out /tmp/python-solver-phase-smoke \
  --workloads cubic_tricubic_hermite \
  --policies uniform_single phase_aware \
  --repetitions 1 --timed-solves 1 --bootstrap-samples 100
```

The fixed three-solve inner sample reduces one-off process noise without an
adaptive minimum-time loop in which a faster policy might execute more
warm-cache solves than a slower policy. The 15 independently randomized worker
samples, rather than the three solves inside one process, remain the units used
for paired uncertainty estimates.

## Outputs and interpretation

The output directory contains:

- `python-solver-phase-threading.json`: manifest, run completion/validity
  status, every raw measurement, correctness signatures, phase diagnostics,
  and summaries;
- `runs.csv`: one row per fresh worker sample;
- `phase_breakdown.csv`: one row per timed solve with lifecycle and aggregate
  Newton timing;
- `iteration_breakdown.csv`: one row per completed Newton iteration with
  evaluation, reduced-system preparation, symbolic analyze, factorize, solve,
  step expansion, line-search, and iteration-wall timing;
- `summary.csv`: median, MAD, range, paired ratio to `phase_aware`, and a paired
  bootstrap 95% interval for total solve time and per-completed-iteration time.

For `paired_ratio_over_phase_aware_*`, a value greater than 1 means
`phase_aware` was faster. Comparisons are paired by workload and repetition.
The controller validates before accepting a completed block that all policies
have the same solver status, iteration count, per-iteration line-search status
and iteration count, accepted-alpha path, symbolic-rebuild path, Hessian shape,
phase call counts, final objective, and result vector. Every DOF is compared
with configurable absolute and relative tolerances. The JSON also records an
exact SHA-256 of canonical little-endian float64 result bytes; this hash is
provenance evidence and is not used in place of the tolerant DOF-wise
comparison. When `--timed-solves` is greater than one, every solve is first
checked against its siblings inside the worker, before cross-policy validation.
The scalar vector checksums remain useful provenance diagnostics but are not
independent correctness gates: summing tens of thousands of tolerated DOF-wise
roundoff differences can make a checksum fail a tolerance that every actual
DOF passes. The authoritative vector check is the element-wise comparison.
Full vectors are retained until their policy block passes that comparison, then
removed from subsequent checkpoints to keep controller serialization bounded;
their SHA-256 and aggregate diagnostics remain in the final artifact. A failed
block retains its full vectors for diagnosis.

Check `run_status` before analyzing a checkpoint. A usable final result has
`state="complete"`, `complete=true`, `valid_so_far=true`, all scheduled blocks
validated, and every row marked `validation_status="valid"`. A worker or
signature failure rewrites the checkpoint with `state="failed"`, an error
message, and invalid/incomplete status rather than leaving an apparently valid
summary. `--case-limit` is recorded together with requested and actual schedule
counts, so a deliberately truncated smoke run is distinguishable from a full
run.

The primary metric is the median public `optimizer.solve()` wall time. The
per-completed-iteration metrics are secondary normalization checks; they do not
replace total time. Structured timings separate optimizer preparation,
NewtonSolver setup, initialization, Newton solve, final objective, and explicit
linear-solver cleanup. Within Newton, nested phase timings explain where a
policy helps. Because nested timers overlap, do not add every timing column
together. `optimizer_accounting_residual_seconds` only closes the non-overlap
top-level lifecycle equation.

A primary result supports the mechanism when:

1. numerical and full per-iteration solver-path signatures match across the
   four phase policies;
2. `phase_aware` beats `phase_single_single` by more than run-to-run variation;
3. PARDISO factorization and solve improve when the linear phase changes from
   budget 1 to budget C;
4. `phase_reversed` and `phase_multi_multi` show that giving evaluation budget
   C does not account for the proposed policy's gain.

If either evaluation=C control still fails its signature after the per-element
cache fix, treat that as an implementation defect or an invalid experimental
block, not as performance evidence.

The benchmark establishes this claim only for the recorded machine, build,
meshes, concurrency, and runtime environment. The JSON records the Git revision
and dirty-worktree state, hashes of the benchmark runner, workload builder,
input VEG mesh, and loaded `pypgo` core binary, plus binary paths and effective
thread variables. Match these fields before combining runs.

The runner records inherited `MKL_THREADING_LAYER`, `KMP_AFFINITY`,
`OMP_PROC_BIND`, `OMP_PLACES`, and the worker's Linux CPU-affinity mask when
available, but it does not impose a placement policy. Pin the benchmark using
the server's normal affinity tooling when strict placement control is needed.
Policy order is randomized within adjacent repetition/workload blocks, not
formally counterbalanced; interpret small effects near the observed drift as
conditionally valid and replicate them with more blocks.
