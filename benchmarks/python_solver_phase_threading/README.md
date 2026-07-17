# Python Newton phase-threading benchmark

This benchmark tests whether semantic phase-aware threading improves a real
Newton solve driven entirely through the public Python API. It does not call a
benchmark-only C++ kernel or expose Newton's internal state machine to Python.

Every timed solve performs exactly one Newton iteration with:

- a real Vega volumetric mesh loaded by `pypgo.mesh.volume.read_veg`;
- a real `pypgo.fem.deformation_energy` using `StableNeo` by default;
- a deterministic nonzero perturbation of the deformation DOFs;
- a fixed top boundary that removes rigid-body null modes;
- `NewtonOptimizer`, `Backtrack`, `NoDamping`, and `MKLPardiso`;
- `max_iterations=1` and `gradient_tolerance=0.0`.

The public `optimizer.solve()` path therefore includes the fixed-topology
initial Hessian and symbolic analysis, the iteration's energy/gradient/Hessian,
PARDISO factorization and solve, line search, and final objective evaluation.

## Workloads

The three defaults use separate mesh sizes so both FEM evaluation and sparse
linear algebra have measurable cost without making the Hermite Hessian
impractically large.

| workload | default mesh | vertices | elements | DOFs |
| --- | --- | ---: | ---: | ---: |
| `tet_linear` | `tet/box-with-sphere-big.veg` | 5266 | 21570 | 15798 |
| `cubic_linear` | `cubic/dragon-center-r16.veg` | 4080 | 2456 | 12240 |
| `cubic_tricubic_hermite` | `cubic/box.veg` | 125 | 64 | 3000 |

The builder computes connected components from the volume elements and fixes
every formulation DOF belonging to the top 1% slab of each component. If a
component's slab contains fewer than three non-collinear vertices, it expands
downward by the minimum number of height-sorted vertices needed to remove that
component's rigid modes. This matters for the default tet mesh, whose box and
sphere are disconnected. Fixed values are zero (the rest displacement), and
deterministic Gaussian noise is added only to free DOFs. The result records the
component sizes, expansion decisions, selected DOF count, and a SHA-256 digest
of the fixed indices.

## Policies

With the default concurrency `C=8`, the benchmark compares:

| policy | evaluation MKL budget | linear-solver MKL budget | execution form |
| --- | ---: | ---: | --- |
| `uniform_single` | 1 | 1 | one outer executor around the whole solve |
| `uniform_multi` | C | C | one outer executor around the whole solve |
| `phase_aware` | 1 | C | `NewtonThreadingPolicy` |
| `phase_reversed` | C | 1 | mechanism-negative control |
| `phase_single_single` | 1 | 1 | two independent phase executors |

All executors use the same arena concurrency and reserved-slot setting.
`phase_single_single` versus `uniform_single` estimates the cost of the phase
dispatch abstraction while holding the effective thread budgets fixed.

## Run

Use a fresh Release build with MKL PARDISO available, then run on an otherwise
idle machine:

```bash
python benchmarks/python_solver_phase_threading/run_python_solver_phase_threading_benchmark.py \
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
material, energy, or optimizer. It performs one full warmup solve and one timed
solve by default. Policy order is randomized within each
`(repetition, workload)` block, and blocks are also randomized.

Quick scheduling check without loading `pypgo`:

```bash
python benchmarks/python_solver_phase_threading/run_python_solver_phase_threading_benchmark.py \
  --dry-run --workloads cubic_tricubic_hermite --repetitions 1
```

A small end-to-end smoke run after rebuilding:

```bash
python benchmarks/python_solver_phase_threading/run_python_solver_phase_threading_benchmark.py \
  --out /tmp/python-solver-phase-smoke \
  --workloads cubic_tricubic_hermite \
  --policies uniform_single phase_aware \
  --repetitions 1 --bootstrap-samples 100
```

`--timed-solves` is available for unusually fast custom meshes, but the
defaults intentionally use one fixed timed solve per fresh process. This avoids
an adaptive minimum-time loop in which a faster policy might execute more
warm-cache solves than a slower policy.

## Outputs and interpretation

The output directory contains:

- `python-solver-phase-threading.json`: manifest, run completion/validity
  status, every raw measurement, correctness signatures, phase diagnostics,
  and summaries;
- `runs.csv`: one row per fresh worker sample;
- `phase_breakdown.csv`: one row per timed solve with phase and Newton timing;
- `summary.csv`: median, MAD, range, paired ratio to `phase_aware`, and a paired
  bootstrap 95% interval.

For `paired_ratio_over_phase_aware_*`, a value greater than 1 means
`phase_aware` was faster. Comparisons are paired by workload and repetition.
The controller validates before accepting a completed block that all policies
have the same solver status, iteration and line-search path, sparse-system
shape, phase call counts, final objective, and result vector. Every DOF is
compared with configurable absolute and relative tolerances. The JSON also
records an exact SHA-256 of canonical little-endian float64 result bytes; this
hash is provenance evidence and is not used in place of the tolerant DOF-wise
comparison. When `--timed-solves` is greater than one, every solve is checked
inside its worker before cross-policy validation.

Check `run_status` before analyzing a checkpoint. A usable final result has
`state="complete"`, `complete=true`, `valid_so_far=true`, all scheduled blocks
validated, and every row marked `validation_status="valid"`. A worker or
signature failure rewrites the checkpoint with `state="failed"`, an error
message, and invalid/incomplete status rather than leaving an apparently valid
summary. `--case-limit` is recorded together with requested and actual schedule
counts, so a deliberately truncated smoke run is distinguishable from a full
run.

The primary metric is public `optimizer.solve()` wall time. The phase counters
are mechanism diagnostics: they show whether evaluation follows the
single-thread-budget behavior and PARDISO follows the multi-thread-budget
behavior. Wall time also includes small common Python API costs such as bounds
synchronization, input conversion, and result conversion. Do not assume phase
times must add exactly to wall time unless the build documents that final
objective evaluation and backend release are included in the counters.

A result supports the mechanism when:

1. numerical and solver-path signatures match across all five policies;
2. `phase_aware` beats both uniform policies by more than run-to-run variation;
3. its evaluation phase approaches `uniform_single` while its linear phase
   approaches `uniform_multi`;
4. `phase_single_single` remains close to `uniform_single`;
5. `phase_reversed` exhibits the predicted unfavorable phase behavior.

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
