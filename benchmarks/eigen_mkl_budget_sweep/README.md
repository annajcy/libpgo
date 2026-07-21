# Eigen/oneMKL threading surface

This benchmark maps oneMKL's TBB behavior over the Cartesian product of the
thread-local budget and enclosing private TBB arena concurrency. It separates
finer task decomposition from harmful over-decomposition by measuring both an
isolated GEMM and outer-parallel GEMM batches.

The benchmark is Linux-only and requires `MKL::MKL` configured with the
`tbb_thread` threading layer. It consists of:

- `eigen_mkl_budget_sweep_probe`, which accepts arbitrary global concurrency,
  arena concurrency, oneMKL local budget, outer-task count, and matrix size;
- `run_eigen_mkl_budget_sweep.py`, which randomizes fresh-process timing runs,
  optionally collects VTune Threading profiles, and writes a machine-readable
  `budget-sweep.json`.

The runner does not modify `MKL_NUM_THREADS` or `MKL_DYNAMIC`. It records their
inherited values in the result so budget `0` can be interpreted correctly:
`mkl_set_num_threads_local(0)` clears the local override and falls back to the
process-global oneMKL setting.

## Timing sweep

Build the probe:

```bash
cmake --build build/base --target eigen_mkl_budget_sweep_probe
```

The default surface uses arena widths `1, 2, 4, 8`, local budgets
`0, 1, 2, 4, 8, 16`, and outer-task counts `1, 8, 32` (for a global
concurrency of 8):

```bash
python benchmarks/eigen_mkl_budget_sweep/run_eigen_mkl_budget_sweep.py \
  build/base/benchmarks/eigen_mkl_budget_sweep/eigen_mkl_budget_sweep_probe \
  --out benchmark-results/eigen-mkl-budget-sweep
```

The default workload uses 1024-by-1024 DGEMMs, ten warm-up batches, 50 timed
batches, and twelve counterbalanced fresh-process repetitions per case. Use
`timing_summary` in `budget-sweep.json` for wall-time comparisons. It reports
the median and median absolute deviation rather than selecting a best run.

## VTune decomposition profiles

Add `--collect-vtune` to profile the isolated `outer_tasks=1` surface. This is
the only default profile mode where `task/GEMM` is attributable solely to
oneMKL; the timing surface still covers outer-task counts 8 and 32. The default
collects twelve VTune repetitions for each arena/budget cell so the arena and
budget order cycles are both complete.

```bash
python benchmarks/eigen_mkl_budget_sweep/run_eigen_mkl_budget_sweep.py \
  build/base/benchmarks/eigen_mkl_budget_sweep/eigen_mkl_budget_sweep_probe \
  --out benchmark-results/eigen-mkl-budget-sweep-vtune \
  --collect-vtune
```

Pass `--vtune /path/to/vtune` when it is not on `PATH`. If Linux
`ptrace_scope` blocks collection, add `--sudo`; the runner restores ownership
of each VTune result directory before exporting reports.

For `outer_tasks=1`, `tbb_tasks_per_process_gemm` normalizes VTune's
`tbb_parallel_for` task count by all GEMMs seen by the collector, including
warm-up calls. This is the clean internal-decomposition comparison. For
`outer_tasks>1`, the task report contains both outer and oneMKL TBB tasks, so
the runner deliberately leaves that normalized field null; use those profiles
for wall time, CPU utilization, spin, and wait behavior instead.

VTune's `Task Count` is the number of instrumented TBB task instances that
executed. It is not necessarily the number of task objects initially allocated
by oneMKL. More tasks than workers establish finer decomposition, but they are
only evidence of harmful over-decomposition when accompanied by worse wall
time, spin, or wait behavior under outer parallel load.

## Focused variants

Use a smaller surface or change outer pressure explicitly:

```bash
python benchmarks/eigen_mkl_budget_sweep/run_eigen_mkl_budget_sweep.py \
  build/base/benchmarks/eigen_mkl_budget_sweep/eigen_mkl_budget_sweep_probe \
  --out benchmark-results/eigen-mkl-budget-sweep-focused \
  --arena-concurrencies 1 4 8 \
  --mkl-local-thread-budgets 0 8 16 \
  --outer-tasks 1 8 \
  --profile-outer-tasks 1 \
  --timing-repetitions 3 \
  --profile-repetitions 1 \
  --allow-incomplete-order-cycle \
  --collect-vtune
```

Use `--dry-run` to print the randomized probe and VTune commands without
creating an output directory.
