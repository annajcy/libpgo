# Eigen/oneMKL per-task nested-inner-arena benchmark

This benchmark tests whether giving every outer task its own persistent
`ArenaThreadingExecutor(1, ...)` can replace a oneMKL local thread budget of
one. Unlike `eigen_mkl_control_matrix`, it keeps the outer arena at width `C`
for every policy, so an inner arena never serializes unrelated outer tasks.

The four policies are:

| Policy | Outer local budget | Per-task inner arena | Inner local budget |
| --- | ---: | ---: | ---: |
| `OuterDefault` | 0 | none | n/a |
| `OuterLocal1` | 1 | none | n/a |
| `InnerArena1Default` | 0 | one distinct arena of width 1 | 0 |
| `InnerArena1Local1` | 0 | one distinct arena of width 1 | 1 |

The probe creates all inner executors before warm-up and timing. Each task
index owns one executor and one output matrix. Therefore the timed comparison
does not include executor construction or retirement and has no concurrent
reuse of an inner arena.

With oneMKL's TBB threading layer, process-global OpenMP thread setters do not
control parallelism. The probe therefore fixes TBB global concurrency to `C`
and records `process_default_mkl_max_threads` instead of claiming that the
oneMKL process default equals `C`. Local budget 0 clears the local override and
falls back to that recorded process default; the TBB controls still bound task
execution concurrency.

## Build and run

This target is Linux-only and requires oneMKL's TBB threading layer:

```bash
cmake --build build/base --target eigen_mkl_nested_inner_arena_probe

python benchmarks/eigen_mkl_nested_inner_arena/run_eigen_mkl_nested_inner_arena.py \
  build/base/benchmarks/eigen_mkl_nested_inner_arena/eigen_mkl_nested_inner_arena_probe \
  --out benchmark-results/eigen-mkl-nested-inner-arena
```

Defaults are `C=8`, 1024-by-1024 DGEMMs, outer-task counts 8 and 32, three
warm-up batches, 50 timed batches, and seven randomized fresh-process
repetitions. The runner writes every raw probe log, `timing.csv`, and
`nested-inner-arena.json`. Summary ratios are paired by repetition and use
`OuterLocal1` as the baseline.

Add `--collect-vtune` to collect Threading profiles for all four policies and
both outer-task counts:

```bash
python benchmarks/eigen_mkl_nested_inner_arena/run_eigen_mkl_nested_inner_arena.py \
  build/base/benchmarks/eigen_mkl_nested_inner_arena/eigen_mkl_nested_inner_arena_probe \
  --out benchmark-results/eigen-mkl-nested-inner-arena-vtune \
  --collect-vtune
```

Use `--sudo` if Linux `ptrace_scope` blocks VTune collection. The runner
restores result-directory ownership before exporting reports.

## Interpretation

`outer_active_peak` verifies that distinct inner arenas preserve outer
parallelism. `observed_mkl_max_threads_*` distinguishes the configured oneMKL
path: `InnerArena1Default` should still observe the recorded process default,
whereas
`InnerArena1Local1` should observe 1. `observed_inner_arena_concurrency_*`
verifies that both nested policies actually execute inside width-1 arenas.

The decisive comparisons are:

- `InnerArena1Default / InnerArena1Local1`: same nested topology, different
  oneMKL local budget;
- `InnerArena1Local1 / OuterLocal1`: both use a oneMKL budget of one, so the
  ratio isolates nested-arena overhead;
- `InnerArena1Default / OuterLocal1`: candidate substitute versus the current
  recommendation.

VTune's task report includes tasks from the outer `parallel_for` as well as
oneMKL, including the configured warm-up batches. Do not interpret its
absolute task count as an MKL-only count. Compare the two inner-arena policies,
whose outer topology and process GEMM count are identical, together with wall
time and the probe's oneMKL-budget telemetry.
