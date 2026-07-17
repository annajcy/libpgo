# CubicLinear PARDISO-aftermath diagnostic

This benchmark isolates why CubicLinear `evaluation=1` became slower after a
`linear=8` phase in the phase-aware Newton benchmark. Every measured case uses
the same real `cubic/dragon-center-r16.veg`, Stable Neo-Hookean CubicLinear
energy, deterministic nonzero displacement, fixed-top boundary, and
`ArenaThreadingExecutor` with evaluation MKL budget 1.

Immediately before each FGH evaluation, the probe performs one of five
operations:

| case | immediate prelude | linear MKL budget |
| --- | --- | ---: |
| `none` | no executor entry | — |
| `noop1` | empty linear-executor call | 1 |
| `noop8` | empty linear-executor call | 8 |
| `pardiso1` | real factorize + solve | 1 |
| `pardiso8` | real factorize + solve | 8 |

The backend is constructed and symbolically analyzed once under budget 1 for
all cases. Sparse reduction for the next iteration occurs after the measured
FGH, so no preparation work sits between the prelude and evaluation. The probe
records both the complete evaluation-executor call and a clock strictly around
`func_grad_hessian` inside its lambda.

The primary discriminators are:

- `noop8 / noop1`: cost of changing arenas/TLS policies without MKL work;
- `pardiso8 / pardiso1`: aftermath of actual multi-threaded PARDISO work.

If only the second ratio is greater than one, the evidence supports cache,
scheduler, or runtime state left by real MKL/TBB work. If the first ratio also
grows, executor/policy switching contributes. `none` measures the absolute
boundary baseline.

Build and run on the MKL TBB Linux configuration:

```bash
cmake --build build/base --target cubic_linear_pardiso_aftermath_probe -j8
python benchmarks/cubic_linear_pardiso_aftermath/run_cubic_linear_pardiso_aftermath.py \
  build/base/bin/cubic_linear_pardiso_aftermath_probe \
  --mesh examples/assets/veg/cubic/dragon-center-r16.veg \
  --out benchmarks/results/cubic-linear-pardiso-aftermath \
  --concurrency 8 --cpu-list 21-28
```

The controller randomizes the case order inside every repetition, runs each
sample in a fresh process, verifies MKL TBB linkage and observed budgets, and
rejects a repetition if FGH dimensions or numerical signatures differ. It
writes raw iteration data, one median sample per worker, and summary CSV/JSON.
