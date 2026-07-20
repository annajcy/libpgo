# macOS Accelerate Python FEM benchmark

This is the macOS variant of the pure-Python FEM threading benchmark. It uses
the same real `.veg` meshes, material construction, perturbed DOF vectors, and
public `pypgo.fem.DeformationEnergy` evaluation paths as the MKL variant. The
only measured policy difference is:

| policy | arena concurrency | Accelerate mode |
| --- | ---: | --- |
| `accelerate_single` | C | `AccelerateThreading.SINGLE` |
| `accelerate_multi` | C | `AccelerateThreading.MULTI` |

Both policies use the same `ArenaThreadingExecutor`, reserved slots, mesh,
material model, operation, and deterministic displacement perturbation. The
worker creates `GlobalTbbControl(8)` by default before volumetric mesh,
material, or energy construction and keeps it alive through warmup and timing.
The runner does not set `VECLIB_MAXIMUM_THREADS` or
`VECLIB_DEFAULT_THREAD_COUNT`, because either environment override could cap
the `MULTI` case independently of `BLASSetThreading`. Their inherited values
are recorded in every measurement.

## Run

```bash
python benchmarks/python_fem_threading/run_python_fem_accelerate_benchmark.py \
  --out benchmarks/results/python-fem-accelerate \
  --concurrency 8
```

The default sweep is 3 formulations x 3 operations x 2 policies x 10
fresh-process repetitions. For a quick smoke test:

```bash
python benchmarks/python_fem_threading/run_python_fem_accelerate_benchmark.py \
  --out /tmp/python-fem-accelerate-smoke \
  --formulations cubic_tricubic_hermite \
  --operations hessian \
  --repetitions 1 \
  --allow-incomplete-order-cycle \
  --min-time 0
```

The output is `python-fem-threading.json`. For every formulation and operation,
`paired_time_ratios.accelerate_multi_over_accelerate_single` is:

```text
MULTI seconds per evaluation / SINGLE seconds per evaluation
```

A ratio greater than 1 means `SINGLE` was faster; a ratio below 1 means
`MULTI` was faster. Compare ratios within a formulation/operation rather than
comparing absolute times across formulations.

The runner rejects `backend=accelerate` outside macOS and checks numerical
result signatures across the two policies before accepting a repetition.
