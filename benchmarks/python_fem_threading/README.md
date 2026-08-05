# Python FEM threading benchmark

This benchmark measures public `pypgo` FEM energy evaluations under different
`ArenaThreadingExecutor` policies. It defaults to the Linux oneMKL policy
sweep; see [README-macos.md](README-macos.md) for the Accelerate SINGLE/MULTI
variant. The workload is deliberately pure Python:

1. load a real Vega volumetric mesh with `pypgo.mesh.volume.read_veg`;
2. build `VolumeMesh` and `pypgo.fem.SimulationImportResult`;
3. construct a `MaterialBinding`, `MaterialState`, `DeformationEnergyOperator`, and
   `DeformationPotentialEnergy`;
4. call the public `value`, `gradient`, or `hessian` API inside an executor.

There is no benchmark-only C++ kernel, binding, or CMake target. The runner uses
Python subprocesses only to give every timed sample a fresh oneMKL/oneTBB
process state.

## Workloads

The default sweep covers:

| formulation | real input mesh | local element DOFs |
| --- | --- | ---: |
| `tet_linear` | `examples/assets/veg/tet/box.veg` | 12 |
| `cubic_linear` | `examples/assets/veg/cubic/box.veg` | 24 |
| `cubic_tricubic_hermite` | `examples/assets/veg/cubic/box.veg` | 192 |

Each formulation is measured separately for `value`, `gradient`, and
`hessian`. Absolute times are not compared across formulations; the relevant
quantity is each policy's speed ratio within one formulation and operation.

The default elastic model is `StableNeoDefinition`, with an elementwise
optimizable/fixed field
initialized from the `.veg` material payload. The default plastic model is
`VolumetricPlasticityDefinition(dofs=6)`. A small deterministic nonzero displacement is
used so correctness checks do not depend on a trivial rest-state result. Every
measurement records the perturbation generator, seed, scale, nonzero count,
L2 norm, and maximum absolute DOF value so the timed state can be audited from
the result JSON without inspecting the runner.

## Threading policies

All cases establish the same process-wide maximum concurrency and run with
`MKL_NUM_THREADS=C`, `OMP_NUM_THREADS=C`, and `MKL_DYNAMIC=FALSE` set before
NumPy or `pypgo` is imported in the worker process. The worker creates
`GlobalTbbControl(C)` immediately after importing the public parallel API and
before constructing the volume mesh, material models, or deformation energy.
The default is `C=8`.

| policy | arena concurrency | oneMKL local budget |
| --- | ---: | ---: |
| `no_executor` | default arena | global setting |
| `budget_0` | C | 0 (clear the local override) |
| `budget_1` | C | 1 |
| `budget_c` | C | C |
| `arena_1_budget_1` | 1 | 1 |

`arena_1_budget_1` is a negative control: it distinguishes serializing inner
BLAS work from serializing the entire element assembly.

## Run

Use an MKL build configured with the oneTBB threading layer, then run:

```bash
python benchmarks/python_fem_threading/run_python_fem_threading_benchmark.py \
  --out benchmarks/results/python-fem-threading \
  --concurrency 8
```

The full default sweep is 3 formulations x 3 operations x 5 policies x 10
fresh-process repetitions. For a quick smoke test:

```bash
python benchmarks/python_fem_threading/run_python_fem_threading_benchmark.py \
  --out /tmp/python-fem-threading-smoke \
  --formulations tet_linear \
  --operations value \
  --policies no_executor budget_1 \
  --repetitions 1 \
  --allow-incomplete-order-cycle \
  --min-time 0.01
```

Use larger real assets without changing the benchmark implementation:

```bash
python benchmarks/python_fem_threading/run_python_fem_threading_benchmark.py \
  --out benchmarks/results/python-fem-threading-large \
  --tet-mesh examples/assets/veg/tet/torus.veg \
  --cubic-mesh examples/assets/veg/cubic/bunny.veg \
  --concurrency 8
```

The Hermite Hessian template grows quickly with mesh size. Check memory usage
before selecting the largest cubic assets.

Add `--native-profile` for public `pypgo.profiling` section statistics. Profiling
adds measurement overhead and should be collected in a separate run from the
primary timing sweep.

## Result

The runner writes `python-fem-threading.json` after every completed sample. It
contains:

- the complete run manifest and effective environment;
- every fresh-process measurement and correctness signature;
- median and median absolute deviation per policy;
- paired `policy / budget_1` time ratios for matching repetitions.

A paired ratio greater than 1 means `budget_1` was faster. The runner checks
energy, gradient, and Hessian signatures across policies before accepting a
complete repetition block.

The mechanism-specific prediction is not that every workload improves equally.
The strongest expected signal is an outer-saturated Hermite Hessian where
`budget_1` beats `budget_0` and `budget_c`, while the 12-DOF and 24-DOF element
kernels show a much smaller policy effect. If the timing result is inconclusive,
profile the same cases before attributing the outcome to oneMKL task
decomposition.
