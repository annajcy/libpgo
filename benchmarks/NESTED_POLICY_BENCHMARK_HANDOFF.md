# Nested-kernel policy benchmark handoff

## Scope

This implements the accepted one-day diagnostic benchmark for deciding whether
`NestedKernelPolicy::Inherit` has enough evidence to replace `Suppress` as the
default. It does not change the default or any production call site.

The benchmark keeps the decision thresholds fixed before observing results:

- practical equivalence: `Inherit / Suppress` within `1.00 ± 0.05`;
- important regression: `Inherit / Suppress > 1.10` for any important case;
- minimum representative-workload gain: geometric mean
  `Inherit / Suppress <= 0.95`;
- a default change additionally requires the direction to agree on macOS
  Accelerate and Linux MKL-TBB.

## Implementation

`parallelism_accelerate_nested_benchmark.cpp` now exposes four workload families:

- `NestedParallelDgemmAccelerate`: the BLAS-heavy positive/crossover control;
- `PolicyDecision/NoBlasControl`: scalar negative control;
- `PolicyDecision/FemElementBatch`: benchmark-only tetrahedral deformation and
  compressible energy arithmetic representative of small FEM element kernels;
- `PolicyDecision/ContactPairBatch`: benchmark-only point/triangle distance and
  barrier arithmetic representative of contact kernels.

The FEM and contact cases intentionally do not instantiate production assemblers.
Doing that would require large mesh/energy setup graphs and would make this
diagnostic executable depend on mutable public APIs. This is recorded plan drift:
they are near-real arithmetic kernels, not end-to-end workloads. A later full
study should add end-to-end scenarios without changing the decision thresholds.

Every case reports a deterministic checksum and runtime telemetry including
outer workers/tasks, problem size, peak process threads, pgo peak participants,
participant pressure, effective TBB parallelism, and the Accelerate threading
mode observed in a worker.

`run_nested_policy_decision.py`:

- discovers complete `Suppress`/`Inherit` pairs;
- runs every policy sample in a new process, preventing persistent Accelerate
  and TBB worker pools from contaminating the other policy;
- randomizes both pair order and within-pair policy order with a recorded seed;
- rejects checksum mismatches;
- normalizes Google Benchmark wall/CPU times to seconds;
- emits raw measurements, paired ratios, median ratios, deterministic bootstrap
  95% intervals, family geometric means, thread telemetry, and a recommendation
  in one JSON artifact.

The benchmark is deliberately not registered with CTest.

Validation budget: targeted build, benchmark smoke, policy unit test, and result
artifact integrity checks. The Accelerate target rebuilt successfully after the
shared-workload refactor. A two-pair post-refactor smoke for both FEM and contact
completed without checksum errors. The existing
`ParallelNestedKernelTest.AccelerateSuppressAndInheritRestoreThreading` CTest
passed. Python byte-compilation, fixed-threshold helper assertions, JSON
integrity assertions, and `git diff --check` also passed.

## macOS Accelerate run

Build:

```sh
conda run -n libpgo cmake --build build/base \
  --target parallelism_accelerate_nested_benchmark -j 6
```

Decision sweep:

```sh
conda run -n libpgo python benchmarks/run_nested_policy_decision.py \
  build/base/benchmarks/parallelism_accelerate_nested_benchmark \
  --out benchmarks/results/nested_policy_accelerate_20260712.json \
  --filter '^(PolicyDecision/|NestedParallelDgemmAccelerate/(Suppress|Inherit)/workers_(1|4|8)/tasks_(1|4|8|16|32)/n_(256|1024|2048))' \
  --repetitions 10 --min-time 0.03s --seed 20260712
```

This is 36 paired configurations: 18 DGEMM configurations spanning outer
workers `1, 4, 8`, task multipliers `1, 4`, and matrix sizes `256, 1024, 2048`,
plus all 18 representative/control configurations. Each configuration has ten
fresh-process pairs (720 benchmark child processes total).

### Result

The sweep completed all 360 fresh-process pairs with no checksum mismatch. The
randomized order was balanced (`Suppress` first 183 times, `Inherit` first 177
times). The raw JSON is intentionally under the repository's ignored
`benchmarks/results/` directory and remains available locally as
`nested_policy_accelerate_20260712.json`.

| Family | Cases | Geomean `Inherit / Suppress` | Classification |
|---|---:|---:|---|
| DGEMM | 18 | 0.893 | Inherit faster |
| No-BLAS control | 6 | 0.977 | Equivalent |
| FEM element batch | 6 | 0.968 | Equivalent |
| Contact pair batch | 6 | 1.003 | Equivalent |

The combined FEM/contact representative geomean is `0.985`, a 1.5% Inherit
gain. It does not clear the predeclared 5% threshold. Three case medians crossed
the predeclared 10% regression line, although two have wide intervals and should
not be overinterpreted individually:

- DGEMM, workers 1/tasks 1/N 256: `1.252`, bootstrap 95% interval
  `[0.780, 2.780]`;
- FEM, workers 1/tasks 1: `1.120`, interval `[1.011, 1.235]`;
- contact, workers 4/tasks 4: `1.104`, interval `[0.892, 1.671]`.

The controlled BLAS crossover is clear. With one outer worker, large DGEMM
strongly favors Inherit: N 1024 ratios are `0.611` (one task) and `0.584` (four
tasks), while N 2048 ratios are `0.532` and `0.522`. At outer concurrency 4 and
8, most DGEMM cases are equivalent; workers 4/tasks 4/N 2048 favors Suppress at
`1.093`, while workers 8/N 1024 favors Inherit (`0.924` for 8 tasks and `0.894`
for 32 tasks). Inherit also raises the median process thread peak in the
workers 8/tasks 32/N 1024 case from 9 to 17.

The predeclared recommendation is therefore to retain `Suppress` as the global
default. Inherit is valuable for known low-outer-concurrency, large-BLAS call
sites, but the representative workload gain is below threshold, meaningful
case-level regressions exist, resource use can increase, and Linux cross-platform
evidence is still missing.

## Linux MKL-TBB gap

Linux MKL-TBB was not available in the current macOS environment and has not
been run. The MKL executable now registers the same shared no-BLAS, FEM, and
contact kernels as Accelerate, in addition to its DGEMM family. It also exports
the checksum, process/pgo telemetry, and the worker-local value of
`mkl_get_max_threads()`. The same runner supports both executables.

Linux prerequisites are oneTBB, Google Benchmark, an MKL installation visible
to CMake (`MKLROOT` when required), a build configured with
`PGO_USE_MKL=ON`, `PGO_BUILD_BENCHMARKS=ON`, and
`PGO_MKL_THREADING=tbb_thread`, plus `MKL_THREADING_LAYER=TBB` in the runtime
environment. A corresponding run is:

```sh
export MKL_THREADING_LAYER=TBB
cmake --preset base -DPGO_USE_MKL=ON -DPGO_BUILD_BENCHMARKS=ON \
  -DPGO_MKL_THREADING=tbb_thread
cmake --build build/base --target parallelism_mkl_nested_benchmark -j
python benchmarks/run_nested_policy_decision.py \
  build/base/benchmarks/parallelism_mkl_nested_benchmark \
  --out benchmarks/results/nested_policy_mkl_tbb.json \
  --filter '^(PolicyDecision/|NestedParallelDgemm/(Suppress|Inherit)/workers_(1|4|8)/tasks_(1|4|8|16|32)/n_(512|1024|2048))' \
  --repetitions 10 --min-time 0.03s --seed 20260712
```

Do not combine the Accelerate and MKL raw measurements as though they came from
one host. Compare family directions and threshold violations across the two JSON
summaries.

## Reviewer checklist

- Confirm every raw pair uses two distinct child processes.
- Confirm `policy_order` varies under seed `20260712`.
- Confirm checksums match within the fixed `1e-10` relative tolerance.
- Independently recompute several `Inherit / Suppress` ratios from raw wall time.
- Confirm any proposed default change satisfies all predeclared thresholds and
  has Linux MKL-TBB evidence in the same direction.
