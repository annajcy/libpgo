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

`parallelism_accelerate_nested_benchmark.cpp` exposes four synthetic workload families:

- `NestedParallelDgemmAccelerate`: the BLAS-heavy positive/crossover control;
- `PolicyDecision/NoBlasControl`: scalar negative control;
- `PolicyDecision/FemElementBatch`: benchmark-only tetrahedral deformation and
  compressible energy arithmetic representative of small FEM element kernels;
- `PolicyDecision/ContactPairBatch`: benchmark-only point/triangle distance and
  barrier arithmetic representative of contact kernels.

`parallelism_real_fem_nested_benchmark.cpp` closes the earlier end-to-end FEM
coverage gap. It builds production `DeformationModelAssembler` instances and
benchmarks the real gather, material preparation, element evaluation, and
gradient/Hessian scatter paths for:

- `CubicLinear`: 24 local DOFs and 8 quadrature points per element;
- `CubicTricubicHermite`: 192 local DOFs and 64 quadrature points per element;
- energy, gradient, Hessian, and the combined energy+gradient+Hessian workload;
- one element count near the runtime concurrency and one smaller/larger scaling
  point appropriate to each formulation.

The synthetic FEM/contact families remain useful controls, but they are not
accepted as substitutes for the real FEM cases when choosing a backend default.
The assembler's original entry points still delegate to `Options{}`; explicit
`Options` overloads were added only so the benchmark can compare policies while
executing the identical production loop body.

Every case reports a deterministic checksum and runtime telemetry including
outer workers/tasks, problem size, peak process threads, pgo peak participants,
participant pressure, effective TBB parallelism, and the Accelerate threading
mode observed in a worker.

`run_nested_policy_decision.py`:

- discovers complete `Suppress`/`Inherit` pairs;
- runs every policy sample in a new process, preventing persistent Accelerate
  and TBB worker pools from contaminating the other policy;
- performs an explicit process-local Google Benchmark warm-up before recording
  each fresh-process measurement;
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
  --repetitions 10 --min-time 0.03s --warmup-time 0.10 --seed 20260712
```

This is 36 paired configurations: 18 DGEMM configurations spanning outer
workers `1, 4, 8`, task multipliers `1, 4`, and matrix sizes `256, 1024, 2048`,
plus all 18 representative/control configurations. Each configuration has ten
fresh-process pairs (720 benchmark child processes total).

### Result status

The previously recorded Accelerate sweep did not request process-local warm-up.
Fresh-process isolation therefore also repeated cold BLAS/runtime initialization,
so that artifact is superseded and must not be used to choose the default policy.
Rerun the command above after this revision before recording a new result table.

## Real cubic FEM sweep

Build on either supported backend:

```sh
cmake --build build/base --target parallelism_real_fem_nested_benchmark -j
```

Run all 16 paired configurations (two formulations, four evaluation kinds, and
two element counts):

```sh
python benchmarks/run_nested_policy_decision.py \
  build/base/benchmarks/parallelism_real_fem_nested_benchmark \
  --out build/benchmark-results/real-fem-$(uname -s).json \
  --filter '^RealFemPolicyDecision/' \
  --repetitions 10 --min-time 0.03s --warmup-time 0.10 --seed 20260712
```

On Linux, export `MKL_THREADING_LAYER=TBB` before running. The result families
are reported separately as `CubicLinear/{Energy,Gradient,Hessian,Full}` and
`CubicTricubicHermite/{Energy,Gradient,Hessian,Full}`. In particular, the
Hermite Hessian executes the dynamic `dFdx.transpose() * dPdF * dFdx` products
that exercise Eigen's MKL GEMM integration in an MKL build.

### macOS real-FEM result (2026-07-12)

The 16 configurations completed with ten randomized fresh-process pairs each;
all policy-pair checksums matched. Ratios below are family geometric means of
the per-configuration median `Inherit / Suppress` wall times:

| Family | Ratio | Classification |
|---|---:|---|
| CubicLinear / Energy | 0.913 | Inherit faster |
| CubicLinear / Gradient | 1.013 | equivalent |
| CubicLinear / Hessian | 0.967 | equivalent |
| CubicLinear / Full | 0.898 | Inherit faster |
| CubicTricubicHermite / Energy | 0.958 | equivalent |
| CubicTricubicHermite / Gradient | 0.968 | equivalent |
| CubicTricubicHermite / Hessian | 1.004 | equivalent |
| CubicTricubicHermite / Full | 1.004 | equivalent |

The aggregate real-FEM geometric mean is `0.965`, a 3.5% Inherit advantage.
That does not clear the predeclared 5% default-change threshold. The expensive
Hermite Hessian and Full cases have much tighter intervals than the short
CubicLinear cases and are effectively equal: for 8 elements their medians and
bootstrap intervals are `1.013 [0.990, 1.027]` and
`1.008 [0.984, 1.025]`, respectively. This is evidence that Suppress does not
help these real FEM paths on the current macOS build, but it is not sufficient
under the fixed rule to change the platform default to Inherit.

Artifact: `build/benchmark-results/real-fem-macos.json` (local build tree).

An MKL `MKL_VERBOSE=1` single-iteration probe confirms that the Hermite Hessian
is not merely large Eigen arithmetic: one policy pair emitted 2,048 calls each
of `DGEMM(192,192,9)` and `DGEMM(192,9,9)` (1,024 of each shape per policy), in
addition to the smaller preparation products. The probe is functional evidence
only and is excluded from timing because another server workload was active.
The Linux regression test
`ParallelNestedKernelTest.MklSuppressAndInheritRestoreLocalThreads` verifies that
Suppress exposes a worker-local MKL maximum of 1, Inherit exposes the caller's
maximum, and both restore the caller state afterward.

### Linux MKL-TBB real-FEM result (2026-07-12)

The server's existing experiment was suspended with `SIGSTOP` for the complete
sweep and restored with `SIGCONT` immediately afterward. The 16 configurations
completed with ten randomized fresh-process pairs each; all checksums matched
and the benchmark exited successfully. Family geometric means are:

| Family | Ratio | Classification |
|---|---:|---|
| CubicLinear / Energy | 0.999 | equivalent |
| CubicLinear / Gradient | 1.002 | equivalent |
| CubicLinear / Hessian | 1.005 | equivalent |
| CubicLinear / Full | 0.988 | equivalent |
| CubicTricubicHermite / Energy | 0.988 | equivalent |
| CubicTricubicHermite / Gradient | 1.022 | equivalent |
| CubicTricubicHermite / Hessian | 1.024 | equivalent |
| CubicTricubicHermite / Full | 1.097 | Suppress faster |

The aggregate real-FEM geometric mean is `1.015`. Most cases are equivalent,
but the decision-critical Hermite Full workload has median ratios of
`1.077 [1.038, 1.130]` for 8 elements and
`1.117 [1.018, 1.210]` for 16 elements. The 16-element case exceeds the
predeclared 10% important-regression threshold. Both policies had the same
median process-thread peak of 17, consistent with oneTBB enforcing the runtime
ceiling while the inherited MKL request still incurs additional scheduling or
kernel-selection overhead.

Artifact: `build/benchmark-results/real-fem-mkl.json` (copied from the Linux
host into the local build tree).

### Default-policy conclusion

- Keep `Suppress` as the MKL default; both the earlier nested-DGEMM evidence and
  the real Hermite Full workload reject a portable Inherit default.
- The current macOS real-FEM data shows no important Inherit regression, but its
  3.5% aggregate gain does not clear the fixed 5% threshold. It therefore does
  not justify changing the Accelerate default to Inherit yet.
- Keep the existing portable `Suppress` default for now. A future
  backend-specific Accelerate=Inherit proposal needs either a stronger stable
  gain on production call sites or an explicitly revised acceptance rule.

## Linux MKL-TBB synthetic sweep

The MKL executable registers the same shared no-BLAS, FEM, and contact kernels
as Accelerate, in addition to its DGEMM family. It also exports the checksum,
process/pgo telemetry, and the worker-local value of `mkl_get_max_threads()`.
The same runner supports both executables. Any result produced before the
explicit runner warm-up and GCC checksum revision is superseded and must be
rerun.

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
  --repetitions 10 --min-time 0.03s --warmup-time 0.10 --seed 20260712
```

Do not combine the Accelerate and MKL raw measurements as though they came from
one host. Compare family directions and threshold violations across the two JSON
summaries.

## Warm-up/checksum revision validation

The implementation review found that fresh-process runs repeated cold runtime
initialization and that the non-const GCC `DoNotOptimize(checksum)` call could
overwrite the DGEMM checksum with a pointer bit pattern. This revision adds an
explicit per-process warm-up argument, records it in the result JSON, and removes
the mutating checksum barrier while retaining the checksum as an exported counter.

Focused macOS validation after the revision:

- Python byte-compilation: passed;
- Accelerate benchmark rebuild: passed;
- two fresh Suppress/Inherit DGEMM pairs with warm-up: passed;
- checksum was finite, plausible, and equal (`136.236024`) for both policies;
- `ParallelNestedKernelTest.AccelerateSuppressAndInheritRestoreThreading`: passed;
- `git diff --check`: passed.

The Linux/GCC build, checksum regression test, and real cubic FEM sweep now pass.
The broader synthetic DGEMM/contact sweep described above remains a separate
follow-up; prior pre-warm-up artifacts are still not accepted decision inputs.

## Reviewer checklist

- Confirm every raw pair uses two distinct child processes.
- Confirm `settings.warmup_time_seconds` records the intended process-local
  warm-up.
- Confirm `policy_order` varies under seed `20260712`.
- Confirm checksums are finite, numerically plausible, and match within the
  fixed `1e-10` relative tolerance.
- Independently recompute several `Inherit / Suppress` ratios from raw wall time.
- Confirm any proposed default change satisfies all predeclared thresholds and
  has Linux MKL-TBB evidence in the same direction.
