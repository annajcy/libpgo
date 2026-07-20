# Benchmark execution policy

Top-level runners use workload-specific warmup.  The exact case, problem size,
backend, and threading policy that will be measured first execute in the same
fresh process until both fixed lower bounds are satisfied:

```text
elapsed >= warmup_seconds AND completed_operations >= warmup_min_operations
```

Warmup timing does not adapt either bound.  Runners reset measurement counters
afterward without destroying the warmed executor, backend, or thread pools.
Stateful workloads restore the same semantic input before every timed solve.
The default bounds are provisional until the calibration described below is
complete; formal runs must record the selected values in their manifests.

Measured policy order uses a deterministic Williams design within each
repetition/workload block. This balances temporal positions and first-order
carry-over while retaining a reproducible seed. A plain cyclic Latin rotation
is insufficient because it repeats the same policy adjacencies in every row.
The required `order_cycle_length` is derived from the number of compared
values: `N` for even `N`, `2N` for odd `N`, and the least common multiple for
nested ordered dimensions. Timing repetitions must be a multiple of that
derived cycle length.

Common options are available on every runner:

```text
--cpu-list
--numa-node
--allow-cross-numa
--allow-incomplete-order-cycle
```

Workload runners additionally expose `--warmup-seconds` and
`--warmup-min-operations`.  Google Benchmark-backed runners pass the time bound
to `--benchmark_min_warmup_time`; its calibrated iteration loop supplies many
more than the minimum operation count for the microbenchmarks used here.

`--allow-incomplete-order-cycle` is an explicit diagnostic escape hatch. Such
a run records `strictly_balanced = false` and must not be mixed into strict
timing comparisons.

On Linux, the manifest records requested and effective CPU affinity,
CPU/core/socket/NUMA topology, cgroup CPU placement and quota, cpufreq
driver/governor/frequency bounds, load average, and CPU pressure.  These are
read-only provenance checks; the harness does not run a proxy workload.

For strict server runs, pass affinity as a benchmark parameter, for example
`--cpu-list 21-28 --numa-node 0`. The controller applies it before workload
warmup, verifies that its size equals benchmark concurrency, and all benchmark
subprocesses inherit it. An affinity inherited from `taskset` is also accepted
when its size already matches concurrency.
Cross-NUMA placement is rejected unless `--allow-cross-numa` is explicit. Do
not compile, profile, or run unrelated jobs concurrently on the allocated
CPUs.  Complete Williams cycles and paired analysis, rather than online timing
rejection, control smooth temporal drift and first-order carry-over.

Before a formal server campaign, run `run_harness_placebo.py` against one
representative oneMKL Google Benchmark case. Its `placebo_a` and `placebo_b`
labels execute the same case, command parameters, environment, and fresh-process
path in a complete AB/BA cycle. The gate rejects material label bias, temporal
position bias, or warmup-path bias before any real policy contrast is
interpreted. The default fixed acceptance region requires every median ratio
to remain within 2% of one and every bootstrap 95% interval to fit inside a 5%
equivalence band, in addition to strict affinity and identical correctness
counters.

For example, on the eight-core Linux allocation:

```bash
python3 benchmarks/run_harness_placebo.py \
  build/base/benchmarks/eigen_mkl_crossover/eigen_mkl_crossover_benchmark \
  --case EigenMklCrossover/ExecutorLocal1/n_128/real_time \
  --out benchmark-results/harness-placebo.json \
  --concurrency 8 --cpu-list 21-28 --numa-node 0
```

## Warmup calibration API

`run_warmup_calibration.py` runs one subject over the counterbalanced candidate
set `0,0.25,0.5,1,2,4` seconds in fresh processes.  Commands are templates with
`{warmup_seconds}`, `{warmup_min_operations}`, `{result_path}`, and
`{repetition}` fields.  Google Benchmark JSON, key-value result markers, and
JSON result markers are supported.

The predeclared rule selects the smallest positive `T` for which paired timing
at `T` is equivalent to both `2T` and `4T`: the median ratio must be within 1%
of one and its bootstrap 95% interval must fit within a 2% equivalence band.
If no candidate qualifies, the artifact returns `extend_candidates` and names
the next duration to test.  Calibration output uses the same atomic progress,
runner provenance, host placement, and Williams-order metadata as formal runs.

## Runner support boundaries

`benchmark_support/` owns reusable execution mechanics only:

- `host.py` applies and verifies affinity/NUMA placement and records topology,
  cgroup, CPU-pressure, and CPU-frequency provenance;
- `harness.py` composes the common CLI and records the host without executing a
  proxy workload;
- `warmup.py` owns exact-workload warmup, calibration scheduling, and the
  predeclared plateau decision;
- `schedule.py` owns Williams ordering and complete-cycle validation;
- `artifact.py` atomically checkpoints JSON and records a uniform
  `artifact_state` with running/complete/failed state, active unit, progress,
  and failure details;
- `google_benchmark.py` lists and runs one JSON-producing Google Benchmark case;
- `mkl.py` configures and verifies the Linux oneMKL + oneTBB stack;
- `process.py` handles subprocess diagnostics, result markers, and executable
  discovery;
- `python_worker.py` runs one fresh Python worker per measured sample; and
- `statistics.py` and `validation.py` contain deterministic summary and CLI
  primitives.

Every runner checkpoints after a completed unit and writes a failed artifact
when a worker, profiler, parser, or correctness validation raises. The
shared artifact layer does not define case semantics or summary schemas.

Individual runners continue to own their case patterns, policy definitions,
correctness checks, summary schema, and decision rules.  Do not introduce a
runner base class: experiments deliberately differ at those semantic seams.

For C++ probes, `benchmark_argument_parser.h` contains the common option
parsing primitives, `parallelism_benchmark_helpers.h` owns shared atomic
telemetry, and `pgo_add_mkl_tbb_probe` is the common CMake target fixture.
