# Benchmark execution policy

All top-level benchmark runners use the shared
`benchmarks/host_preconditioning.py` helper before starting measured cases.
The policy has three distinct layers:

1. a policy-neutral all-core host preheat (60 seconds by default, long enough
   to reach the sustained package power/thermal state on the benchmark host);
2. policy-neutral throughput probes until the trailing three probes have at
   most 2% relative median absolute deviation (and no greater than 10% total
   spread); and
3. each benchmark's existing process-local or kernel-local warm-up.

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
--host-preheat-seconds
--host-preheat-workers
--host-probe-seconds
--host-required-stable-probes
--host-max-probes
--host-stability-tolerance
--host-drift-tolerance
--host-abort-drift-tolerance
--allow-host-drift
--skip-host-preconditioning
```

`--allow-incomplete-order-cycle` is an explicit diagnostic escape hatch. Such
a run records `strictly_balanced = false` and must not be mixed into strict
timing comparisons.

Use `--skip-host-preconditioning` only for dry runs or diagnostics. The choice
is explicit in the output manifest. On Linux, the manifest also records
requested and effective CPU affinity, CPU/core/socket/NUMA topology, cpufreq
driver/governor/frequency bounds, load average, and CPU pressure before and
after preconditioning.

For strict server runs, pass affinity as a benchmark parameter, for example
`--cpu-list 21-28 --numa-node 0`. The controller applies it before
preconditioning, verifies that its size equals benchmark concurrency, and all
preheat workers and benchmark subprocesses inherit it. An affinity inherited
from `taskset` is also accepted when its size already matches concurrency.
Cross-NUMA placement is rejected unless `--allow-cross-numa` is explicit. Do
not compile, profile, or run unrelated jobs concurrently on the allocated
CPUs. Process-local warmup does not replace host preconditioning.

Every measured case is preceded by a policy-neutral guard probe. Probes within
5% of the initial stable baseline are marked `stable`; larger deviations abort
the run by default, before the affected case is measured. This both rejects
host drift and gives every policy the same immediate predecessor instead of
allowing a long-running policy to thermally condition the next one.

`--allow-host-drift` explicitly permits deviations between the drift and abort
thresholds and records them as `drifted`. Results produced with that override
are diagnostic rather than strict policy comparisons. A deviation above the
abort threshold always stops the run. The complete guard trajectory remains in
the result artifact for audit, and `strict_timing_comparability` is set to
`false` whenever a guard is skipped or accepted as drifted.

Before a formal server campaign, run `run_harness_placebo.py` against one
representative oneMKL Google Benchmark case. Its `placebo_a` and `placebo_b`
labels execute the same case, command parameters, environment, and fresh-process
path in a complete AB/BA cycle. The gate rejects material label bias, temporal
position bias, or guard-throughput bias before any real policy contrast is
interpreted. The default fixed acceptance region requires every median ratio
to remain within 2% of one and every bootstrap 95% interval to fit inside a 5%
equivalence band, in addition to strict affinity, stable guards, and identical
correctness counters.

For example, on the eight-core Linux allocation:

```bash
python3 benchmarks/run_harness_placebo.py \
  build/base/benchmarks/eigen_mkl_crossover/eigen_mkl_crossover_benchmark \
  --case EigenMklCrossover/ExecutorLocal1/n_128 \
  --out benchmark-results/harness-placebo.json \
  --concurrency 8 --cpu-list 21-28 --numa-node 0
```

## Runner support boundaries

`benchmark_support/` owns reusable execution mechanics only:

- `google_benchmark.py` lists and runs one JSON-producing Google Benchmark case;
- `mkl.py` configures and verifies the Linux oneMKL + oneTBB stack;
- `process.py` handles subprocess diagnostics, result markers, and executable
  discovery;
- `python_worker.py` runs one fresh Python worker per measured sample; and
- `statistics.py` and `validation.py` contain deterministic summary and CLI
  primitives.

Individual runners continue to own their case patterns, policy definitions,
correctness checks, summary schema, and decision rules.  Do not introduce a
runner base class: experiments deliberately differ at those semantic seams.

For C++ probes, `benchmark_argument_parser.h` contains the common option
parsing primitives, `parallelism_benchmark_helpers.h` owns shared atomic
telemetry, and `pgo_add_mkl_tbb_probe` is the common CMake target fixture.
