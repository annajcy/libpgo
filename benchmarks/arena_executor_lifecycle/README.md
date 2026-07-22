# Arena Executor Lifecycle Benchmark

This benchmark estimates the incremental lifecycle cost of a nested
`ArenaThreadingExecutor`. The nested operation increments a checksum and does
no numerical work, so the primary contrast measures arena, observer, policy,
State allocation, construction/destruction, registry locking/scanning, and
retirement overhead rather than GEMM cost. Concurrent cases also include the
real contention created when several tasks retire executors simultaneously.

## Cases

| Case | Inner-executor lifetime | Purpose |
| --- | --- | --- |
| `reuse` | Construct one executor per outer task before warmup, reuse through measurement, retire afterward | Expected production pattern and primary baseline. |
| `recreate` | Construct, execute, and retire once per outer task in every measured batch | Measures hot-loop executor lifecycle overhead. |
| `recreate_retained_attachment` | Recreate per operation and retain an attached arena for every retired State | Positive control that forces retirement-list growth and repeated scans; it is not a recommended usage pattern. |

Both primary cases execute the same `parallel_for` batches inside one reusable
outer executor. Every batch performs `outer_tasks` nested `execute()` calls.
`reuse` constructs no inner executor in the timed region; `recreate` constructs
and retires one per nested call.

## Build and Run

```bash
cmake --build build/base --target arena_executor_lifecycle_benchmark -j

python3 benchmarks/arena_executor_lifecycle/run_arena_executor_lifecycle_benchmark.py \
  build/base/benchmarks/arena_executor_lifecycle/arena_executor_lifecycle_benchmark \
  --out benchmark-results/arena-executor-lifecycle.json \
  --concurrency 8 --outer-tasks 1 8 32 --repetitions 8 --min-time 0.25s
```

The runner launches every case in a fresh process, uses a Williams
counterbalanced order, and reports paired ratios, absolute extra nanoseconds per
inner call, bootstrap intervals, host placement, and raw retirement telemetry.

The `recreate_retained_attachment` case is an optional direct-probe positive
control. Use a fixed, small operation count because it intentionally creates a
growing retired list:

```bash
build/base/benchmarks/arena_executor_lifecycle/arena_executor_lifecycle_benchmark \
  --case=recreate_retained_attachment --concurrency=2 \
  --outer-tasks=2 --warmup-seconds=0 --warmup-min-operations=10 \
  --measurement-min-seconds=0 --measurement-min-operations=100 \
  --drain-timeout-ms=5000
```

## Interpretation Fixed in Advance

- A paired `recreate / reuse` interval above the configured equivalence band
  supports the claim that hot-loop construction and retirement have measurable
  cost.
- A ratio interval contained by that band weakens the claim on the target host.
- High run-to-run variance or an interval spanning both equivalence and a
  practically important slowdown is inconclusive and calls for more paired
  repetitions, not a post-hoc threshold.
- `retired_after_measurement` should remain zero for `reuse` and `recreate`.
  Growth proportional to `measurement_inner_calls` is expected only for the
  retained-attachment positive control.

The runner reports a `retirement_backlog_observed` annotation when either
normal case has a nonzero end-of-measurement snapshot. One snapshot can be a
small detach tail rather than proof of a growing backlog, so it does not
override the paired timing classification; investigate its scaling separately
before attributing timing to list growth.

The ratio is sensitive to the deliberately empty body. For `outer_tasks > 1`,
wall time divided by logical inner calls is a throughput-amortized quantity,
not the latency of one executor. The paired
`extra_amortized_wall_nanoseconds_per_inner_call` estimates wall impact for the
tested concurrent topology; use `outer_tasks=1` when interpreting a sequential
per-call latency. `drain_seconds` is reported separately from the timed region
and must not be added unless the caller's real lifecycle requires synchronous
reclamation at that boundary.

Warmup retirement debt is drained before measurement, so every timed process
starts with an empty registry. The primary comparison measures clean-start
steady lifecycle behavior, not an application that deliberately carries an
existing retirement backlog into the hot loop.
