# Benchmark execution policy

All top-level benchmark runners use the shared
`benchmarks/host_preconditioning.py` helper before starting measured cases.
The policy has three distinct layers:

1. a policy-neutral all-core host preheat (10 seconds by default);
2. policy-neutral throughput probes until the trailing three probes agree
   within 2%; and
3. each benchmark's existing process-local or kernel-local warm-up.

Measured policy order uses deterministic Latin-style rotations within each
repetition/workload block. This balances temporal positions while retaining a
reproducible seed. It replaces independent per-block shuffles, which can be
biased by monotonic CPU/package performance drift.

Common options are available on every runner:

```text
--host-preheat-seconds
--host-preheat-workers
--host-probe-seconds
--host-required-stable-probes
--host-max-probes
--host-stability-tolerance
--skip-host-preconditioning
```

Use `--skip-host-preconditioning` only for dry runs or diagnostics. The choice
is explicit in the output manifest. On Linux, the manifest also records CPU
affinity, cpufreq driver/governor/frequency bounds, load average, and CPU
pressure before and after preconditioning.

For strict server runs, apply CPU affinity to the controller process; all
preheat workers and benchmark subprocesses inherit it. Do not compile, profile,
or run unrelated jobs concurrently on the allocated CPUs. Process-local warmup
does not replace host preconditioning.
