# Eigen/oneMKL task profiling probe

This diagnostic executable isolates one Eigen-to-oneMKL DGEMM path so that
Intel VTune can compare the oneMKL TBB threading layer under `Default`,
`Local1`, `Local2`, and `Local4` service-API states. Each mode must run in a
fresh process; the runner enforces this by launching one VTune collection per
mode.

Build the probe on Linux with the oneMKL TBB threading layer enabled, then run:

```bash
python benchmarks/eigen_mkl_task_profile/run_eigen_mkl_task_profile.py \
  build/base/benchmarks/eigen_mkl_task_profile/eigen_mkl_task_profile_probe \
  --out benchmark-results/eigen-mkl-task-profile
```

If Linux `ptrace_scope` blocks user-mode collection, use `--sudo`. The runner
uses elevated privileges only for the VTune collector and restores ownership of
each result directory before producing reports:

```bash
python benchmarks/eigen_mkl_task_profile/run_eigen_mkl_task_profile.py \
  build/base/benchmarks/eigen_mkl_task_profile/eigen_mkl_task_profile_probe \
  --out benchmark-results/eigen-mkl-task-profile \
  --sudo
```

The default workload uses an aligned TBB arena with concurrency 8, three warm-up
DGEMMs, and 50 profiled 1024-by-1024 DGEMMs. Use VTune's Threading timeline to
compare task activity, active workers, scheduler overhead, process CPU time, and
wall time between modes.

The runner exports a VTune summary and hotspots CSV for every mode in addition
to preserving the complete VTune result directories.

VTune may not expose stable symbols or exact task identities for proprietary
oneMKL dense kernels. Treat the result as evidence about TBB scheduling activity,
not necessarily as an exact count of tasks created by oneMKL. An instrumented
oneTBB runtime is required if exact executed-task-node counts are needed.
