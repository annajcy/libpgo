#!/usr/bin/env bash
# Run the TBB-only V2 MKL sweep and always resume an optionally paused user job.
set -euo pipefail

repo_root=${1:?usage: run_mkl_tbb_bound_sweep.sh REPO_ROOT [SUSPEND_PID]}
suspend_pid=${2:-}
status_file="${repo_root}/benchmark-sweep.status"
results_dir="${repo_root}/benchmark-results"
did_suspend=false

cleanup() {
  local status=$?
  if ${did_suspend}; then
    kill -CONT "${suspend_pid}" 2>/dev/null || true
  fi
  printf '%s\n' "${status}" > "${status_file}"
  trap - EXIT
  exit "${status}"
}
trap cleanup EXIT

cd "${repo_root}"
if [[ -n "${suspend_pid}" ]]; then
  kill -STOP "${suspend_pid}"
  did_suspend=true
fi

mkdir -p "${results_dir}"
export MKL_THREADING_LAYER="${MKL_THREADING_LAYER:-TBB}"

python benchmarks/run_mkl_tbb_bound_benchmark.py \
  build/mkl-tbb-bound/benchmarks/parallelism_mkl_nested_benchmark \
  --out "${results_dir}/mkl-tbb-bound-synthetic.json" \
  --repetitions 10 --min-time 0.10s --warmup-time 0.20 --seed 20260713

python benchmarks/run_mkl_tbb_bound_benchmark.py \
  build/mkl-tbb-bound/benchmarks/parallelism_real_fem_nested_benchmark \
  --out "${results_dir}/mkl-tbb-bound-real-fem.json" \
  --repetitions 10 --min-time 0.10s --warmup-time 0.20 --seed 20260713
