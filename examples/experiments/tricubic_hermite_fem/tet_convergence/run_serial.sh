#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/../../../.." && pwd)
CONDA_BIN=${CONDA_BIN:-conda}
CONDA_ENV=${CONDA_ENV:-libpgo}
NUM_THREADS=${NUM_THREADS:-21}

export MKL_THREADING_LAYER=${MKL_THREADING_LAYER:-TBB}
cd "$REPO_ROOT"

if [[ -n "${RUN_LOG:-}" ]]; then
  mkdir -p "$(dirname -- "$RUN_LOG")"
  exec > >(tee -a "$RUN_LOG") 2>&1
fi

printf 'tet convergence serial run started=%s commit=%s threads=%s\n' \
  "$(date -Is)" "$(git rev-parse HEAD)" "$NUM_THREADS"

RUN=("$CONDA_BIN" run --no-capture-output -n "$CONDA_ENV" python -u)

for study in bunny dragon; do
  "${RUN[@]}" \
    examples/experiments/tricubic_hermite_fem/tet_convergence/run_static.py \
    --study "$study" \
    --num-threads "$NUM_THREADS" \
    --force
  "${RUN[@]}" \
    examples/experiments/tricubic_hermite_fem/tet_convergence/analyze.py \
    --study "$study"
done

printf 'tet convergence serial run completed=%s\n' "$(date -Is)"
