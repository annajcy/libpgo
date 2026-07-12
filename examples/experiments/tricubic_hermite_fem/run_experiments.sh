#!/bin/bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: run_experiments.sh

Generate the shared meshes, run all five static and dynamic cases for bunny and
dragon, and write every summary. Set CONDA_ENV to override the libpgo env.
EOF
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
  usage
  exit 0
fi

if (( $# != 0 )); then
  usage >&2
  exit 2
fi

EXPERIMENT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
CONDA_ENV=${CONDA_ENV:-libpgo}
RUN=(conda run -n "$CONDA_ENV" python)

step() {
  echo ""
  echo "========================================="
  echo "$1"
  echo "========================================="
}

step "Step 1/13: Generate dragon conservative r15 cubic mesh"
"${RUN[@]}" "$EXPERIMENT_DIR/mesh/generate_cubic_mesh.py" --study dragon --resolution 15

step "Step 2/13: Generate bunny conservative r15 cubic mesh"
"${RUN[@]}" "$EXPERIMENT_DIR/mesh/generate_cubic_mesh.py" --study bunny --resolution 15

step "Step 3/13: Generate dragon x8 cubic-linear mesh"
"${RUN[@]}" "$EXPERIMENT_DIR/mesh/subdivide_cubic_mesh.py" \
  --factor 2 \
  --input "$EXPERIMENT_DIR/assets/veg/cubic/dragon-conservative-r15.veg" \
  --output "$EXPERIMENT_DIR/assets/veg/cubic/dragon-conservative-r15-subdiv2.veg"

step "Step 4/13: Generate dragon x27 cubic-linear mesh"
"${RUN[@]}" "$EXPERIMENT_DIR/mesh/subdivide_cubic_mesh.py" \
  --factor 3 \
  --input "$EXPERIMENT_DIR/assets/veg/cubic/dragon-conservative-r15.veg" \
  --output "$EXPERIMENT_DIR/assets/veg/cubic/dragon-conservative-r15-subdiv3.veg"

step "Step 5/13: Generate bunny x8 cubic-linear mesh"
"${RUN[@]}" "$EXPERIMENT_DIR/mesh/subdivide_cubic_mesh.py" \
  --factor 2 \
  --input "$EXPERIMENT_DIR/assets/veg/cubic/bunny-conservative-r15.veg" \
  --output "$EXPERIMENT_DIR/assets/veg/cubic/bunny-conservative-r15-subdiv2.veg"

step "Step 6/13: Generate bunny x27 cubic-linear mesh"
"${RUN[@]}" "$EXPERIMENT_DIR/mesh/subdivide_cubic_mesh.py" \
  --factor 3 \
  --input "$EXPERIMENT_DIR/assets/veg/cubic/bunny-conservative-r15.veg" \
  --output "$EXPERIMENT_DIR/assets/veg/cubic/bunny-conservative-r15-subdiv3.veg"

step "Step 7/13: Tune dragon tet reference by binary search"
"${RUN[@]}" "$EXPERIMENT_DIR/mesh/tune_tet_reference.py" \
  --study dragon --target-ratio 5

step "Step 8/13: Tune bunny tet reference by binary search"
"${RUN[@]}" "$EXPERIMENT_DIR/mesh/tune_tet_reference.py" \
  --study bunny --target-ratio 5

step "Step 9/13: Run dragon static five-case comparison"
"${RUN[@]}" "$EXPERIMENT_DIR/run_static.py" --study dragon --force

step "Step 10/13: Run bunny static five-case comparison"
"${RUN[@]}" "$EXPERIMENT_DIR/run_static.py" --study bunny --force

step "Step 11/13: Run bunny dynamic five-case comparison"
"${RUN[@]}" "$EXPERIMENT_DIR/run_dynamic.py" --study bunny --force

step "Step 12/13: Run dragon dynamic five-case comparison"
"${RUN[@]}" "$EXPERIMENT_DIR/run_dynamic.py" --study dragon --force

step "Step 13/13: Summarize all studies and modes"
"${RUN[@]}" "$EXPERIMENT_DIR/summarize.py" --all

echo ""
echo "========================================="
echo "ALL TRICUBIC HERMITE FEM EXPERIMENTS DONE"
echo "========================================="
