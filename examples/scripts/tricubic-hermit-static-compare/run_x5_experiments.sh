#!/bin/bash
set -euo pipefail

# ── Config ──────────────────────────────────────────────────────
CONDA_ENV="libpgo"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
DRAGON_OUT="$ROOT/examples/outputs/dragon-static-compare-conservative-r15-x5"
BUNNY_OUT="$ROOT/examples/outputs/bunny-static-compare-conservative-r15-ear-tip-x5"

# 6 cases x 4 threads = 24 cores, within 32 limit
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export OPENBLAS_NUM_THREADS=4

mkdir -p "$DRAGON_OUT" "$BUNNY_OUT"

log() { echo "[$(date -Iseconds)] $*"; }

# ── Helpers ─────────────────────────────────────────────────────
run_dragon() {
    local case="$1"
    local logfile="${DRAGON_OUT}/${case}.log"
    log "START dragon/${case}  →  ${logfile}"
    conda run -n "$CONDA_ENV" python -u "$SCRIPT_DIR/static_compare.py" \
        --case dragon --cases "$case" --output-root "$DRAGON_OUT" --force > "$logfile" 2>&1
    local rc=$?
    log "DONE  dragon/${case}  (exit=${rc})"
    return $rc
}

run_bunny() {
    local case="$1"
    local logfile="${BUNNY_OUT}/${case}.log"
    log "START bunny/${case}  →  ${logfile}"
    conda run -n "$CONDA_ENV" python -u "$SCRIPT_DIR/static_compare.py" \
        --case bunny --cases "$case" --output-root "$BUNNY_OUT" --force > "$logfile" 2>&1
    local rc=$?
    log "DONE  bunny/${case}  (exit=${rc})"
    return $rc
}

# ── Phase 1: tet_ref (baseline) ──────────────────────────────────
SKIP_TET=0
if [ -f "${DRAGON_OUT}/tet_ref/summary.json" ] && [ -f "${BUNNY_OUT}/tet_ref/summary.json" ]; then
    log "══════════ Phase 1: tet_ref (SKIPPED — outputs exist) ══════════"
    SKIP_TET=1
fi

if [ "$SKIP_TET" -eq 0 ]; then
    log "══════════ Phase 1: tet_ref ══════════"
    run_dragon tet_ref &
    PID_DRAGON_TET=$!
    run_bunny tet_ref &
    PID_BUNNY_TET=$!

    FAILED_TET=0
    wait $PID_DRAGON_TET || { log "FAILED dragon/tet_ref"; FAILED_TET=1; }
    wait $PID_BUNNY_TET || { log "FAILED bunny/tet_ref"; FAILED_TET=1; }
    log "══════════ Phase 1 complete ══════════"

    if [ "$FAILED_TET" -eq 1 ]; then
        log "WARNING: one or both tet_ref cases failed — proceeding with available data"
    fi
fi

# ── Phase 2: cubic cases ─────────────────────────────────────────
log "══════════ Phase 2: cubic cases ══════════"
PIDS=()
for case in cubic_linear cubic_linear_x8 cubic_hermite; do
    run_dragon "$case" &
    PIDS+=($!)
    run_bunny "$case" &
    PIDS+=($!)
done

FAILED=0
for pid in "${PIDS[@]}"; do
    wait "$pid" || FAILED=1
done
log "══════════ Phase 2 complete ══════════"

# ── Phase 3: summarize ───────────────────────────────────────────
log "══════════ Phase 3: summarize ══════════"

conda run -n "$CONDA_ENV" python "$SCRIPT_DIR/summarize_results.py" \
    --case dragon \
    --reference-root "$DRAGON_OUT" --hermite-root "$DRAGON_OUT" \
    --output-prefix "${DRAGON_OUT}/comparison" || log "FAILED dragon summarize"

conda run -n "$CONDA_ENV" python "$SCRIPT_DIR/summarize_results.py" \
    --case bunny \
    --reference-root "$BUNNY_OUT" --hermite-root "$BUNNY_OUT" \
    --output-prefix "${BUNNY_OUT}/comparison" || log "FAILED bunny summarize"

log "══════════ ALL DONE (failed=${FAILED}) ══════════"
