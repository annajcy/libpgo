#!/bin/bash
ROOT=/home/byk/codebase/libpgo
STATIC=$ROOT/examples/scripts/tricubic-hermit-static-compare
DYNAMIC=$ROOT/examples/scripts/tricubic-hermit-dynamic-compare
CONDA="conda run -n libpgo"

echo "========================================="
echo "Static: bunny"
echo "========================================="
$CONDA python $STATIC/static_compare.py --case bunny --force || echo "(exit code $?, continuing)"

echo ""
echo "========================================="
echo "Dynamic: bunny"
echo "========================================="
$CONDA python $DYNAMIC/dynamic_compare.py --force || echo "(exit code $?, continuing)"

echo ""
echo "=== ALL DONE ==="
