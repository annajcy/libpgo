#!/bin/bash
# Run all four cases sequentially. tet_ref reuses cache, cubic cases run fresh.
cd /mnt/data02/jcy/libpgo_private/examples/scripts/tricubic-hermit-dynamic-compare

PYTHON=/mnt/data02/jcy/miniforge3-libpgo/envs/libpgo/bin/python

# tet_ref: no --force, will reuse cache if available
echo "=== [1/4] tet_ref (cache) ==="
$PYTHON dynamic_compare.py --cases tet_ref || exit 1

# cubic cases: force re-run for clean benchmark with NoDamping policy
echo "=== [2/4] cubic_hermite ==="
$PYTHON dynamic_compare.py --cases cubic_hermite --force || exit 1

echo "=== [3/4] cubic_linear ==="
$PYTHON dynamic_compare.py --cases cubic_linear --force || exit 1

echo "=== [4/4] cubic_linear_x8 ==="
$PYTHON dynamic_compare.py --cases cubic_linear_x8 --force || exit 1

echo "=== DONE: all 4 cases complete ==="
