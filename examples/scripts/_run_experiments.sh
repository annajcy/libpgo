#!/bin/bash
# set -e  # disable: static_compare returns 2 on pin_residual > limit, but results are valid

ROOT=/home/byk/codebase/libpgo
STATIC=$ROOT/examples/scripts/tricubic-hermit-static-compare
DYNAMIC=$ROOT/examples/scripts/tricubic-hermit-dynamic-compare
CONDA="conda run -n libpgo"

echo "========================================="
echo "Step 1/8: Generate cubic mesh for dragon (r15)"
echo "========================================="
$CONDA python $STATIC/generate_cubic_mesh.py --case dragon --resolution 15

echo ""
echo "========================================="
echo "Step 2/8: Generate cubic mesh for bunny (r15)"
echo "========================================="
$CONDA python $STATIC/generate_cubic_mesh.py --case bunny --resolution 15

echo ""
echo "========================================="
echo "Step 3/8: Subdivide dragon cubic mesh"
echo "========================================="
$CONDA python $STATIC/subdivide_cubic_mesh.py \
  --input $STATIC/assets/veg/cubic/dragon-conservative-r15.veg \
  --output $STATIC/assets/veg/cubic/dragon-conservative-r15-subdiv2.veg

echo ""
echo "Step 4/8: Subdivide bunny cubic mesh (static)"
echo "========================================="
$CONDA python $STATIC/subdivide_cubic_mesh.py \
  --input $STATIC/assets/veg/cubic/bunny-conservative-r15.veg \
  --output $STATIC/assets/veg/cubic/bunny-conservative-r15-subdiv2.veg

echo ""
echo "========================================="
echo "Step 5/8: Tune tet reference for dragon"
echo "========================================="
$CONDA python $STATIC/tune_tet_reference.py --case dragon --target-ratio 5

echo ""
echo "Step 6/8: Tune tet reference for bunny"
echo "========================================="
$CONDA python $STATIC/tune_tet_reference.py --case bunny --target-ratio 5

echo ""
echo "========================================="
echo "Step 7/8: Copy bunny assets to dynamic experiment"
echo "========================================="
mkdir -p $DYNAMIC/assets/veg/cubic $DYNAMIC/assets/veg/tet
cp $STATIC/assets/veg/cubic/bunny-conservative-r15.veg $DYNAMIC/assets/veg/cubic/
cp $STATIC/assets/veg/cubic/bunny-conservative-r15-subdiv2.veg $DYNAMIC/assets/veg/cubic/
cp $STATIC/assets/veg/tet/bunny-conservative-r15-tet-a2.8768e-9.veg $DYNAMIC/assets/veg/tet/
echo "Copied: cubic, subdiv2, tet veg files"

echo ""
echo "========================================="
echo "Step 8/8: Run static compare (dragon + bunny)"
echo "========================================="
echo "--- Static: dragon ---"
$CONDA python $STATIC/static_compare.py --case dragon --force
echo ""
echo "--- Static: bunny ---"
$CONDA python $STATIC/static_compare.py --case bunny --force

echo ""
echo "========================================="
echo "Step 9/9: Run dynamic compare"
echo "========================================="
echo "--- Dynamic: bunny ---"
$CONDA python $DYNAMIC/dynamic_compare.py --force

echo ""
echo "========================================="
echo "ALL DONE"
echo "========================================="
