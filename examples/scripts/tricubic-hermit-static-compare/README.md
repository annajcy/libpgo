# Tricubic Hermite Static Compare

Run the formulation comparison:

```bash
conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/static_compare.py \
  --case dragon
```

Use `--case bunny` for the bunny study. Cases can be run independently and
completed outputs are reused unless `--force` is given:

```bash
conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/static_compare.py \
  --case dragon --cases cubic_linear
```

Mesh sanity report for the same assets:

```bash
conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/mesh_quality.py \
  --case dragon --json /tmp/dragon-mesh-quality.json
```

Regenerate the conservative cubic mesh and boundary OBJ:

```bash
conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/generate_cubic_mesh.py \
  --case dragon --resolution 15
```

Then tune a TetGen reference to a target tet/Hermite DOF ratio:

```bash
conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/tune_tet_reference.py \
  --case dragon --target-ratio 5
```

Generated assets are ignored under `assets/`.

Summarize completed outputs:

```bash
conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/summarize_results.py \
  --case dragon
```

The command writes `comparison.{json,csv,md}` under the selected output root.

The legacy lightweight `dragon.veg` is the same pipeline at r8:

```bash
conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/generate_cubic_mesh.py \
  --case dragon --resolution 8 --output examples/scripts/tricubic-hermit-static-compare/assets/veg/cubic/dragon.veg
```
