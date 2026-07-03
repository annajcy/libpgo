# Tricubic Hermite Dynamic Compare

Bunny drop comparison with IPC contact against `assets/obj/bottom.1.obj`.

Run all formulation cases:

```bash
conda run -n libpgo python examples/scripts/tricubic-hermit-dynamic-compare/dynamic_compare.py
```

Run one case:

```bash
conda run -n libpgo python examples/scripts/tricubic-hermit-dynamic-compare/dynamic_compare.py \
  --cases cubic_hermite
```

The script writes per-case dynamic surfaces plus `comparison.{json,csv}` under:

```text
examples/outputs/bunny-dynamic-drop-compare-conservative-r15
```

Alembic output is enabled by default. Use `--no-write-abc` if the local build
does not have animation IO enabled.

Assets live under this experiment directory. The `.veg` files are local copies
of the conservative r15 bunny meshes and are ignored by Git, matching the
static comparison setup.
