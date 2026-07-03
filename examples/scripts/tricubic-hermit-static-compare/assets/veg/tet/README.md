# Tet Reference Assets

Tet reference `.veg` files are generated locally and ignored by Git because
they are large numerical baselines.

Generate the cubic assets first; this script reads
`assets/veg/cubic/*-conservative-r15.veg` and
`assets/obj/*-conservative-r15-surface.obj`.

Generate the dragon r15 reference used by `static_compare.py --case dragon`:

```bash
conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/tune_tet_reference.py \
  --case dragon \
  --target-ratio 5
```

Generate the bunny r15 reference used by `static_compare.py --case bunny`:

```bash
conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/tune_tet_reference.py \
  --case bunny \
  --target-ratio 5
```

The checked comparison scripts currently reference the tuned filenames:

- `dragon-conservative-r15-tet-a1.47683e-7.veg`
- `bunny-conservative-r15-tet-a2.89036e-9.veg`

To regenerate those exact files without re-running the bracket search:

```bash
conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/tune_tet_reference.py \
  --case dragon \
  --a 1.47683e-7
conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/tune_tet_reference.py \
  --case bunny \
  --a 2.89036e-9
```
