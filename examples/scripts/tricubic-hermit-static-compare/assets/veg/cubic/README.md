# Cubic Assets

Cubic `.veg` files are generated locally and ignored by Git.

Generate the conservative r15 cubic mesh for a case:

```bash
conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/generate_cubic_mesh.py \
  --case dragon

conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/generate_cubic_mesh.py \
  --case bunny
```

Those commands also generate the matching conservative boundary OBJ files under
`assets/obj/`.

Generate the x8 subdivided cubic meshes used by `cubic_linear_x8`:

```bash
conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/subdivide_cubic_mesh.py \
  --input examples/scripts/tricubic-hermit-static-compare/assets/veg/cubic/dragon-conservative-r15.veg \
  --output examples/scripts/tricubic-hermit-static-compare/assets/veg/cubic/dragon-conservative-r15-subdiv2.veg

conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/subdivide_cubic_mesh.py \
  --input examples/scripts/tricubic-hermit-static-compare/assets/veg/cubic/bunny-conservative-r15.veg \
  --output examples/scripts/tricubic-hermit-static-compare/assets/veg/cubic/bunny-conservative-r15-subdiv2.veg
```

The static comparison expects these local filenames:

- `dragon-conservative-r15.veg`
- `dragon-conservative-r15-subdiv2.veg`
- `bunny-conservative-r15.veg`
- `bunny-conservative-r15-subdiv2.veg`
