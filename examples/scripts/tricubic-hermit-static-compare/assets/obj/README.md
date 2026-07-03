# OBJ Assets

`dragon.obj` and `bunny.obj` are source surface meshes and are kept in Git.

The conservative r15 boundary meshes are generated locally and ignored by Git:

- `dragon-conservative-r15-surface.obj`
- `bunny-conservative-r15-surface.obj`

Generate them with the matching cubic meshes:

```bash
conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/generate_cubic_mesh.py \
  --case dragon

conda run -n libpgo python examples/scripts/tricubic-hermit-static-compare/generate_cubic_mesh.py \
  --case bunny
```

Those commands write both:

- `assets/veg/cubic/<case>-conservative-r15.veg`
- `assets/obj/<case>-conservative-r15-surface.obj`
