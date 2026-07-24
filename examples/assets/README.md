# Generated simulation assets

The OBJ surfaces in `obj/` are source assets. Volume meshes that can be
recreated from those surfaces are generated locally and ignored by Git.
`veg/tet/torus.veg` remains tracked because this directory does not contain a
corresponding source OBJ.

Generate the default fine asset set at the paths expected by the example
configs and demos:

```bash
python examples/assets/generate_veg_assets.py
```

This writes:

- cubic meshes at resolution 12;
- fTetWild meshes with relative edge length `0.05`; and
- an additional static `dragon_big.veg` with relative edge length `0.025`.

Outputs are written below `examples/assets/veg/{cubic,tet}/`, with
`examples/assets/veg/manifest.json` recording source hashes, meshing
parameters, material defaults, and actual mesh sizes.

Generate selected assets or a separate higher-resolution experiment level:

```bash
python examples/assets/generate_veg_assets.py \
  --assets bunny dragon \
  --cubic-resolution 16 \
  --tet-target-elements 20000 \
  --static-dragon-target-elements 100000 \
  --output-root examples/assets/veg/generated/high
```

The cubic mesher uses conservative occupancy by default so the source surface
can be embedded in the volume mesh. fTetWild controls resolution by relative
edge length, not by an exact element count. Use `--tet-backend tetgen` and
`--tet-target-elements` when a conforming TetGen mesh is preferred; the target
remains approximate because TetGen is controlled by maximum element volume.
Use `--max-parallelism N` to limit meshing through
`pypgo.parallel.GlobalTbbControl` without thread-count environment variables.

Before a full simulation, override the scene's volume path and run zero or a
small number of steps to validate surface embedding and solver behavior.

CI and lightweight test environments only need the box assets:

```bash
python examples/assets/generate_veg_assets.py \
  --assets box \
  --cubic-resolution 4 \
  --tet-backend tetgen \
  --tet-target-elements 100 \
  --static-dragon-target-elements 0
```
