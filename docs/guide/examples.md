# Examples

libpgo includes several example simulations under the `examples/` directory.

## Basic API Test

```bash
uv run python src/api/python/pypgo/pgo_test_01.py
```

Runs basic pypgo API calls including mesh loading and stiffness matrix computation.

## Simulation Runner

Run a simulation from a JSON config:

```bash
uv run python src/api/python/pypgo/pgo_run_sim.py examples/box/box.json
```

Available examples:

| Example | Integrator | Description |
|---|---|---|
| `examples/box/` | Newmark | Falling box |
| `examples/box-with-sphere/` | Newmark | Box with sphere contact |
| `examples/dragon/` | Backward Euler | Static dragon deformation |
| `examples/dragon-dyn/` | Backward Euler | Dynamic dragon simulation |
| `examples/bunny/` | Backward Euler | Bunny simulation |

### Deterministic Mode

For reproducible run-to-run results:

```bash
uv run python src/api/python/pypgo/pgo_run_sim.py examples/box/box.json --deterministic
```

Or set `"deterministic": true` in the simulation JSON config.

## Alembic Export

Export simulation output to `.abc` files for Blender/Maya:

```bash
uv run python src/api/python/pypgo/pgo_dump_abc.py examples/box/anim.json examples/box
```

## Cubic Mesh Generation

Generate a $4 \times 4 \times 4$ cubic/hexahedral volumetric mesh:

```bash
cmake --build build --target cubicMesher
build/bin/cubicMesher uniform \
  --resolution 4 \
  --output-mesh examples/cubic-box/cubic-box.veg \
  --output-surface examples/cubic-box/cubic-box.obj \
  --E 1e6 --nu 0.45 --density 1000
```
