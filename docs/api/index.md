# API Reference

## Python API (`pypgo`)

The `pypgo` package provides Python bindings for libpgo via pybind11.

### Core Functions

- `pypgo.run_sim_from_config(config_path, deterministic=False)` — run a simulation from a JSON config file
- Mesh loading and manipulation
- Stiffness matrix computation
- Animation I/O (when built with `PGO_FEATURE_ANIMATION_IO`)

### Usage

```python
import pypgo

# Check version
print(pypgo.__version__)

# Run a simulation
pypgo.run_sim_from_config("examples/box/box.json", deterministic=True)
```

## C++ API

The C++ API is organized into the core library modules:

- **energy** — energy formulations for FEM simulation
- **scene** — scene graph, mesh I/O, materials
- **solve** — solvers and time integrators
- **utils** — math and utility functions

See [Core Modules](../architecture/core-modules.md) for module descriptions.

!!! note
    Full C++ API reference documentation (e.g., via Doxygen) is planned but not yet generated. Refer to the header files under `src/core/` for current API details.
