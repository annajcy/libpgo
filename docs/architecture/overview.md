# Architecture Overview

libpgo is organized into three top-level source directories under `src/`.

```text
src/
├── core/          # Core library modules
│   ├── energy/    # Energy formulations (elasticity, contact, etc.)
│   ├── external/  # Vendored or adapted third-party code
│   ├── scene/     # Scene graph, mesh I/O, and simulation setup
│   ├── solve/     # Solvers and time integrators
│   └── utils/     # Shared utilities (math, containers, etc.)
├── api/           # Public API layer (Python bindings)
└── tools/         # Standalone command-line tools
    ├── animation/ # Animation export utilities
    ├── cubicMesher/   # Cubic/hexahedral mesh generator
    ├── remeshSurface/ # Surface remeshing tool
    ├── runSim/        # Simulation runner
    └── tetMesher/     # Tetrahedral mesh generator
```

## Build System

The build uses **CMake** with **Conan 2.x** for C++ dependencies and **scikit-build-core** for Python packaging.

- `CMakePresets.json` defines presets for core, geometry, animation, full, and Python builds
- `cmake/BootstrapConan.cmake` automatically exports local Conan recipes and runs `conan install` during configure
- Private Conan recipes live under `conan/recipes/`

## Feature Model

The build is feature-oriented. See the [C++ Library Guide](../guide/cpp-library.md) for the full option table.

Key features: Python bindings, animation I/O, geometry stack (Boost/CGAL/Ceres/NLopt/SuiteSparse), libigl, geogram, gmsh, MKL, ARPACK.
