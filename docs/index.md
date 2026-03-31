# libpgo Documentation

**libpgo** is a library for **P**hysically based simulation, **G**eometric shape modeling, and **O**ptimization.

It extends [VegaFEM](https://viterbi-web.usc.edu/~jbarbic/vega/) and is designed for academic research purposes.

## Quick Links

- [Getting Started](getting-started.md) — prerequisites, build instructions, and first run
- [Architecture Overview](architecture/overview.md) — how the codebase is organized
- [C++ Library Guide](guide/cpp-library.md) — building and using the C++ library
- [Python Bindings Guide](guide/python-bindings.md) — installing and using `pypgo`
- [Examples](guide/examples.md) — simulation demos and usage walkthroughs
- [API Reference](api/index.md) — generated API documentation

## Key Features

- Implicit time integration (Backward Euler, Newmark, TR-BDF2)
- Finite element simulation with tetrahedral and cubic/hexahedral meshes
- Contact and friction handling
- Geometric shape modeling via CGAL, libigl, and geogram
- Python bindings via pybind11
- Alembic animation I/O for Blender/Maya workflows
