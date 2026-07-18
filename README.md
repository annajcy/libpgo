# libpgo

[![Linux CI](https://github.com/annajcy/libpgo/actions/workflows/linux-ci.yml/badge.svg?branch=main)](https://github.com/annajcy/libpgo/actions/workflows/linux-ci.yml)
[![Windows CI](https://github.com/annajcy/libpgo/actions/workflows/windows-ci.yml/badge.svg?branch=main)](https://github.com/annajcy/libpgo/actions/workflows/windows-ci.yml)
[![macOS CI](https://github.com/annajcy/libpgo/actions/workflows/macos-ci.yml/badge.svg?branch=main)](https://github.com/annajcy/libpgo/actions/workflows/macos-ci.yml)
[![Documentation](https://github.com/annajcy/libpgo-doc/actions/workflows/pages.yml/badge.svg?branch=main)](https://github.com/annajcy/libpgo-doc/actions/workflows/pages.yml)

libpgo is a Python-first C++ library for physically based simulation,
geometric shape modeling, and optimization. Its `pypgo` package is the
recommended interface for most workflows: Python composes models and
experiments, while the C++ core owns performance-sensitive geometry, numerical
algorithms, and simulation state.

libpgo extends [VegaFEM](https://viterbi-web.usc.edu/~jbarbic/) for academic
research and modern Python workflows.

## What You Can Build

- Surface and volumetric mesh workflows: I/O, processing, quality checks, and meshing.
- Finite-element deformation models with elastic and plastic material fields.
- Static equilibrium and implicit dynamic simulation assembled from energies, constraints, contact, mass, and time-stepping policies.
- Nonlinear optimization with composable objectives, sparse solvers, line searches, damping, and diagnostics.
- Geometry-driven material optimization, implicit surfaces, animation I/O, and optional geometry or acceleration backends.

## Get Started

Most users begin with `pypgo`. Choose a prebuilt wheel when you only need the
Python package; choose a source build when you want to edit C++ code or Python
bindings, run native tests, or profile the library.

- [Choose a build path](docs/guide/build/index.md)
- [Use a prebuilt pypgo wheel](docs/guide/build/build-from-wheel.md)
- [Build from source](docs/guide/build/build-from-source.md)

After installing `pypgo`, this small optimization problem shows the basic workflow:

```python
import numpy as np
import pypgo as pgo
import pypgo.solver as solver

target = np.array([1.0, -2.0, 4.0], dtype=np.float64)
energy = pgo.energy.QuadraticEnergy(np.eye(3), b=-target)
problem = solver.OptimizationProblem(objective=energy)

optimizer = solver.NewtonOptimizer(
    damping=solver.NoDamping(),
    line_search=solver.Backtrack(),
)
result = optimizer.solve(problem, np.zeros(3, dtype=np.float64))

assert result.converged
assert np.allclose(result.x, target)
```

For finite-element and contact workflows, start from the runnable notebooks in
[`examples/`](examples/) rather than adapting this deliberately small solver
example.

## Documentation

You can view the documentation at [getting started](https://annajcy.github.io/libpgo-doc). Visit the [guide](docs/guide/index.md) for setup, development, and repository navigation.

## License

libpgo includes code derived from VegaFEM and integrates third-party libraries
with their own licenses. See [LICENSE](LICENSE) and the notices in individual
source files for details.
