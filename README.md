# libpgo

libpgo is a C++ library for physically based simulation, geometric shape
modeling, and optimization. It extends
[VegaFEM](https://viterbi-web.usc.edu/~jbarbic/vega/) for academic research.

`pypgo`, the Python package built on top of libpgo, is the recommended entry
point for most users.

Start with [Getting Started](./docs/index.md) for the documentation overview.

## How to Build

Choose the guide that matches how you want to use the project:

- [Install and use a pypgo wheel](./docs/guide/build-from-wheel.md)
- [Build pypgo and libpgo from source](./docs/guide/build-from-source.md)

Use the wheel guide when you only need the Python package. Use the source guide
when editing C++ kernels or Python bindings, running native tests, or profiling
the library.

## Examples

- [API and simulation notebooks](./examples/) cover meshes, energies, solvers,
  contact, FEM, animation, and static or dynamic simulation workflows.
- [Simulation CLI scenes](./examples/sim_configs/README.md) provide runnable
  tet, cubic, and shell configurations with static, dynamic, and IPC examples.
- [Tricubic Hermite FEM experiments](./examples/experiments/tricubic_hermite_fem/README.md)
  provide reproducible static and dynamic experiment entry points.

## License

libpgo includes code derived from VegaFEM and integrates third-party libraries
with their own licenses. See [LICENSE](./LICENSE) and the notices in individual
source files for details.
