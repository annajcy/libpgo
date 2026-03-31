# Core Modules

The `src/core/` directory contains the main library functionality.

## energy

Energy formulations for finite element simulation, including elasticity models and contact energies.

Source: `src/core/energy/`

## scene

Scene management, mesh I/O, material definitions, and simulation configuration.

Source: `src/core/scene/`

## solve

Numerical solvers and time integration schemes:

- Implicit Backward Euler (BE)
- Implicit Newmark (NW)
- TR-BDF2

Source: `src/core/solve/`

## utils

Shared utilities including math helpers, sparse matrix operations, and container types.

Source: `src/core/utils/`

## external

Vendored or adapted third-party code used by the core library.

Source: `src/core/external/`
