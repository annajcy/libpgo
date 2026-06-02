## libpgo: Library for Physically based Simulation (P), Geometric Shape Modeling (G), and Optimization (O)

The library is designed to primarily focus on physically based simulations, geometric shape modeling, and optimization.
The source code extends [VegaFEM](https://viterbi-web.usc.edu/~jbarbic/vega/) and is designed for academic research purposes.

---

## Build With Conda

Conda is the recommended build environment for both the Python package and the
native CMake build. Use one conda environment for Python packages and native
runtime/build packages so CMake, Python, Boost, MKL, TBB, Gmsh, OpenVDB, and
other dependencies are resolved from a consistent prefix.

- The Python package is being redesigned as a Python-first API. The old
  C-style Python wrapper has been removed and its functionality will return
  through focused Python modules.
- The native CMake build uses the `base` preset, which enables the full default
  feature set including Gmsh, OpenVDB, TBB, and MKL where supported.
- Platform compilers still come from the host system: GCC/Clang on Linux,
  Apple Clang on macOS, and Visual Studio 2022 on Windows.

Use Miniforge or Miniconda when possible, and keep packages on the
`conda-forge` channel. The commands below use `conda` consistently so the
environment, Python packages, and native CMake dependencies all resolve from
one conda prefix.

### System Prerequisites

Install conda: See [Conda Installation](https://www.anaconda.com/docs/getting-started/miniconda/install/overview#choose-your-installation-guide).

A C++ compiler is the only system-level requirement — all library dependencies
are managed by the `environment.yml` file in the repository root.

Linux (GCC via apt):

```bash
sudo apt-get install -y build-essential git
```

macOS (Apple Clang via Xcode):

```bash
xcode-select --install
```

Windows:

- Install Visual Studio 2022 with the C++ desktop workload.
- Run builds from an x64 MSVC developer shell.

Blender and ffmpeg are optional. They are only needed by
`scripts/render_abc_preview.py` for Alembic GIF previews.

### Create the conda environment

All packages — build tools, C++ libraries, and Python dependencies — are
declared in `environment.yml`. Create the `libpgo` environment with a single
command:

```bash
conda env create -f environment.yml
conda activate libpgo
```

To update an existing environment after pulling changes:

```bash
conda env update -f environment.yml --prune
```

To recreate the environment from scratch:

```bash
conda deactivate
conda env remove -n libpgo -y
conda env create -f environment.yml
conda activate libpgo
```

**MKL (Linux / Windows only):** MKL is commented out in `environment.yml`
because it is unavailable on Apple Silicon. Linux and Windows users who want
the `base` preset to pick up MKL can install it after environment creation:

```bash
conda install -n libpgo -y mkl-devel
```

`mamba` can be used as an optional accelerator only when it belongs to the same
conda installation that owns the `libpgo` environment. Avoid mixing a
Homebrew/micromamba `mamba` with a Miniconda environment, because that can
create another `libpgo` under a different prefix.

### Python Package Build

The Python package is installed in editable mode with pip:

```bash
conda activate libpgo
python -m pip install -e . --no-build-isolation
```

For Python API development, rebuild the native `_core` extension in place after
changing C++ bindings or native mesh code:

```bash
python setup.py build_ext --inplace
```

This command uses the `pypgo` CMake preset and writes the extension back
into `pypgo/`, where the editable package imports it. By default it uses the
detected CPU count for the native build; pass `-j N` if you want to override
the number of parallel build jobs.

To build a wheel:

```bash
python setup.py bdist_wheel
pip install dist/pypgo-*.whl
```

### Native CMake Build

Native builds are CMake-preset driven. The default native build uses the `base`
preset:

```bash
conda activate libpgo
cmake --preset base
cmake --build --preset base
ctest --test-dir build/base --output-on-failure
```

On Windows, add `-G Ninja` to the configure step:

```powershell
cmake --preset base -G Ninja
cmake --build --preset base
ctest --test-dir build/base --output-on-failure
```

The `base` preset enables MKL, Alembic, Gmsh, TetWild, OpenVDB, the Python
binding, and the C API. On macOS, CMake automatically forces `PGO_USE_MKL=OFF`
and `PGO_ENABLE_CUDA=OFF`.

Other shared presets are available for debug, CUDA, Knitro, and Pardiso builds:

| Configure preset | Binary directory | Purpose |
| --- | --- | --- |
| `base` | `build/base` | Default release build. |
| `pypgo` | `build/pypgo` | Lightweight preset for Python-first native bindings. |
| `base_debug` | `build/base_debug` | Debug build. |
| `base_cuda` | `build/base_cuda` | `base` plus CUDA. |
| `base_cuda_debug` | `build/base_cuda_debug` | Debug CUDA build. |
| `base_knitro` | `build/base_knitro` | `base` plus Knitro. |
| `base_knitro_cuda` | `build/base_knitro_cuda` | Knitro plus CUDA. |
| `all` | `build/all` | `base` plus Knitro, Pardiso, and CUDA. |
| `all_debug` | `build/all_debug` | Debug version of `all`. |

Machine-specific SDK paths belong in untracked `CMakeUserPresets.json`, not in
the shared presets. Use it for local `KNITRO_LIBRARY_HINT`,
`PARDISO_LIBRARY_HINT`, `cudss_DIR`, or similar paths.

Example `CMakeUserPresets.json` (local, optional):

<details>
<summary>Click to expand example</summary>

```json
{
    "version": 3,
    "configurePresets": [
        {
            "name": "local-base",
            "displayName": "Local base",
            "description": "Local IDE profile inheriting the shared base preset.",
            "inherits": "base",
            "environment": {
                "CONDA_PREFIX": "/Users/jinceyang/miniconda3/envs/libpgo"
            }
        },
        {
            "name": "local-base-debug",
            "displayName": "Local base debug",
            "description": "Local IDE profile inheriting the shared base_debug preset.",
            "inherits": "base_debug",
            "environment": {
                "CONDA_PREFIX": "/Users/jinceyang/miniconda3/envs/libpgo"
            }
        },
        {
            "name": "local-all",
            "displayName": "Local all",
            "description": "Local IDE profile inheriting all with local Knitro/Pardiso hints.",
            "inherits": "all",
            "environment": {
                "CONDA_PREFIX": "/Users/jinceyang/miniconda3/envs/libpgo"
            },
            "cacheVariables": {
                "KNITRO_LIBRARY_HINT": "/opt/artelys/knitro-15.0.1-Linux64",
                "PARDISO_LIBRARY_HINT": "/opt/panua-pardiso-20240229-linux"
            }
        },
        {
            "name": "local-base-cuda",
            "displayName": "Local base CUDA",
            "description": "Local IDE profile inheriting base_cuda with local cuDSS hint.",
            "inherits": "base_cuda",
            "environment": {
                "CONDA_PREFIX": "/Users/jinceyang/miniconda3/envs/libpgo"
            },
            "cacheVariables": {
                "cudss_DIR": "C:/Program Files/NVIDIA cuDSS/v0.7/lib/13/cmake/cudss"
            }
        },
        {
            "name": "local-pypgo",
            "displayName": "Local pypgo",
            "description": "Local IDE profile inheriting pypgo with conda env.",
            "inherits": "pypgo",
            "environment": {
                "CONDA_PREFIX": "/Users/jinceyang/miniconda3/envs/libpgo"
            }
        }
    ],
    "buildPresets": [
        {
            "name": "local-pypgo",
            "displayName": "Local pypgo",
            "configurePreset": "local-pypgo",
            "targets": ["pypgo_core"],
            "jobs": 32
        },
        {
            "name": "local-base",
            "configurePreset": "local-base",
            "jobs": 32
        },
        {
            "name": "local-base-debug",
            "configurePreset": "local-base-debug",
            "jobs": 32
        },
        {
            "name": "local-all",
            "configurePreset": "local-all",
            "jobs": 32
        },
        {
            "name": "local-base-cuda",
            "configurePreset": "local-base-cuda",
            "jobs": 32
        }
    ]
}
```

</details>

### Dependency Ownership

- Conda supplies CMake, Ninja, Python packages, and most native runtime/build
  packages: Boost, MKL, TBB, Gmsh, OpenVDB, Imath, zlib, numpy, pytest,
  setuptools, and wheel.
- The host package manager supplies platform basics that are awkward to keep
  fully inside conda: Linux compiler/system BLAS/GMP/MPFR headers and macOS
  Homebrew GMP/MPFR/Imath.
- FetchContent-managed C++ dependencies are downloaded and built by this
  repository: Eigen, fmt, spdlog, nlohmann_json, SuiteSparse, Ceres, CGAL,
  geogram, libigl, Alembic, nanobind, and related internal dependencies.

---

## Usage & Test

The primary runnable examples in this repository are now IPC examples driven by `runIPCSim` under `examples/ipc/`.

Build the IPC tools:

```bash
cmake --preset base
cmake --build --preset base --target runIPCSim convertAnimation
```

Run named IPC batches from the JSON config:

```bash
scripts/run_sim_batch.py --config examples/ipc/ipc_batch.json --job squash_regression --dry-run
scripts/run_sim_batch.py --config examples/ipc/ipc_batch.json --job squash_regression --skip-existing
scripts/run_sim_batch.py --config examples/ipc/ipc_batch.json --case cubic_box_with_sphere_lite --overwrite
scripts/run_sim_batch.py --config examples/ipc/ipc_batch.json --job all_ipc_abc
```

The generic batch runner reads [`examples/ipc/ipc_batch.json`](./examples/ipc/ipc_batch.json), runs the stages declared by each job, and defaults jobs without a `stages` field to `runIPCSim` with `--log` followed by `convertAnimation` with the matching per-case `anim.json`. Use [`examples/ipc/README.md`](./examples/ipc/README.md) for the full case list, job definitions, and output-overwrite policy.

The same runner also supports Alembic preview rendering when a case supplies
`render_config`. The render stage calls
[`scripts/render_abc_preview.py`](./scripts/render_abc_preview.py), which uses
Blender to render `.abc` frames and ffmpeg to encode a GIF:

```bash
scripts/render_abc_preview.py --config my_render_config.json --overwrite
scripts/run_sim_batch.py --config examples/ipc/ipc_batch.json --job sim --overwrite
```

Run representative IPC cases from the repo root:

```bash
build/base/bin/runIPCSim examples/ipc/shell/shell-hang/shell-ipc.json
build/base/bin/runIPCSim examples/ipc/shell/shell-drop/shell-ipc.json
build/base/bin/runIPCSim examples/ipc/tet/box-hang/box-ipc.json
build/base/bin/runIPCSim examples/ipc/cubic/box-with-sphere/box-ipc.json
```

Convert dumped frame sequences to Alembic:

```bash
build/base/bin/convertAnimation examples/ipc/shell/shell-hang/anim.json
build/base/bin/convertAnimation examples/ipc/shell/shell-drop/anim.json
build/base/bin/convertAnimation examples/ipc/tet/box-hang/anim.json
build/base/bin/convertAnimation examples/ipc/cubic/box-with-sphere/anim.json
```

For the full IPC case list and per-case notes, see [`examples/ipc/README.md`](./examples/ipc/README.md).

Legacy penalty-based volume contact is available through the same entrypoint:

```bash
build/base/bin/runIPCSim --legacy path/to/legacy-volume-config.json
```

`runIPCSim` accepts both `"sim-type": "dynamic"` and `"sim-type": "static"`. Static mode performs a one-shot Newton solve from the rest state, writes the same unified `states/deform0000.u` and `surface/ret0000.obj` layout as dynamic mode, and does not support `restart-from-u`. Static output is written only after Newton convergence; unconstrained gravity-only static drops, including legacy penalty-contact drops without attachments, are expected to fail instead of producing a partial state.

`--legacy` accepts the old volume JSON shape with either `tet-mesh` or `cubic-mesh` and uses the penalty contact model instead of IPC contact. Legacy static mode preserves the old volume static semantics: it solves elastic, attachment, and external-force energies without adding the legacy penalty contact energies. Shell legacy configs are no longer supported; use the IPC shell examples above for shell simulations.

Solver status is reported through the shared `SolverResult` / `SolveStatus` API used by `NewtonSolver`, `EnergyOptimizer`, and `TimeIntegratorSolver`. Static runs require `Converged` before writing output. Dynamic implicit Euler currently preserves the legacy timestep policy: `Converged`, `MaxIterations`, and `StepTooSmall` are accepted timestep statuses, while other statuses are failures. External solvers keep backend-specific return codes in `SolverResult::rawStatusCode`.

## Tools

### Cubic Mesher

`cubicMesher` converts a closed triangle surface mesh in `.obj` format into a cubic volumetric `.veg` mesh and can optionally export the extracted cubic surface as `.obj`.

Build the tool:

```bash
cmake --preset base
cmake --build --preset base --target cubicMesher
```

Basic usage:

```bash
build/base/bin/cubicMesher \
--input-mesh examples/ipc/cubic/box/box.obj \
--resolution 4 \
--output-mesh /tmp/libpgo-box.veg \
--output-surface /tmp/libpgo-box-surface.obj \
--E 10000000 \
--nu 0.45 \
--density 1000
```

Main arguments:

- `--input-mesh`: input closed triangle mesh in `.obj`
- `--resolution`: number of cubic cells along the shortest input AABB edge
- `--output-mesh`: output cubic `.veg`
- `--output-surface`: optional extracted surface `.obj`
- `--E`, `--nu`, `--density`: isotropic material parameters written into the output mesh

### Tet Mesher

`tetMesher` converts a closed triangle surface mesh into a tetrahedral `.veg` simulation mesh from a JSON job config. The JSON selects the backend, backend parameters, input/output paths, and optional generated boundary surface export. Paths inside the config are resolved relative to the config file.

Build `tetMesher` with the default `base` preset:

```bash
cmake --preset base
cmake --build --preset base --target tetMesher
```

Run a tet meshing job:

```bash
build/base/bin/tetMesher --config path/to/tetmesh.json
```

Basic TetGen config:

```json
{
  "version": 1,
  "backend": "tetgen",
  "input_mesh": "union_shell_remesh.obj",
  "output_mesh": "union_shell_tetgen.veg",
  "output_surface": "union_shell_tetgen_surface.obj",
  "print_stats": true,
  "tetgen": {
    "command": "pq1.414a0.01"
  }
}
```

The fTetWild backend is optional. Enable it when configuring, then build it in the preset build tree:

```bash
cmake --preset base -DPGO_TET_MESHER_USE_TET_WILD=ON
cmake --build --preset base --target tetMesher
```

Basic fTetWild config:

```json
{
  "version": 1,
  "backend": "tetwild",
  "input_mesh": "union_shell_remesh.obj",
  "output_mesh": "union_shell.veg",
  "output_surface": "union_shell_tet_surface.obj",
  "print_stats": true,
  "quiet": true,
  "tetwild": {
    "lr": 0.05,
    "epsr": 0.001,
    "stop_energy": 10,
    "max_threads": 8
  }
}
```

Config fields:

- `backend`: `tetgen` or `tetwild`
- `input_mesh`: input closed triangle mesh, typically `.obj`
- `output_mesh`: output tetrahedral `.veg`
- `output_surface`: optional generated tet boundary surface `.obj`
- `print_stats`, `quiet`: optional shared booleans
- `tetgen.command`: TetGen command string
- `tetwild.lr` / `tetwild.la`: relative or absolute fTetWild target edge length
- `tetwild.epsr`: fTetWild relative envelope tolerance
- `tetwild.stop_energy`, `tetwild.max_threads`: fTetWild optimization controls

## Third-party libraries

This library use the following third-party libraries:<br>
alembic, argparse, autodiff, boost, ceres, cgal, fmt, geogram, gmesh, json, knitro, libigl, mkl, spdlog, suitesparse, tbb, tinyobj-loader

---

## Licence

This library is developed using [VegaFEM](https://viterbi-web.usc.edu/~jbarbic/vega/) along with various third-party libraries, each governed by their respective licenses. Detailed copyright and license information is included within the majority of the source files.

In instances where specific licensing details are not provided within a source file, the copyright remains with the author. The licensing for those source files adhere to the principles of the pre-existing license framework. For instance, if a source file without licensing details incorporates components that fall under the GPL parts of CGAL, then that file will adhere to the GPL. All other source files default to the MIT License unless stated otherwise.

---

## TODO

- [x] Functional and compilable on three major platforms.
- [ ] Documentation
- [ ] More python interface
- [ ] Cleanup source code with non-MIT/non-FreeBSD licence.
- [ ] GUI
