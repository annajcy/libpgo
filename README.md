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

Note: `--prune` removes conda packages no longer listed, but it does **not**
migrate a package that moved between the conda list and the `pip:` subsection
(conda leaves the old build in place and pip then sees it as already
satisfied). If a dependency was switched from conda to pip (e.g. `pytorch` →
pip `torch`, `pyvista`, `pytest`, `notebook`), recreate the environment from
scratch using the block below so it resolves from the intended source.

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

The Python package is installed in editable mode with pip inside the active
conda environment:

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

For conda package builds, select the conda-oriented CMake presets explicitly:

```bash
PYPGO_CMAKE_PRESET=pypgo-conda python -m pip install . --no-build-isolation --no-deps -v
```

The matching CI/local dependency file is `.github/conda/pypgo-conda.yml`.

Use `pypgo-conda-mkl` for an MKL-enabled package on platforms where MKL is
available:

```bash
PYPGO_CMAKE_PRESET=pypgo-conda-mkl python -m pip install . --no-build-isolation --no-deps -v
```

The MKL dependency file is `.github/conda/pypgo-conda-mkl.yml`.

PyPI wheels are not the primary distribution target. The release path is conda
packaging so large native runtime dependencies such as Gmsh, OpenVDB, Boost,
TBB, Imath, and MKL can be expressed as conda package dependencies instead of
being bundled into Python wheels.

### Conda Package Release

The release workflow is `.github/workflows/conda-release.yml`. It builds conda
packages from `conda-recipe/` on tag pushes and manual dispatches:

- `pypgo`: Linux, macOS Apple Silicon, and Windows.
- `pypgo-mkl`: Linux and Windows only. macOS does not publish an MKL variant.

Install either `pypgo` or `pypgo-mkl` in one environment, not both. The two
conda packages expose the same Python package name and are marked mutually
exclusive in the recipe.

The same recipe is parameterized by CI environment variables:

| Package | CMake preset | MKL |
| --- | --- | --- |
| `pypgo` | `pypgo-conda` | Off |
| `pypgo-mkl` | `pypgo-conda-mkl` | On |

To test the conda recipe locally:

```bash
conda activate libpgo
conda install -y conda-build anaconda-client
PYPGO_CONDA_PACKAGE=pypgo \
PYPGO_CMAKE_PRESET=pypgo-conda \
PYPGO_WITH_MKL=0 \
conda build conda-recipe --output-folder conda-bld --no-anaconda-upload
```

For the MKL package on Linux or Windows:

```bash
PYPGO_CONDA_PACKAGE=pypgo-mkl \
PYPGO_CMAKE_PRESET=pypgo-conda-mkl \
PYPGO_WITH_MKL=1 \
conda build conda-recipe --output-folder conda-bld --no-anaconda-upload
```

Every release build uploads the `.conda` packages as GitHub Actions artifacts.
To publish them to Anaconda.org, configure these repository settings:

- Secret `ANACONDA_API_TOKEN`: an Anaconda.org API token with upload access.
- Variable `ANACONDA_USER`: the Anaconda.org account or organization name.

Pushing a tag such as `v0.0.4` builds and uploads to the `main` label. Manual
workflow runs can build artifacts without upload, or upload to a selected label
such as `dev`.

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
| `pypgo-conda` | `build/pypgo-conda` | Conda package build for Python bindings, with portable CPU flags and MKL disabled. |
| `pypgo-conda-mkl` | `build/pypgo-conda-mkl` | Conda package build for Python bindings with MKL enabled. |
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

- Conda supplies CMake, Ninja, and the native runtime/build packages plus the
  Python packages that are ABI-tied to them: Boost, MKL, TBB, Gmsh, OpenVDB,
  Imath, zlib, numpy, setuptools, and wheel. `numpy` stays on conda because
  OpenVDB's Python bindings hard-depend on conda-numpy — making it pip-only
  would leave two conflicting numpy installs on the same import path.
- Pip (declared in the `pip:` subsection of `environment.yml`) supplies the
  pure-Python / pip-first packages that are not build-time native deps: `torch`
  (official macOS arm64 wheel, with MPS; replaces conda `pytorch`), `pyvista`
  (pulls its own `vtk` wheel — conda `vtk-base` is intentionally not installed,
  to avoid a duplicate `vtkmodules` import path), `pytest`, `notebook`, and the
  `trame` / `trame-vtk` / `trame-vuetify` stack (conda-forge lags their
  releases). `conda env create` installs these automatically after the conda
  solve, including in CI.
- The host package manager supplies platform basics that are awkward to keep
  fully inside conda: Linux compiler/system BLAS/GMP/MPFR headers and macOS
  Homebrew GMP/MPFR/Imath.
- FetchContent-managed C++ dependencies are downloaded and built by this
  repository: Eigen, fmt, spdlog, nlohmann_json, SuiteSparse, Ceres, CGAL,
  geogram, libigl, Alembic, nanobind, and related internal dependencies.

---

## Usage & Test

The repository is Python-first for runnable workflows. C++ command-line tools
have been removed; C++ remains the numerical kernel and Python bindings expose
the user-facing API.

Build the Python extension and core tests:

```bash
cmake --preset base -DPGO_ENABLE_PYTHON=ON
cmake --build --preset base --target pypgo_core
python -m pytest tests/pypgo
```

Common Python entry points:

- `pypgo.tools.sim`: high-level simulation builders and dynamic runners
- `pypgo.fem`: formulation-aware FEM energy, mass, body force, and embedding helpers
- `pypgo.contact`: IPC, floor, and sampled-penalty contact energies
- `pypgo.sim`: dynamic stepping
- `pypgo.tools.mesh`: mesh quality, cubic meshing, tet meshing, and remeshing wrappers
- `pypgo.animation`: animation loading and Alembic/VDB export
- `pypgo.tools.stress`: stress-field statistics

For runnable simulation scenes (tet/cubic/shell, static/dynamic, IPC and
penalty contact), see [`examples/sim_configs/README.md`](./examples/sim_configs/README.md)
and the `pypgo-sim-*` CLI family.

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
