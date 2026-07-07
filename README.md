## libpgo: Library for Physically based Simulation (P), Geometric Shape Modeling (G), and Optimization (O)

**libpgo** is designed to primarily focus on physically based simulations, geometric shape modeling, and optimization.
The source code extends [VegaFEM](https://viterbi-web.usc.edu/~jbarbic/vega/) and is designed for academic research purposes.

**pypgo** is the python binding of libpgo, which is the recommended main entry point for users.

---

## Install pypgo with Python Only

Use a CI wheel artifact when you only need the Python package and do not want
to build the C++ source. The wheels are conda-environment artifacts, not
standalone PyPI wheels. `pypgo` does not declare a pip NumPy dependency, so
NumPy and its BLAS/LAPACK runtime must come from the same conda environment as
the extension.

Linux and Windows use MKL:

```bash
conda create -n pypgo -c conda-forge python=3.12 pip numpy "libblas=*=*mkl" "liblapack=*=*mkl" mkl-devel
conda activate pypgo
python -m pip install --no-deps pypgo-*.whl
```

macOS uses Accelerate:

```bash
conda create -n pypgo -c conda-forge python=3.12 pip numpy "libblas=*=*accelerate" "liblapack=*=*accelerate"
conda activate pypgo
python -m pip install --no-deps pypgo-*.whl
```

## Build and Install pypgo from Source

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

**Build toolchain policy:** 

On Linux, the full build toolchain (compilers, CMake,
Ninja, and all C++ library dependencies) should come from the active conda
environment. Do NOT use system GCC —
mixing a system GCC with conda-forge C++ libraries (especially TBB, MKL) causes
CXXABI version mismatches (e.g. `undefined reference to __cxa_call_terminate@CXXABI_1.3.15`).
Install the Linux compilers separately after creating/updating the environment:

```bash
conda activate libpgo       # or libpgo-mkl
conda install -c conda-forge gcc gxx
```

On macOS, use the host Apple Clang

On Windows, use the host MSVC

macOS (Apple Clang via Xcode):

```bash
xcode-select --install
```

Windows:

- Install Visual Studio 2022 with the C++ desktop workload.
- Run builds from an x64 MSVC developer shell.



### VS Code CMake Tools

The VS Code CMake Tools extension does not source shell profiles and cannot
inherit `conda activate`.  Its compiler-probing step runs in a minimal
environment where only the system compiler is visible, so it will pick up the
wrong GCC on Linux even when `CONDA_PREFIX` is set in the preset.

On **Linux**, create a `.vscode/settings.json` that pins the compiler,
build tool, and proxy settings so the entire build sees the same conda prefix:

```json
{
  "cmake.cmakePath": "/path/to/env/bin/cmake",
  "cmake.configureArgs": [
    "-DCMAKE_MAKE_PROGRAM=/path/to/env/bin/ninja",
    "-DCMAKE_C_COMPILER=/path/to/env/bin/gcc",
    "-DCMAKE_CXX_COMPILER=/path/to/env/bin/g++"
  ],
  "cmake.configureEnvironment": {
    "http_proxy": "http://127.0.0.1:7890",
    "https_proxy": "http://127.0.0.1:7890"
  },
  "cmake.buildEnvironment": {
    "http_proxy": "http://127.0.0.1:7890",
    "https_proxy": "http://127.0.0.1:7890"
  }
}
```

Replace `/path/to/env` with the actual conda environment prefix.  On macOS
and Windows this is unnecessary — the host compiler and runtime are the single
ABI source for those platforms (see the toolchain policy above).

After creating the file, run **Developer: Reload Window** for the settings to
take effect.

### Create the conda environment

Build tools, C++ libraries, and the conda-side Python packages are declared in
`environment.yml`; pip-managed Python dependencies for the default `pypgo`
flavor are installed by the editable build step below. Create the `libpgo`
environment with a single command:

```bash
conda env create -f environment.yml
conda activate libpgo
```

To update an existing environment after pulling changes:

```bash
conda env update -f environment.yml --prune
```

Note: `--prune` only reconciles the packages listed in `environment.yml`;
it does not touch the editable `pypgo` install or its pip-sourced deps (`torch`,
`pyvista`, the `trame` stack, ...), which come from the build step below. After
changing those, re-run the editable install. For a clean slate, recreate the
environment from scratch using the block below.

To recreate the environment from scratch:

```bash
conda deactivate
conda env remove -n libpgo -y
conda env create -f environment.yml
conda activate libpgo
```

Install the platform BLAS stack after creating or updating the shared
environment:

```bash
# Linux / Windows
conda install -n libpgo -c conda-forge mkl-devel "libblas=*=*mkl" "liblapack=*=*mkl"

# macOS
conda install -n libpgo -c conda-forge "libblas=*=*accelerate" "liblapack=*=*accelerate"
```

This keeps NumPy on the same BLAS backend as the native extension.

`mamba` can be used as an optional accelerator only when it belongs to the same
conda installation that owns the `libpgo` environment. Avoid mixing a
Homebrew/micromamba `mamba` with a Miniconda environment, because that can
create another `libpgo` under a different prefix.

### Python Package Build

For local development, the shortest path is to build the `pypgo_core` CMake
target. It writes the native extension directly into `pypgo/` as
`pypgo/_core.*`, so imports work from the repository root:

```bash
conda activate libpgo
cmake --preset pypgo
cmake --build --preset pypgo
```

After changing C++ bindings or native mesh code, rerun:

```bash
cmake --build --preset pypgo
```

Install the optional Python packages you need after activating either
environment:

```bash
# Visualization
python -m pip install pyvista trame trame-vtk trame-vuetify

# PyTorch-based FEM layers
python -m pip install torch

# Tests and notebooks
python -m pip install pytest pytest-timeout notebook
```

Use editable install only when you want Python package metadata or console
scripts installed into the active environment. Keep NumPy and the BLAS/LAPACK
runtime on conda, and install the Python package without pip dependency
resolution:

```bash
conda activate libpgo
python -m pip install -e . --no-build-isolation --no-deps
```

For CI wheel builds, select the portable CMake presets explicitly:

```bash
PYPGO_CMAKE_PRESET=pypgo-ci python -m pip install . --no-build-isolation --no-deps -v
```

The matching CI/local dependency file is `environment.yml`.

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

The `base` preset enables Alembic, Gmsh, TetWild, OpenVDB, the Python binding,
and the C API. Linux/Windows builds use MKL; macOS builds use Accelerate and
force `PGO_ENABLE_CUDA=OFF`.

Other shared presets are available for debug, CUDA, Knitro, and Pardiso builds:

| Configure preset | Binary directory | Purpose |
| --- | --- | --- |
| `base` | `build/base` | Default release build. |
| `pypgo` | `build/pypgo` | Lightweight preset for Python-first native bindings. |
| `pypgo-ci` | `build/pypgo-ci` | CI/package build for Python bindings with portable CPU flags. |
| `base_debug` | `build/base_debug` | Debug build. |
| `base_relwithdebinfo` | `build/base_relwithdebinfo` | Release build with debug info. |
| `base_cuda` | `build/base_cuda` | `base` plus CUDA. |
| `base_cuda_debug` | `build/base_cuda_debug` | Debug CUDA build. |
| `base_cuda_relwithdebinfo` | `build/base_cuda_relwithdebinfo` | Release CUDA build with debug info. |
| `base_knitro` | `build/base_knitro` | `base` plus Knitro. |
| `base_knitro_cuda` | `build/base_knitro_cuda` | Knitro plus CUDA. |
| `all` | `build/all` | `base` plus Knitro, Pardiso, and CUDA. |
| `all_debug` | `build/all_debug` | Debug version of `all`. |
| `all_relwithdebinfo` | `build/all_relwithdebinfo` | Release version of `all` with debug info. |

Use the `*_relwithdebinfo` presets when you want optimized binaries that still
carry symbols for profiling or debugging:

```bash
cmake --preset base_relwithdebinfo
cmake --build --preset base_relwithdebinfo
```

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

```json

{
  "version": 3,
  "configurePresets": [
    {
      "name": "local-all",
      "displayName": "Server local all",
      "inherits": "all",
      "environment": {
        "CONDA_PREFIX": "/mnt/data02/jcy/miniforge3-libpgo/envs/libpgo",
        "PATH": "/mnt/data02/jcy/miniforge3-libpgo/envs/libpgo/bin:$penv{PATH}",
        "LD_LIBRARY_PATH": "/opt/panua-pardiso-20240229-linux/lib:/opt/artelys/knitro-15.0.1-Linux64/lib:$penv{LD_LIBRARY_PATH}"
      },
      "cacheVariables": {
        "PGO_ENABLE_CUDA": "ON",
        "PGO_USE_MKL": "ON",
        "PGO_HAS_ORIG_PARDISO": "OFF",
        "PARDISO_LIBRARY_HINT": "/opt/panua-pardiso-20240229-linux",
        "KNITRO_LIBRARY_HINT": "/opt/artelys/knitro-15.0.1-Linux64",
        "PGO_ENABLE_OPENVDB": "ON"
      }
    }
  ],
  "buildPresets": [
    {
      "name": "local-all",
      "configurePreset": "local-all",
      "jobs": 32
    }
  ]
}
```

</details>



### Dependency Ownership

- Conda supplies CMake, Ninja, and the native runtime/build packages for local
  source builds and CI wheels: Boost, MKL, TBB, Gmsh, OpenVDB, Imath,
  zlib, setuptools, and wheel. In conda environments, `numpy` stays on conda so
  it shares the same BLAS backend as the native extension.
- CI wheel artifacts follow the same ownership model: `pypgo` expects conda to
  supply NumPy and the BLAS/LAPACK runtime. Install artifact wheels with
  `--no-deps`.
- Pip supplies only optional pure-Python / pip-first packages that are not
  build-time native deps. `setup.py` keeps extras for convenience, but the base
  package itself declares no pip dependencies. Add optional pip packages
  explicitly:
  `torch` (official macOS arm64 wheel, with MPS; replaces conda `pytorch`),
  `pyvista` (pulls its own `vtk` wheel — conda `vtk-base` is intentionally not
  installed, to avoid a duplicate `vtkmodules` import path), `pytest`,
  `notebook`, and the `trame` / `trame-vtk` / `trame-vuetify` stack
  (conda-forge lags their releases).
- The host package manager supplies platform basics that are awkward to keep
  fully inside conda, such as macOS Homebrew GMP/MPFR/Imath.
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
