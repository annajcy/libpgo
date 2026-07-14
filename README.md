## libpgo: Library for Physically based Simulation (P), Geometric Shape Modeling (G), and Optimization (O)

**libpgo** is designed to primarily focus on physically based simulations, geometric shape modeling, and optimization.
The source code extends [VegaFEM](https://viterbi-web.usc.edu/~jbarbic/vega/) and is designed for academic research purposes.

**pypgo** is the python binding of libpgo, which is the recommended main entry point for users.

---

## System Prerequisites

### Install conda

See [Conda Installation](https://www.anaconda.com/docs/getting-started/miniconda/install/overview#choose-your-installation-guide).

### Install build tools

CMake 3.28 or newer, Ninja, pkg-config, and the platform compiler from
the system package manager / platform toolchain. Conda owns the project
libraries and Python environment, not the build tools.

| Platform | System build tools | Notes |
| --- | --- | --- |
| Linux | `gcc`, `g++`, `cmake`, `ninja-build`, `pkg-config` | Ubuntu 24.04 packages are sufficient. |
| macOS | Xcode command line tools, Homebrew `cmake`, `ninja`, `pkg-config` | The Xcode SDK provides the platform toolchain. |
| Windows | Visual Studio 2022 MSVC, CMake, Ninja | Run builds from an x64 MSVC developer shell. |

#### Linux:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake ninja-build pkg-config
```

<details>
<summary>What if you are using Ubuntu 22.04</summary>

Ubuntu 22.04's default packages are too old for libpgo's source build
baseline: GCC is 11 and CMake is 3.22. Keep `ninja-build` and `pkg-config`
from apt, but install a newer CMake and GCC:

Install base build utilities, Ninja, and pkg-config:

```bash
sudo apt-get update
sudo apt-get install -y \
  ca-certificates gpg wget software-properties-common \
  build-essential ninja-build pkg-config
```

Install CMake from Kitware's apt repository:

```bash
test -f /usr/share/doc/kitware-archive-keyring/copyright || \
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | \
gpg --dearmor - | \
sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null

echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | \
sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null

sudo apt-get update
test -f /usr/share/doc/kitware-archive-keyring/copyright || \
sudo rm /usr/share/keyrings/kitware-archive-keyring.gpg
sudo apt-get install -y kitware-archive-keyring cmake
```

Check that the default CMake tools are the Kitware versions:

```bash
which -a cmake ctest cpack
cmake --version
ctest --version
cpack --version
```

If another older CMake installation appears first on `PATH` (e.g. a `pip
install cmake` wrapper in `/usr/local/bin`), remove it and register the
Kitware version with `update-alternatives`:

```bash
sudo rm /usr/local/bin/cmake /usr/local/bin/ctest /usr/local/bin/cpack 2>/dev/null
sudo update-alternatives --install /usr/local/bin/cmake cmake /usr/bin/cmake 100
sudo update-alternatives --install /usr/local/bin/ctest ctest /usr/bin/ctest 100
sudo update-alternatives --install /usr/local/bin/cpack cpack /usr/bin/cpack 100
```

Avoid `export PATH="/usr/bin:..."` in shell startup files — it overrides
the entire `PATH` and breaks conda environment activation.

Install GCC 13 / G++ 13 from the Ubuntu Toolchain PPA. Add the PPA source
directly; this avoids `add-apt-repository`, which may time out while contacting
the Launchpad API:

```bash
sudo install -d -m 0755 /etc/apt/keyrings

wget -O /tmp/ubuntu-toolchain-r-test.asc \
  'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x1E9377A2BA9EF27F'

sudo gpg --batch --yes --dearmor \
  -o /etc/apt/keyrings/ubuntu-toolchain-r-test.gpg \
  /tmp/ubuntu-toolchain-r-test.asc

echo "deb [signed-by=/etc/apt/keyrings/ubuntu-toolchain-r-test.gpg] https://ppa.launchpadcontent.net/ubuntu-toolchain-r/test/ubuntu jammy main" | \
  sudo tee /etc/apt/sources.list.d/ubuntu-toolchain-r-test.list >/dev/null

sudo apt-get update
apt-cache policy gcc-13 g++-13
sudo apt-get install -y gcc-13 g++-13
```

Then point CMake at the newer compiler for local builds:

```bash
cmake --preset base \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-13 \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-13
cmake --build --preset base
```

Optionally make GCC 13 the default `gcc` / `g++` on a personal development
machine or container:

```bash
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 130
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 130
sudo update-alternatives --config gcc
sudo update-alternatives --config g++
```

</details>

#### macOS:

```bash
xcode-select --install
brew install cmake ninja pkg-config
```

#### Windows:

- Install Visual Studio 2022 with the C++ desktop workload.
- Install CMake and Ninja, or use the versions bundled with the Visual Studio /
  GitHub Actions runner image.
- Run builds from an x64 MSVC developer shell.

## Install pypgo with Python Only

Use a CI wheel artifact when you only need the Python package and do not want
to build the C++ source. The wheels are conda-environment artifacts, not
standalone PyPI wheels. `pypgo` does not declare a pip NumPy dependency, so
NumPy and its BLAS/LAPACK runtime must come from the same conda environment as
the extension.

#### Linux and Windows use MKL:

```bash
conda create -n pypgo -c conda-forge python=3.12 pip numpy "libblas=*=*mkl" "liblapack=*=*mkl" mkl-devel
conda env config vars set -n pypgo MKL_THREADING_LAYER=TBB
conda activate pypgo
python -m pip install --no-deps pypgo-*.whl
```

The conda MKL BLAS packages route NumPy through the `mkl_rt` dispatcher; they
do not select the TBB threading layer by themselves. Set
`MKL_THREADING_LAYER=TBB` before importing NumPy so NumPy and `pypgo` share the
same TBB runtime. `pypgo.parallel.initialize(max_concurrency=...)` can then
apply one process-wide TBB concurrency ceiling instead of leaving a separate
MKL OpenMP pool outside that ceiling.

#### macOS NumPy may use Accelerate:

```bash
conda create -n pypgo -c conda-forge python=3.12 pip numpy "libblas=*=*accelerate" "liblapack=*=*accelerate"
conda activate pypgo
python -m pip install --no-deps pypgo-*.whl
```

This selects NumPy's application-owned BLAS. Native libpgo/pypgo targets do
not link Accelerate, BLAS, or LAPACK on macOS and do not control NumPy's BLAS
threads.

Some optional geometry backends, including Gmsh and OpenVDB, may bring an
OpenMP runtime into the Python process. Keep those calls outside
`pgo::parallel::parallelFor` / TBB worker bodies; nesting OpenMP-backed APIs
inside TBB loops can oversubscribe CPU threads.

## Build and Install pypgo from Source

Conda is the recommended build environment for both the Python package and the
native CMake build. Use one conda environment for Python, NumPy, BLAS/LAPACK,
Boost, TBB, Gmsh, OpenVDB, and other native dependencies. Use system CMake,
Ninja, pkg-config, and the platform compiler.

- The Python package is being redesigned as a Python-first API. The old
  C-style Python wrapper has been removed and its functionality will return
  through focused Python modules.
- The native CMake build uses the `base` preset, which enables the full default
  feature set including Gmsh, OpenVDB, TBB, and MKL where supported.
- The compiler follows the platform owner: system GCC/G++ on Linux, Apple
  Clang on macOS, and MSVC on Windows.

Use Miniforge or Miniconda when possible, and keep packages on the
`conda-forge` channel. The commands below use `conda` consistently for Python,
BLAS/LAPACK, and native project dependencies.

### Create the conda environment

C++ libraries and the conda-side Python packages are declared in
`environment.yml`; pip-managed Python dependencies for the default `pypgo`
flavor are installed by the editable build step below. CMake, Ninja,
pkg-config, and the compiler come from the system prerequisites above. Create
the `libpgo` environment with a single command:

```bash
conda env create -f environment.yml
conda activate libpgo
```

To update an existing environment after pulling changes:

```bash
conda env update -f environment.yml --prune
conda activate libpgo
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

#### Linux / Windows
```bash
conda install -n libpgo -c conda-forge mkl-devel "libblas=*=*mkl" "liblapack=*=*mkl"
conda env config vars set -n libpgo MKL_THREADING_LAYER=TBB
conda deactivate
conda activate libpgo
```

#### macOS
```bash
conda install -n libpgo -c conda-forge "libblas=*=*accelerate" "liblapack=*=*accelerate"
```

On Linux and Windows, the environment variable makes NumPy's `mkl_rt` dispatcher
select `mkl_tbb_thread`. The native build's `PGO_MKL_THREADING=tbb_thread`
selects the libpgo link-time MKL layer; it does not configure NumPy. The
threading-layer variable must therefore be active before the first NumPy/MKL
import in each process.

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
and the C API. Linux/Windows builds use oneMKL with its TBB threading layer;
macOS native targets use Eigen kernels, do not link Accelerate/BLAS/LAPACK,
and force `PGO_ENABLE_CUDA=OFF`.

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

- The system package manager supplies CMake, Ninja, pkg-config, and the
  platform compiler for local source builds and CI.
- Conda supplies Python plus native project dependencies for local source
  builds and CI wheels: Boost, MKL, TBB, Gmsh, OpenVDB, GMP, MPFR, Imath, zlib,
  setuptools, and wheel. In conda environments, `numpy` stays on conda; its
  BLAS runtime is application-owned and separate from libpgo's concurrency
  contract.
- CI wheel artifacts follow the same ownership model: `pypgo` expects conda to
  supply NumPy and NumPy's own runtime dependencies. Install artifact wheels
  with `--no-deps`.
- Pip supplies only optional pure-Python / pip-first packages that are not
  build-time native deps. `setup.py` keeps extras for convenience, but the base
  package itself declares no pip dependencies. Add optional pip packages
  explicitly:
  `torch` (official macOS arm64 wheel, with MPS; replaces conda `pytorch`),
  `pyvista` (pulls its own `vtk` wheel — conda `vtk-base` is intentionally not
  installed, to avoid a duplicate `vtkmodules` import path), `pytest`,
  `notebook`, and the `trame` / `trame-vtk` / `trame-vuetify` stack
  (conda-forge lags their releases).
- FetchContent-managed C++ dependencies are downloaded and built by this
  repository: Eigen, fmt, spdlog, nlohmann_json, Ceres, CGAL,
  geogram, libigl, Alembic, nanobind, and related internal dependencies.

---

## CPU Parallelism

Native libpgo code starts CPU work only through `pgo::parallel`. Configure the
repeatable process-wide ceiling with `pypgo.parallel.initialize(max_concurrency=...)`.
The first-party scheduling primitives (`parallelFor`, `parallelForChunks`,
`parallelReduce`, and `parallelSort`) enter an arena aligned with that ceiling,
so oneTBB task decomposition sees the same limit. Use
`withTbbConcurrencyLimit(N, fn)` around a nested TBB/oneMKL-TBB body when it
needs a smaller bound; `withSingleThreadedTbb(fn)` is the `N=1` form. Nested
bounds are monotonic: an inner helper or pgo scheduling call can only retain
or lower an enclosing bound. Algorithms default to `tbb::auto_partitioner`;
pass a oneTBB partitioner object such as `tbb::static_partitioner{}` for an
explicit choice. The canonical form is
`parallelFor(begin, end, fn, grainSize, partitioner)`; the convenience spelling
`parallelFor(begin, end, tbb::static_partitioner{}, fn)` is also available.
An explicitly supplied grain size must be positive.

oneMKL builds must use `PGO_MKL_THREADING=tbb_thread`. Libpgo does not call MKL
or Accelerate thread setters and does not control application-owned NumPy,
OpenMP, BLAS, or LAPACK runtimes.

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
