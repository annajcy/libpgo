# libpgo

[![Linux CI](https://github.com/annajcy/libpgo/actions/workflows/linux-ci.yml/badge.svg?branch=main)](https://github.com/annajcy/libpgo/actions/workflows/linux-ci.yml)
[![Windows CI](https://github.com/annajcy/libpgo/actions/workflows/windows-ci.yml/badge.svg?branch=main)](https://github.com/annajcy/libpgo/actions/workflows/windows-ci.yml)
[![macOS CI](https://github.com/annajcy/libpgo/actions/workflows/macos-ci.yml/badge.svg?branch=main)](https://github.com/annajcy/libpgo/actions/workflows/macos-ci.yml)

`libpgo` is a Python-first C++ library for physically based simulation,
geometric shape modeling, and optimization. Its `pypgo` package is the
recommended interface for most workflows: Python composes models and
experiments, while the C++ core owns performance-sensitive geometry, numerical
algorithms, and simulation state.

The project uses [uv](https://docs.astral.sh/uv/) for development and release
packaging. Prebuilt wheels are self-contained and do **not** require Conda or a
system installation of MKL, TBB, GMP, or MPFR.

## Install a prebuilt wheel

Release wheels are built for CPython 3.12:

| Platform | Supported target | Tag |
| --- | --- | --- |
| Linux | manylinux_2_28, x86-64 | `cp312-abi3-manylinux_2_28_x86_64` |
| macOS | macOS 26+, arm64 | `cp312-abi3-macosx_26_0_arm64` |
| Windows | Windows x86-64 | `cp312-abi3-win_amd64` |

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) first,
then create a fresh environment:

```bash
uv venv --python 3.12
uv pip install "numpy==2.0.2"
uv pip install --no-deps /path/to/pypgo-0.0.4-cp312-abi3-manylinux_2_28_x86_64.whl
uv run python -c "import pypgo; print(pypgo.__version__, pypgo._core.build_info())"
```

The macOS and Windows commands use the corresponding wheel filename from the
table above. Linux and Windows wheels bundle oneMKL, oneTBB, GMP, and MPFR;
the macOS wheel bundles oneTBB, GMP, and MPFR and links the system Accelerate
framework.

## Build from source

Source builds require Python 3.12, CMake 3.28 or newer, Ninja, and a C++20
compiler with OpenMP. Native dependencies are resolved as follows:

- GMP and MPFR are the only required system packages: `apt install libgmp-dev
  libmpfr-dev` on Debian/Ubuntu, `dnf install gmp-devel mpfr-devel` on
  Fedora/EL, `brew install gmp mpfr` on macOS.
- TBB: Homebrew `tbb` on macOS; on Linux and Windows it is installed into the
  uv environment by the pinned `mkl-devel` package (oneMKL + oneTBB).
- Everything else (Eigen, Boost, CGAL, Ceres, GMSH, OpenVDB, fTetWild, ...) is
  fetched and built from pinned upstream sources by CMake `FetchContent`.
- OpenMP: GCC/Clang provide it on Linux; macOS requires `brew install libomp`;
  Windows uses the MSVC OpenMP runtime.

Install uv, then:

```bash
uv sync --locked
uv run cmake --preset base -G Ninja
uv run cmake --build build/base
uv run ctest --test-dir build/base --output-on-failure

export PYTHONPATH="$PWD"
uv run python -m pytest -q tests/pypgo/test_pgo_smoke.py
```

`uv sync` installs CMake, Ninja, pytest, and the platform-specific MKL/TBB
packages, but does not install `pypgo` itself. The `base` preset builds the
extension into the repository-root `pypgo/` package, so `PYTHONPATH="$PWD"`
is enough to import it after an incremental C++ rebuild.

On Windows, run the commands from an x64 Native Tools PowerShell so `cl` and
`dumpbin` resolve to the MSVC toolchain, and use
`$env:PYTHONPATH = "$PWD"`.

## Package a self-contained wheel

Each platform has one packaging entry point:

```text
uv run python scripts/pypgo_wheel_<platform>.py package
  --wheel-dir <wheelhouse>
  [--report-dir <reports>]
```

The command builds a raw wheel with the `pypgo-wheel` CMake preset, repairs it
with `auditwheel` (Linux), `delocate` (macOS), or `delvewheel` (Windows),
audits the result, and only then copies the wheel into `--wheel-dir`.

## Usage & tools

The wheel installs the `pypgo-*` command-line tools:

```bash
uv run pypgo-sim-tet-static examples/configs/...json
uv run pypgo-animation-convert ...
```

See `python -m pypgo.tools` modules and `examples/` for runnable scenes.

## License

libpgo extends [VegaFEM](https://viterbi-web.usc.edu/~jbarbic/) for academic
research and integrates third-party libraries with their own licenses. See
[LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
