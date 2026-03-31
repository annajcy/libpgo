# Python Bindings Guide

Python packaging uses **uv + pyproject.toml + scikit-build-core + CMake**.

## Editable Build

```bash
cd libpgo
uv sync
```

This will:

- Create or update `.venv`
- Install Python dependencies
- Configure CMake through scikit-build-core
- Run the Conan dependency bootstrap during configure
- Build the `pypgo` extension in editable mode

## Verify Installation

```bash
uv run python -c "import pypgo; print(pypgo.__version__)"
```

## Run Tests

```bash
uv run pytest
```

## Build Distributable Wheels

```bash
uv build
```

Artifacts are written to `dist/`.

## CMake-Only Python Build

For a Python binding build without animation export:

```bash
cmake --preset python-minimal-release
cmake --build --preset python-minimal-release
```

## Dev Loop

For day-to-day iteration:

```bash
uv sync
uv run pytest
```

scikit-build-core reuses the build tree under `build/scikit-build`, so repeated `uv sync` benefits from Ninja and CMake incremental rebuilds.
