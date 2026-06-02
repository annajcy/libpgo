"""Generate, validate, and optionally execute all example notebooks.

Tests are split into two tiers:
- **Tier 1 – static** (always run): generate notebooks, check nbformat compliance,
  verify all cells are well-formed, parse code for syntax errors.
- **Tier 2 – live execution** (opt-in via ``--run-notebooks`` / ``RUN_NOTEBOOKS=1``):
  actually execute every notebook against the installed pypgo package.
  Useful in CI with a full conda environment.

Usage::

    # Tier 1 only (fast, no pypgo import needed)
    pytest tests/pypgo/test_notebook.py -v

    # Tier 2 included
    pytest tests/pypgo/test_notebook.py -v --run-notebooks

    # Or via environment variable
    RUN_NOTEBOOKS=1 pytest tests/pypgo/test_notebook.py -v
"""

from __future__ import annotations

import ast
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import nbformat
import pytest

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = ROOT / "pypgo" / "examples" / "scripts"
OUTPUT_DIR = ROOT / "pypgo" / "examples"

# Map notebook stem → generator script filename.
GENERATORS: dict[str, str] = {
    "energy_api_demo": "generate_energy_api_demo.py",
    "mesh_api_demo": "generate_mesh_api_demo.py",
    "deformation_fem_api_demo": "generate_deformation_fem_api_demo.py",
    "animation_api_demo": "generate_animation_api_demo.py",
    "numpy_interoperate": "generate_numpy_interoperate.py",
    "solver_api_demo": "generate_solver_api_demo.py",
    "implicit_api_demo": "generate_implicit_api_demo.py",
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _run_notebook_flag(request: pytest.FixtureRequest) -> bool:
    """Return True when the caller has opted into live notebook execution."""
    if request.config.getoption("--run-notebooks", default=False):
        return True
    return os.environ.get("RUN_NOTEBOOKS", "") == "1"


def _load_generator_module(script_name: str):
    """Import a generator script as a module without affecting sys.path."""
    spec = importlib.util.spec_from_file_location(
        f"generator_{script_name.replace('.py', '')}",
        SCRIPT_DIR / script_name,
    )
    module = importlib.util.module_from_spec(spec)
    sys_path = sys.path.copy()
    sys.path.insert(0, str(SCRIPT_DIR))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path = sys_path
    return module


def _generate_notebook(notebook_stem: str, tmp_path: Path) -> Path:
    """Run the generator and write the .ipynb to *tmp_path*."""
    script_name = GENERATORS[notebook_stem]
    module = _load_generator_module(script_name)

    # The generators export a CELLS list and rely on write_notebook.
    cells = module.CELLS
    out = tmp_path / f"{notebook_stem}.ipynb"
    module.write_notebook(out, cells)
    return out


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="session")
def generated_notebooks(tmp_path_factory: pytest.TempPathFactory):
    """Generate every example notebook into a temporary directory.

    Returns ``dict[stem, Path]`` so individual tests can reuse the files
    without re-generating.
    """
    tmp = tmp_path_factory.mktemp("notebooks")
    result: dict[str, Path] = {}
    for stem in GENERATORS:
        result[stem] = _generate_notebook(stem, tmp)
    return result


# =========================================================================
# Tier 1 – Static validation (always runs)
# =========================================================================


class TestNotebookGeneration:
    """Verify that every generator produces a valid .ipynb file."""

    @pytest.mark.parametrize("stem", list(GENERATORS))
    def test_notebook_is_valid_nbformat(self, stem, generated_notebooks):
        """Generated file passes nbformat validation."""
        path = generated_notebooks[stem]
        with open(path) as fh:
            nb = nbformat.reader.read(fh)
        nbformat.validate(nb)
        assert nb["nbformat"] >= 4

    @pytest.mark.parametrize("stem", list(GENERATORS))
    def test_notebook_has_cells(self, stem, generated_notebooks):
        """Notebook contains at least one cell."""
        path = generated_notebooks[stem]
        with open(path) as fh:
            nb = json.load(fh)
        assert len(nb["cells"]) > 0, f"{stem} has zero cells"

    @pytest.mark.parametrize("stem", list(GENERATORS))
    def test_notebook_has_kernel_metadata(self, stem, generated_notebooks):
        """Kernelspec metadata is present (required for execution)."""
        path = generated_notebooks[stem]
        with open(path) as fh:
            nb = json.load(fh)
        ks = nb["metadata"]["kernelspec"]
        assert ks["display_name"] == "Python 3"
        assert ks["language"] == "python"

    @pytest.mark.parametrize("stem", list(GENERATORS))
    def test_markdown_cells_have_content(self, stem, generated_notebooks):
        """Every markdown cell has non-whitespace source."""
        path = generated_notebooks[stem]
        with open(path) as fh:
            nb = json.load(fh)
        for idx, cell in enumerate(nb["cells"]):
            if cell["cell_type"] == "markdown":
                src = "".join(cell["source"]).strip()
                assert src, f"{stem} cell #{idx} markdown cell is empty"

    @pytest.mark.parametrize("stem", list(GENERATORS))
    def test_code_cells_have_unique_ids(self, stem, generated_notebooks):
        """Every cell has a unique non-empty id."""
        path = generated_notebooks[stem]
        with open(path) as fh:
            nb = json.load(fh)
        ids = [c["id"] for c in nb["cells"]]
        assert len(ids) == len(set(ids)), f"{stem} has duplicate cell ids"
        assert all(ids), f"{stem} has empty cell ids"


class TestNotebookSyntax:
    """Every code cell in every generated notebook is syntactically valid Python."""

    @pytest.mark.parametrize("stem", list(GENERATORS))
    def test_all_code_cells_parse(self, stem, generated_notebooks):
        path = generated_notebooks[stem]
        with open(path) as fh:
            nb = json.load(fh)
        errors: list[str] = []
        for idx, cell in enumerate(nb["cells"]):
            if cell["cell_type"] != "code":
                continue
            src = "".join(cell["source"])
            try:
                ast.parse(src)
            except SyntaxError as exc:
                errors.append(
                    f"{stem} cell #{idx} (id={cell['id']}): {exc}"
                )
        assert not errors, "\n".join(errors)


class TestNotebookSources:
    """Content-level smoke checks for each notebook."""

    def test_energy_demo_covers_all_energy_types(self, generated_notebooks):
        with open(generated_notebooks["energy_api_demo"]) as fh:
            nb = json.load(fh)
        source = "\n".join("".join(c["source"]) for c in nb["cells"] if c["cell_type"] == "code")
        assert "LinearEnergy" in source
        assert "QuadraticEnergy" in source
        assert "VertexAttachment" in source
        assert "EnergySet" in source
        assert "max_step" in source

    def test_mesh_demo_loads_assets(self, generated_notebooks):
        with open(generated_notebooks["mesh_api_demo"]) as fh:
            nb = json.load(fh)
        source = "\n".join("".join(c["source"]) for c in nb["cells"] if c["cell_type"] == "code")
        assert "VolumeMesh" in source
        assert "SimulationMesh.create_volumetric" in source
        assert "SimulationMesh.create_shell" in source

    def test_deformation_fem_picks_up_formulation(self, generated_notebooks):
        with open(generated_notebooks["deformation_fem_api_demo"]) as fh:
            nb = json.load(fh)
        source = "\n".join("".join(c["source"]) for c in nb["cells"] if c["cell_type"] == "code")
        assert "TetP1" in source
        assert "LinearCubic" in source
        assert "KoiterShell" in source
        assert "deformation_energy" in source

    def test_solver_demo_covers_newton_api(self, generated_notebooks):
        with open(generated_notebooks["solver_api_demo"]) as fh:
            nb = json.load(fh)
        source = "\n".join("".join(c["source"]) for c in nb["cells"] if c["cell_type"] == "code")
        assert "solve_newton" in source
        assert "NewtonOptions" in source
        assert "fixed_dofs" in source
        assert "line_search" in source
        assert "EnergySet" in source
        assert "constraints.Linear" in source
        assert "constraints.Bounded" in source
        assert "ConstraintPenalty" in source
        assert "ConstraintViolationPenalty" in source

    def test_animation_demo_uses_animation_io(self, generated_notebooks):
        with open(generated_notebooks["animation_api_demo"]) as fh:
            nb = json.load(fh)
        source = "\n".join("".join(c["source"]) for c in nb["cells"] if c["cell_type"] == "code")
        assert "AnimationReader" in source or "animation" in source.lower()

    def test_numpy_demo_demonstrates_interop(self, generated_notebooks):
        with open(generated_notebooks["numpy_interoperate"]) as fh:
            nb = json.load(fh)
        source = "\n".join("".join(c["source"]) for c in nb["cells"] if c["cell_type"] == "code")
        assert "ndarray" in source or "numpy" in source


# =========================================================================
# Tier 2 – Live execution (opt-in)
# =========================================================================


def _module_exists(name: str) -> bool:
    try:
        importlib.import_module(name)
        return True
    except ImportError:
        return False


def _exec_notebook(path: Path) -> tuple[int, str]:
    """Run a notebook with ``jupyter nbconvert --execute``.

    Returns (returncode, combined stdout+stderr).
    """
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "jupyter",
            "nbconvert",
            "--to",
            "notebook",
            "--execute",
            "--ExecutePreprocessor.timeout=300",
            "--output",
            path.name,  # write to same temp dir
            str(path),
        ],
        capture_output=True,
        text=True,
        timeout=360,
        cwd=path.parent,
    )
    return result.returncode, result.stdout + "\n" + result.stderr


class TestNotebookExecution:
    """Execute every generated notebook end-to-end.

    These tests require a complete pypgo installation and all example
    assets (meshes, textures, etc.) to be present.  Skip when any
    prerequisite is missing or the user has not opted in.
    """

    def test_pypgo_is_installed(self):
        """Sanity: pypgo (at minimum 'energy' sub-package) imports."""
        try:
            import pypgo.energy  # noqa: F401
        except ImportError as exc:
            pytest.skip(f"pypgo.energy not importable: {exc}")

    @pytest.mark.parametrize(
        "stem",
        [
            p
            for p in GENERATORS
            if p != "animation_api_demo"  # animation needs runIPCSim output
        ],
    )
    def test_notebook_executes_cleanly(
        self, stem, generated_notebooks, request
    ):
        """Execute the notebook and assert zero exit code."""
        if not _run_notebook_flag(request):
            pytest.skip("opt-in via --run-notebooks or RUN_NOTEBOOKS=1")

        # Check pyvista where needed (mesh / deformation fem).
        if stem in ("mesh_api_demo", "deformation_fem_api_demo") and not _module_exists("pyvista"):
            pytest.skip("pyvista not installed – rendering cells may fail")

        path = generated_notebooks[stem]
        rc, output = _exec_notebook(path)
        if rc != 0:
            # Print tail of output for debugging.
            tail = output[-2000:] if len(output) > 2000 else output
            pytest.fail(
                f"{stem} execution failed (exit {rc}):\n{tail}"
            )
