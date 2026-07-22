"""Validate API guides and optimization demo scripts.

Tests are split into two tiers:
- **Tier 1 – static** (always run): inspect guide and demo source.
- **Tier 2 – live execution** (opt-in via ``--run-notebooks`` / ``RUN_NOTEBOOKS=1``):
  execute API guide code against the installed pypgo package.
  Useful in CI with a full conda environment.

Usage::

    # Tier 1 only (fast, no pypgo import needed)
    PYPGO_RUN_NOTEBOOK_TESTS=1 pytest tests/pypgo/test_notebook.py -v

    # Tier 2 included
    PYPGO_RUN_NOTEBOOK_TESTS=1 pytest tests/pypgo/test_notebook.py -v --run-notebooks

    # Or via environment variable
    PYPGO_RUN_NOTEBOOK_TESTS=1 RUN_NOTEBOOKS=1 pytest tests/pypgo/test_notebook.py -v
"""

from __future__ import annotations

import ast
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

ROOT = Path(__file__).resolve().parents[2]
API_DOC_DIR = ROOT / "docs" / "pypgo"
OPTIMIZATION_DEMO_DIR = ROOT / "examples" / "demo" / "optimization"
API_GUIDES = (
    "animation",
    "contact",
    "energy",
    "fem",
    "implicit",
    "mesh",
    "numpy",
    "simulation",
    "solver",
)


def _api_guide_source(name: str) -> str:
    return (API_DOC_DIR / f"{name}.md").read_text()


def _api_guide_python(name: str) -> str:
    return "\n\n".join(
        re.findall(r"```python\n(.*?)\n```", _api_guide_source(name), re.DOTALL)
    )


def _optimization_demo_source(name: str) -> str:
    return (OPTIMIZATION_DEMO_DIR / name / "main.py").read_text()


def _optimization_demo_readme(name: str) -> str:
    return (OPTIMIZATION_DEMO_DIR / name / "README.md").read_text()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _run_notebook_flag(request: pytest.FixtureRequest) -> bool:
    """Return True when the caller has opted into live notebook execution."""
    if request.config.getoption("--run-notebooks", default=False):
        return True
    return os.environ.get("RUN_NOTEBOOKS", "") == "1"


# =========================================================================
# Tier 1 – Static validation (always runs)
# =========================================================================


class TestExampleSources:
    """Content-level smoke checks for API guides and optimization demos."""

    def test_energy_demo_covers_all_energy_types(self):
        source = _api_guide_source("energy")
        assert "LinearEnergy" in source
        assert "QuadraticEnergy" in source
        assert "VertexAttachment" in source
        assert "EnergySet" in source
        assert "max_step" in source

    def test_contact_guide_explains_the_shared_contact_contract(self):
        source = _api_guide_source("contact")
        assert "ContactSurface.identity" in source
        assert "ContactSurface.from_surface_embedding" in source
        assert "FloorEnergy" in source
        assert "pc.IPCEnergy" in source
        assert "pc.IPCParameters" in source
        assert "pc.SampledPenaltyEnergy" in source
        assert "pc.FrictionParameters" in source
        assert "begin_step" in source
        assert "SCENE = {" not in source
        assert "ASSET_DIR" not in source

    def test_mesh_guide_explains_object_layers_without_repo_assets(self):
        source = _api_guide_source("mesh")
        assert "TriMeshData" in source
        assert "TriMeshGeo" in source
        assert "VolumeMesh" in source
        assert "SimulationMesh.create_volumetric" in source
        assert "SurfaceEmbedding" in source
        assert "ASSET_DIR" not in source

    def test_deformation_fem_picks_up_formulation(self):
        source = _api_guide_source("fem")
        assert "TetLinear" in source
        assert "CubicLinear" in source
        assert "KoiterShell" in source
        assert "CubicTricubicHermite" in source
        assert "num_vertices * 24" in source
        assert "surface_embedding_matrix" in source
        assert "deformation_energy" in source

    def test_solver_demo_covers_newton_api(self):
        source = _api_guide_source("solver")
        assert "OptimizationProblem" in source
        assert "NewtonOptimizer" in source
        assert "fix_variables" in source
        assert "line_search" in source
        assert "ConstraintPenalty" in source
        assert "ConstraintViolationPenalty" in source

    def test_animation_guide_uses_in_memory_data_and_capability_checks(self):
        source = _api_guide_source("animation")
        assert "AbcWriter" in source
        assert "has_animation_io" in source
        assert "has_stress_vdb_export" in source
        assert "write_u_file" in source
        assert "compute_stress_field_stats" in source
        assert "ASSET_DIR" not in source

    def test_numpy_demo_demonstrates_interop(self):
        source = _api_guide_source("numpy")
        assert "ndarray" in source or "numpy" in source

    def test_plastic_shape_match_demo_covers_inverse_design_path(self):
        source = _optimization_demo_source("plastic_shape_match")
        ast.parse(source)
        assert "pgo.fem.PlasticStaticEquilibriumLayer" in source
        assert "pf.ElementwiseField()" in source
        assert "volume.extract_surface_mesh()" in source
        assert "torch.optim.Adam" in source
        assert "loss.backward()" in source
        assert "plastic_shape_match_weights.npz" in source
        assert "np.savez" in source
        readme = _optimization_demo_readme("plastic_shape_match")
        assert "PlasticStaticEquilibriumLayer" in readme
        assert "elementwise field" in readme

    def test_elastic_material_optimization_demo_covers_inverse_design_path(self):
        source = _optimization_demo_source("elastic_material_optimization")
        ast.parse(source)
        assert "pgo.fem.ElasticStaticEquilibriumLayer" in source
        assert "objective_energy=energy" in source
        assert "external_load=" in source
        assert "ShellDensityElasticThickness" in source
        assert "SelfWeightGravity" in source
        assert "pe.LinearEnergy(" in source
        assert "torch.nn.Sequential" in source
        assert "elastic[0::5] = membrane" in source
        assert "torch.optim.Adam" in source
        assert "loss.backward()" in source
        assert "elastic_shape_match_weights.npz" in source
        assert "np.savez" in source
        readme = _optimization_demo_readme("elastic_material_optimization")
        assert "ElasticStaticEquilibriumLayer" in readme
        assert "added exactly once" in readme


# =========================================================================
# Tier 2 – Live execution (opt-in)
# =========================================================================


class TestApiGuideExecution:
    """Execute each API guide's Python blocks in document order."""

    @pytest.mark.parametrize("name", API_GUIDES)
    def test_api_guide_executes_cleanly(self, name, request):
        if not _run_notebook_flag(request):
            pytest.skip("opt-in via --run-notebooks or RUN_NOTEBOOKS=1")
        result = subprocess.run(
            [sys.executable, "-c", _api_guide_python(name)],
            capture_output=True,
            text=True,
            timeout=360,
            cwd=ROOT,
        )
        if result.returncode != 0:
            output = result.stdout + "\n" + result.stderr
            tail = output[-2000:] if len(output) > 2000 else output
            pytest.fail(
                f"{name}.md execution failed (exit {result.returncode}):\n{tail}"
            )
