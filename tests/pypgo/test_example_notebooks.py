import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
EXAMPLES = ROOT / "examples"


def notebook_source(name: str) -> str:
    with open(EXAMPLES / name) as fh:
        nb = json.load(fh)
    return "\n".join("".join(cell["source"]) for cell in nb["cells"])


def test_mesh_api_demo_includes_pyvista_helpers_and_real_assets():
    source = notebook_source("mesh_api_demo.ipynb")

    assert "ASSET_DIR = REPO_ROOT / \"examples\" / \"assets\" / \"obj\"" in source
    assert "pip install -e .[examples]" in source
    assert "from pypgo.mesh.visualize import plot_surface, plot_volume_surface" in source

    assert "box.obj" in source
    assert "bunny.obj" in source
    assert "box-with-sphere.obj" in source
    assert "dragon.obj" in source


def test_implicit_api_demo_uses_vis_helpers_and_parallel_controls():
    source = notebook_source("implicit_api_demo.ipynb")

    assert "from pypgo import implicit" in source
    assert "from pypgo.mesh import visualize as vis" in source
    assert source.count("vis.plot_surface") >= 5
    assert "parallel_control = pgo.parallel.GlobalTbbControl(4)" in source
    assert "pgo.parallel.initialize" not in source


def test_static_solve_dragon_gravity_demo_uses_soft_surface_attachment():
    source = notebook_source("static_solve_dragon_gravity_demo.ipynb")

    assert '"dragon.obj"' in source
    assert '"dragon_big.veg"' in source
    assert '"dragon-surface-fixed.txt"' in source
    assert "fixed_vertices = np.loadtxt" in source
    assert "formulation = pf.TetLinear()" in source
    assert "pf.VolumetricPlasticity(dofs=0)" in source
    assert "gravity_energy = pe.LinearEnergy(-gravity_force)" in source
    assert "attachment_coeff = 1e5" in source
    assert "surface_attachment = pe.EmbeddedVertexAttachment" in source
    assert "formulation.surface_embedding_matrix" in source
    assert "vertex_indices=fixed_vertices" in source
    assert "vis.write_points_obj" in source
    assert "vis.plot_points_on_mesh(" in source
    assert "tet_data,\n    fixed_positions" in source
    assert "problem.fix_variables" not in source
