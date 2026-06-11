import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = ROOT / "examples" / "scripts"


def load_generator(module_name, filename):
    sys.path.insert(0, str(SCRIPT_DIR))
    try:
        spec = importlib.util.spec_from_file_location(
            module_name,
            SCRIPT_DIR / filename,
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(SCRIPT_DIR))
    return module


def load_mesh_demo_generator():
    return load_generator("generate_mesh_api_demo_test", "generate_mesh_api_demo.py")


def load_implicit_demo_generator():
    return load_generator("generate_implicit_api_demo_test", "generate_implicit_api_demo.py")


def load_static_solve_dragon_generator():
    return load_generator(
        "generate_static_solve_dragon_gravity_demo_test",
        "generate_static_solve_dragon_gravity_demo.py",
    )


def test_mesh_api_demo_includes_pyvista_helpers_and_real_assets():
    module = load_mesh_demo_generator()
    source = "\n".join(cell.source for cell in module.CELLS)

    assert "ASSET_DIR = REPO_ROOT / \"examples\" / \"assets\" / \"obj\"" in source
    assert "pip install -e .[examples]" in source
    assert "from pypgo.mesh.visualize import plot_surface, plot_volume_surface" in source

    assert "box.obj" in source
    assert "bunny.obj" in source
    assert "box-with-sphere.obj" in source
    assert "dragon.obj" in source


def test_implicit_api_demo_uses_vis_helpers_and_parallel_controls():
    module = load_implicit_demo_generator()
    source = "\n".join(cell.source for cell in module.CELLS)

    assert "from pypgo import implicit" in source
    assert "from pypgo.mesh import visualize as vis" in source
    assert source.count("vis.plot_surface") >= 5
    assert "pgo.parallel.thread_limit" in source
    assert "pgo.parallel.set_num_threads" in source


def test_static_solve_dragon_gravity_demo_uses_soft_surface_attachment():
    module = load_static_solve_dragon_generator()
    source = "\n".join(cell.source for cell in module.CELLS)

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
    assert "tet_data,\n            fixed_positions" in source
    assert "problem.fix_variables" not in source
