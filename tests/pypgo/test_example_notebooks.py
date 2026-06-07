import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = ROOT / "pypgo" / "examples" / "scripts"


def load_mesh_demo_generator():
    sys.path.insert(0, str(SCRIPT_DIR))
    try:
        spec = importlib.util.spec_from_file_location(
            "generate_mesh_api_demo_test",
            SCRIPT_DIR / "generate_mesh_api_demo.py",
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(SCRIPT_DIR))
    return module


def load_implicit_demo_generator():
    sys.path.insert(0, str(SCRIPT_DIR))
    try:
        spec = importlib.util.spec_from_file_location(
            "generate_implicit_api_demo_test",
            SCRIPT_DIR / "generate_implicit_api_demo.py",
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(SCRIPT_DIR))
    return module


def test_mesh_api_demo_includes_pyvista_helpers_and_real_assets():
    module = load_mesh_demo_generator()
    source = "\n".join(cell.source for cell in module.CELLS)

    assert "ASSET_DIR = REPO_ROOT / \"pypgo\" / \"examples\" / \"assets\" / \"obj\"" in source
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
