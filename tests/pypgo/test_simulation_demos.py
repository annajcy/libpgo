import ast
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
DEMO_ROOT = ROOT / "examples" / "demo" / "simulation"
CASES = {
    "tet_static_dragon": (
        "TetLinear",
        "EmbeddedVertexAttachment",
        "surface_embedding_matrix",
    ),
    "cubic_dynamic_box_sphere_ipc": (
        "CubicLinear",
        "ContactSurface.embedded",
        "DynamicSimulation",
    ),
    "shell_dynamic_ipc_drop": (
        "KoiterShell",
        "ContactSurface.identity",
        "DynamicSimulation",
    ),
    "cubic_static_box_hang_hermite": (
        "CubicTricubicHermite",
        "dofs_per_vertex",
        "surface_embedding_matrix",
    ),
}


@pytest.mark.parametrize(("case", "expected"), CASES.items())
def test_simulation_demo_uses_public_python_api(case, expected):
    source = (DEMO_ROOT / case / "main.py").read_text()
    tree = ast.parse(source)

    assert [node.name for node in tree.body if isinstance(node, ast.FunctionDef)] == [
        "main"
    ]
    assert "pypgo.tools.sim" not in source
    assert "json" not in source
    assert 'if __name__ == "__main__":' in source
    for symbol in expected:
        assert symbol in source


@pytest.mark.parametrize("case", CASES)
def test_simulation_demo_has_matching_readme_and_assets(case):
    readme = (DEMO_ROOT / case / "README.md").read_text()

    assert f"examples/sim_configs/{case}.json" in readme
    assert f"python examples/demo/simulation/{case}/main.py" in readme


def test_dynamic_demos_expose_short_run_controls():
    for case in ("cubic_dynamic_box_sphere_ipc", "shell_dynamic_ipc_drop"):
        source = (DEMO_ROOT / case / "main.py").read_text()
        assert "NUM_STEPS =" in source
        assert "DUMP_INTERVAL =" in source
        assert "if not frame.accepted:" in source


def test_simulation_demo_assets_exist():
    assets = ROOT / "examples" / "assets"
    expected = (
        assets / "veg" / "tet" / "dragon.veg",
        assets / "veg" / "cubic" / "box.veg",
        assets / "veg" / "cubic" / "box-with-sphere.veg",
        assets / "obj" / "dragon.obj",
        assets / "obj" / "box.obj",
        assets / "obj" / "box-with-sphere.obj",
        assets / "obj" / "shell.obj",
        assets / "obj" / "bottom.obj",
        assets / "fixed" / "dragon-surface-fixed.txt",
    )
    assert all(path.is_file() for path in expected)


def test_demo_outputs_are_case_local_and_ignored():
    for script in (ROOT / "examples" / "demo").glob("**/main.py"):
        source = script.read_text()
        assert ' / "output"' in source
        assert ' / "outputs"' not in source

    gitignore = (ROOT / ".gitignore").read_text().splitlines()
    assert "**/output/" in gitignore
