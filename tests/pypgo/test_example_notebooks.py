import ast
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
API_DOCS = ROOT / "docs" / "pypgo"


def guide_source(name: str) -> str:
    return (API_DOCS / f"{name}.md").read_text()


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


def test_api_guides_have_valid_python_blocks():
    for name in API_GUIDES:
        source = guide_source(name)
        assert "_api_demo" not in source
        assert "API Demo" not in source
        blocks = re.findall(r"```python\n(.*?)\n```", source, re.DOTALL)
        assert blocks, f"{name}.md has no Python examples"
        for index, block in enumerate(blocks):
            try:
                ast.parse(block)
            except SyntaxError as exc:
                raise AssertionError(
                    f"{name}.md Python block #{index} is invalid: {exc}"
                ) from exc


def test_tricubic_hermite_is_integrated_into_fem_guide():
    source = guide_source("fem")

    assert "pf.CubicTricubicHermite()" in source
    assert "num_vertices * 24" in source
    assert "surface_embedding_matrix" in source
    assert not (API_DOCS / "tricubic-hermite.md").exists()


def test_mesh_guide_uses_small_asset_independent_examples():
    source = guide_source("mesh")

    assert "TriMeshData" in source
    assert "VolumeMesh" in source
    assert "TemporaryDirectory" in source
    assert "ASSET_DIR" not in source
    assert "examples/assets" not in source


def test_implicit_guide_explains_lazy_field_pipeline():
    source = guide_source("implicit")

    assert "from pypgo import implicit" in source
    assert "ImplicitField → GridField → TriMeshData" in source
    assert "sample_to_grid" in source
    assert "extract_marching_cubes" in source
    assert "vis.plot_surface" not in source
