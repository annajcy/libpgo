import importlib.util

import pypgo as pgo
from pypgo.tools.mesh.surface import cleanup as surface_cleanup
from pypgo.tools.mesh.surface import quality as surface_quality
from pypgo.tools.mesh.surface import remesh as surface_remesh
from pypgo.tools.mesh.volume import cubic_mesher, ftetwild_mesher, tetgen_mesher, volume_info
from pypgo.tools.sim import volume_ipc


def test_tool_modules_are_per_cli_files():
    assert volume_info.main is not None
    assert cubic_mesher.main is not None
    assert tetgen_mesher.main is not None
    assert ftetwild_mesher.main is not None
    assert surface_quality.main is not None
    assert surface_remesh.main is not None
    assert surface_cleanup.main is not None
    assert volume_ipc.main is not None


def test_tools_package_remains_cli_facing_namespace():
    assert pgo.tools.__all__ == ["mesh", "sim"]
    assert pgo.tools.mesh.__all__ == ["surface", "volume"]
    assert importlib.util.find_spec("pypgo.tools.stress") is None
    assert importlib.util.find_spec("pypgo.tools.mesh.tet_mesher") is None
    assert importlib.util.find_spec("pypgo.tools.mesh.surface_remesher") is None
    assert importlib.util.find_spec("pypgo.tools.mesh.surface_smooth") is None
    assert importlib.util.find_spec("pypgo.tools.mesh.surface_repair") is None
    assert importlib.util.find_spec("pypgo.tools.mesh.surface_simplify") is None
    assert importlib.util.find_spec("pypgo.tools.mesh.surface_remove_isolated_vertices") is None
    assert importlib.util.find_spec("pypgo.tools.mesh.surface_merge_close_vertices") is None
    assert importlib.util.find_spec("pypgo.tools.mesh.volume_info") is None
    assert importlib.util.find_spec("pypgo.tools.mesh.cubic_mesher") is None
