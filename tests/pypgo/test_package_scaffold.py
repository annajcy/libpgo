import importlib.util


def test_python_first_package_imports_without_legacy_api():
    import pypgo

    assert pypgo.__all__ == ["mesh", "sim", "sparse", "tools"]
    assert importlib.util.find_spec("pypgo.legacy") is None
    assert importlib.util.find_spec("pypgo.mesh_geo") is None
    assert importlib.util.find_spec("pypgo.io") is None


def test_public_modules_are_lazy_importable():
    import pypgo

    assert pypgo.mesh.__name__ == "pypgo.mesh"
    assert pypgo.mesh.geo.__name__ == "pypgo.mesh.geo"
    assert pypgo.mesh.veg.__name__ == "pypgo.mesh.veg"
    assert pypgo.sim.__name__ == "pypgo.sim"
    assert pypgo.sparse.__name__ == "pypgo.sparse"
    assert pypgo.tools.__name__ == "pypgo.tools"
