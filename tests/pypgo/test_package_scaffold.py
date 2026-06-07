import importlib.util


def test_python_first_package_imports_without_legacy_api():
    import pypgo

    assert "mesh" in pypgo.__all__
    assert "sim" in pypgo.__all__
    assert "sparse" in pypgo.__all__
    assert "tools" in pypgo.__all__
    assert "fem" in pypgo.__all__
    assert "energy" in pypgo.__all__
    assert "constraints" in pypgo.__all__
    assert "solver" in pypgo.__all__
    assert importlib.util.find_spec("pypgo.legacy") is None
    assert importlib.util.find_spec("pypgo.mesh_geo") is None
    assert importlib.util.find_spec("pypgo.io") is None


def test_public_modules_are_lazy_importable():
    import pypgo

    assert pypgo.mesh.__name__ == "pypgo.mesh"
    assert pypgo.mesh.geo.__name__ == "pypgo.mesh.geo"
    assert pypgo.mesh.volume.__name__ == "pypgo.mesh.volume"
    assert pypgo.sim.__name__ == "pypgo.sim"
    assert pypgo.sparse.__name__ == "pypgo.sparse"
    assert pypgo.tools.__name__ == "pypgo.tools"
    assert pypgo.constraints.__name__ == "pypgo.constraints"
    assert pypgo.solver.__name__ == "pypgo.solver"
