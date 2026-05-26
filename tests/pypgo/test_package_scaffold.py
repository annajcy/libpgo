import importlib.util


def test_python_first_package_imports_without_legacy_api():
    import pypgo

    assert pypgo.__all__ == ["mesh_geo", "mesh", "io"]
    assert importlib.util.find_spec("pypgo.legacy") is None


def test_public_modules_are_lazy_importable():
    import pypgo

    assert pypgo.mesh_geo.__name__ == "pypgo.mesh_geo"
    assert pypgo.mesh.__name__ == "pypgo.mesh"
    assert pypgo.io.__name__ == "pypgo.io"
