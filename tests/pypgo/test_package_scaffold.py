import importlib.util


def test_python_first_package_imports_without_legacy_api():
    import pypgo

    assert pypgo.__all__ == []
    assert importlib.util.find_spec("pypgo.legacy") is None
