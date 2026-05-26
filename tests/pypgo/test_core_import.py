def test_private_core_import_and_metadata():
    import pypgo._core as c
    info = c.build_info()
    assert isinstance(info, dict)
    assert info["module"] == "pypgo._core"
    assert info["binding"] == "nanobind"
    assert info["mesh_geo"] is True
