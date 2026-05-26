import numpy as np
import pypgo as pgo


def test_veg_tet_io_roundtrip(tmp_path):
    """Write and read back a tet .veg file."""
    vertices = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)
    cells = np.array([[0, 1, 2, 3]], dtype=np.int64)

    tet_geo = pgo.mesh_geo.TetCellMeshGeo(vertices, cells)
    material = pgo.mesh.MaterialSpec(1e6, 0.33, 1200.0)

    veg_file = str(tmp_path / "temp_mesh.veg")
    pgo.io.write_veg_geo(veg_file, tet_geo, material)

    # Load it back
    loaded_geo, loaded_material = pgo.io.read_veg_geo(veg_file)

    assert isinstance(loaded_geo, pgo.mesh_geo.TetCellMeshGeo)
    assert loaded_geo.num_vertices == 4
    assert loaded_geo.num_cells == 1
    assert np.allclose(loaded_geo.vertices, vertices)
    assert np.array_equal(loaded_geo.cells, cells)

    assert isinstance(loaded_material, pgo.mesh.MaterialSpec)
    assert loaded_material.E == 1e6
    assert loaded_material.nu == 0.33
    assert loaded_material.density == 1200.0


def test_obj_io_roundtrip(tmp_path):
    """Write and read back an .obj file."""
    vertices = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0]
    ], dtype=np.float64)
    cells = np.array([[0, 1, 2]], dtype=np.int64)

    surface = pgo.mesh_geo.TriCellMeshGeo(vertices, cells)

    obj_file = str(tmp_path / "temp_surface.obj")
    pgo.io.write_obj_geo(obj_file, surface)

    # Load it back
    loaded_surface = pgo.io.read_obj_geo(obj_file)

    assert isinstance(loaded_surface, pgo.mesh_geo.TriCellMeshGeo)
    assert loaded_surface.num_vertices == 3
    assert loaded_surface.num_cells == 1
    assert np.allclose(loaded_surface.vertices, vertices)
    assert np.array_equal(loaded_surface.cells, cells)


def test_veg_roundtrip_then_volume_mesh(tmp_path):
    """Verify that geometry read from .veg can construct a VolumeMesh."""
    vertices = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)
    cells = np.array([[0, 1, 2, 3]], dtype=np.int64)

    tet_geo = pgo.mesh_geo.TetCellMeshGeo(vertices, cells)
    material = pgo.mesh.MaterialSpec(2e9, 0.4, 900.0)

    veg_file = str(tmp_path / "temp2.veg")
    pgo.io.write_veg_geo(veg_file, tet_geo, material)

    loaded_geo, loaded_mat = pgo.io.read_veg_geo(veg_file)

    # Should be able to construct a VolumeMesh from the loaded data
    vol = pgo.mesh.VolumeMesh(loaded_geo, loaded_mat)
    assert vol.num_vertices == 4
    assert vol.num_elements == 1
