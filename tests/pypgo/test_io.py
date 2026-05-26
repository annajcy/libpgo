import numpy as np
import pytest

import pypgo as pgo


def test_veg_tet_io_roundtrip(tmp_path):
    vertices = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    elements = np.array([[0, 1, 2, 3]], dtype=np.int64)
    tet_data = pgo.mesh_geo.TetMeshData(vertices, elements)
    material = pgo.mesh.MaterialSpec(1e6, 0.33, 1200.0)

    veg_file = str(tmp_path / "temp_mesh.veg")
    pgo.io.write_veg_geo(veg_file, tet_data, material)
    loaded_data, loaded_material = pgo.io.read_veg_geo(veg_file)

    assert isinstance(loaded_data, pgo.mesh_geo.TetMeshData)
    assert loaded_data.num_vertices == 4
    assert loaded_data.num_elements == 1
    assert np.allclose(loaded_data.vertices, vertices)
    assert np.array_equal(loaded_data.elements, elements)

    assert isinstance(loaded_material, pgo.mesh.MaterialSpec)
    assert loaded_material.E == 1e6
    assert loaded_material.nu == 0.33
    assert loaded_material.density == 1200.0


def test_obj_io_roundtrip(tmp_path):
    vertices = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )
    elements = np.array([[0, 1, 2]], dtype=np.int64)
    surface_data = pgo.mesh_geo.TriMeshData(vertices, elements)

    obj_file = str(tmp_path / "temp_surface.obj")
    pgo.io.write_obj_geo(obj_file, surface_data)
    loaded_surface = pgo.io.read_obj_geo(obj_file)

    assert isinstance(loaded_surface, pgo.mesh_geo.TriMeshData)
    assert loaded_surface.num_vertices == 3
    assert loaded_surface.num_elements == 1
    assert np.allclose(loaded_surface.vertices, vertices)
    assert np.array_equal(loaded_surface.elements, elements)


def test_veg_roundtrip_then_volume_mesh(tmp_path):
    vertices = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    elements = np.array([[0, 1, 2, 3]], dtype=np.int64)
    tet_data = pgo.mesh_geo.TetMeshData(vertices, elements)
    material = pgo.mesh.MaterialSpec(2e9, 0.4, 900.0)

    veg_file = str(tmp_path / "temp2.veg")
    pgo.io.write_veg_geo(veg_file, tet_data, material)
    loaded_data, loaded_mat = pgo.io.read_veg_geo(veg_file)

    volume = pgo.mesh.VolumeMesh(loaded_data, loaded_mat)
    assert volume.num_vertices == 4
    assert volume.num_elements == 1


def test_io_rejects_mesh_geo_facades(tmp_path):
    material = pgo.mesh.MaterialSpec()
    tri_geo = pgo.mesh_geo.TriMeshGeo(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64),
        np.array([[0, 1, 2]], dtype=np.int64),
    )
    tet_geo = pgo.mesh_geo.TetMeshGeo(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )

    with pytest.raises(TypeError, match="TriMeshData"):
        pgo.io.write_obj_geo(str(tmp_path / "surface.obj"), tri_geo)
    with pytest.raises(TypeError, match="TetMeshData or CubicMeshData"):
        pgo.io.write_veg_geo(str(tmp_path / "mesh.veg"), tet_geo, material)
