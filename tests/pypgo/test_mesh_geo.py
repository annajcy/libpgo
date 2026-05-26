import numpy as np
import pytest

import pypgo as pgo


def tri_vertices():
    return np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )


def tet_vertices():
    return np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def cubic_vertices():
    return np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 1.0, 0.0],
            [1.0, 0.0, 1.0],
            [0.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
        ],
        dtype=np.float64,
    )


def test_mesh_data_constructs_from_numpy_arrays():
    tri = pgo.mesh_geo.TriMeshData(tri_vertices(), np.array([[0, 1, 2]], dtype=np.int64))
    tet = pgo.mesh_geo.TetMeshData(tet_vertices(), np.array([[0, 1, 2, 3]], dtype=np.int64))
    cubic = pgo.mesh_geo.CubicMeshData(cubic_vertices(), np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64))

    assert tri.num_vertices == 3
    assert tri.num_elements == 1
    assert tri.mesh_type == pgo.mesh_geo.MeshDataType.Triangle
    assert np.array_equal(tri.elements, np.array([[0, 1, 2]], dtype=np.int64))

    assert tet.num_vertices == 4
    assert tet.num_elements == 1
    assert tet.mesh_type == pgo.mesh_geo.MeshDataType.Tet
    assert tet.element_vtx_id(0, 3) == 3

    assert cubic.num_vertices == 8
    assert cubic.num_elements == 1
    assert cubic.mesh_type == pgo.mesh_geo.MeshDataType.Cubic
    assert cubic.element_vtx_id(0, 7) == 7


def test_mesh_data_copies_numpy_inputs():
    vertices = tri_vertices()
    elements = np.array([[0, 1, 2]], dtype=np.int64)
    data = pgo.mesh_geo.TriMeshData(vertices, elements)

    vertices[0, 0] = 99.0
    elements[0, 0] = 99

    assert data.vertices[0, 0] == 0.0
    assert data.elements[0, 0] == 0


def test_mesh_data_rejects_invalid_shapes_and_indices():
    vertices = tri_vertices()

    with pytest.raises((ValueError, TypeError)):
        pgo.mesh_geo.TriMeshData(vertices, np.array([[0, 1]], dtype=np.int64))

    with pytest.raises((ValueError, TypeError)):
        pgo.mesh_geo.TriMeshData(vertices, np.array([[0, 1, 99]], dtype=np.int64))

    with pytest.raises((TypeError, ValueError)):
        pgo.mesh_geo.TriMeshData(vertices, np.array([[0.0, 1.0, 2.0]], dtype=np.float64))


def test_mesh_geo_facades_query_and_convert_to_mesh_data():
    tri = pgo.mesh_geo.TriMeshGeo(tri_vertices(), np.array([[0, 1, 2]], dtype=np.int64))
    tet = pgo.mesh_geo.TetMeshGeo(tet_vertices(), np.array([[0, 1, 2, 3]], dtype=np.int64))
    cubic = pgo.mesh_geo.CubicMeshGeo(cubic_vertices(), np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64))

    assert tri.num_triangles == 1
    assert np.array_equal(tri.triangles, np.array([[0, 1, 2]], dtype=np.int64))
    assert tri.tri_vtx_id(0, 2) == 2
    assert isinstance(tri.to_mesh_data(), pgo.mesh_geo.TriMeshData)

    assert tet.num_tets == 1
    assert np.array_equal(tet.tets, np.array([[0, 1, 2, 3]], dtype=np.int64))
    assert tet.tet_vtx_id(0, 3) == 3
    assert isinstance(tet.to_mesh_data(), pgo.mesh_geo.TetMeshData)

    assert cubic.num_cubes == 1
    assert np.array_equal(cubic.cubes, np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64))
    assert cubic.cube_vtx_id(0, 7) == 7
    assert isinstance(cubic.to_mesh_data(), pgo.mesh_geo.CubicMeshData)


def test_mesh_geo_constructs_from_mesh_data():
    tri_data = pgo.mesh_geo.TriMeshData(tri_vertices(), np.array([[0, 1, 2]], dtype=np.int64))
    tet_data = pgo.mesh_geo.TetMeshData(tet_vertices(), np.array([[0, 1, 2, 3]], dtype=np.int64))
    cubic_data = pgo.mesh_geo.CubicMeshData(cubic_vertices(), np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64))

    assert np.array_equal(pgo.mesh_geo.TriMeshGeo.from_mesh_data(tri_data).triangles, tri_data.elements)
    assert np.array_equal(pgo.mesh_geo.TetMeshGeo.from_mesh_data(tet_data).tets, tet_data.elements)
    assert np.array_equal(pgo.mesh_geo.CubicMeshGeo.from_mesh_data(cubic_data).cubes, cubic_data.elements)

    with pytest.raises(TypeError):
        pgo.mesh_geo.TetMeshGeo.from_mesh_data(tri_data)


def test_material_spec_constructs_from_init_and_defaults():
    material = pgo.mesh.MaterialSpec(1e6, 0.33, 1200.0)
    assert material.E == 1e6
    assert material.nu == 0.33
    assert material.density == 1200.0

    default_material = pgo.mesh.MaterialSpec()
    assert default_material.E == 1e9
    assert default_material.nu == 0.45
    assert default_material.density == 1000.0


def test_volumemesh_constructs_from_volume_mesh_data_only():
    tet_data = pgo.mesh_geo.TetMeshData(tet_vertices(), np.array([[0, 1, 2, 3]], dtype=np.int64))
    cubic_data = pgo.mesh_geo.CubicMeshData(cubic_vertices(), np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64))
    material = pgo.mesh.MaterialSpec(1e6, 0.33, 1200.0)

    tet_volume = pgo.mesh.VolumeMesh(tet_data, material)
    assert tet_volume.num_vertices == 4
    assert tet_volume.num_elements == 1
    assert tet_volume.mesh_data is tet_data
    assert tet_volume.geometry is tet_data
    assert tet_volume.material is material

    cubic_volume = pgo.mesh.VolumeMesh(cubic_data, material)
    assert cubic_volume.num_vertices == 8
    assert cubic_volume.num_elements == 1


def test_volumemesh_rejects_geo_and_tri_data():
    material = pgo.mesh.MaterialSpec()
    tri_data = pgo.mesh_geo.TriMeshData(tri_vertices(), np.array([[0, 1, 2]], dtype=np.int64))
    tet_geo = pgo.mesh_geo.TetMeshGeo(tet_vertices(), np.array([[0, 1, 2, 3]], dtype=np.int64))
    cubic_geo = pgo.mesh_geo.CubicMeshGeo(cubic_vertices(), np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64))

    with pytest.raises(TypeError, match="TetMeshData or CubicMeshData"):
        pgo.mesh.VolumeMesh(tri_data, material)
    with pytest.raises(TypeError, match="TetMeshData or CubicMeshData"):
        pgo.mesh.VolumeMesh(tet_geo, material)
    with pytest.raises(TypeError, match="TetMeshData or CubicMeshData"):
        pgo.mesh.VolumeMesh(cubic_geo, material)


def test_volumemesh_load_and_save_roundtrip(tmp_path):
    vertices = tet_vertices()
    elements = np.array([[0, 1, 2, 3]], dtype=np.int64)
    tet_data = pgo.mesh_geo.TetMeshData(vertices, elements)
    material = pgo.mesh.MaterialSpec(2e9, 0.4, 900.0)
    volume = pgo.mesh.VolumeMesh(tet_data, material)

    veg_file = str(tmp_path / "roundtrip.veg")
    volume.save(veg_file)
    loaded = pgo.mesh.VolumeMesh.load(veg_file)

    assert loaded.num_vertices == 4
    assert loaded.num_elements == 1
    assert isinstance(loaded.mesh_data, pgo.mesh_geo.TetMeshData)
    assert np.allclose(loaded.mesh_data.vertices, vertices)
    assert np.array_equal(loaded.mesh_data.elements, elements)
    assert loaded.material.E == material.E
    assert loaded.material.nu == material.nu
    assert loaded.material.density == material.density


def test_old_public_names_are_removed():
    assert not hasattr(pgo.mesh_geo, "TriCellMeshGeo")
    assert not hasattr(pgo.mesh_geo, "TetCellMeshGeo")
    assert not hasattr(pgo.mesh_geo, "CubicCellMeshGeo")
    assert not hasattr(pgo.mesh_geo, "CellMeshType")
