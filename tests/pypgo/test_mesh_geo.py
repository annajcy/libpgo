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
    tri = pgo.mesh.TriMeshData(tri_vertices(), np.array([[0, 1, 2]], dtype=np.int64))
    tet = pgo.mesh.TetMeshData(tet_vertices(), np.array([[0, 1, 2, 3]], dtype=np.int64))
    cubic = pgo.mesh.CubicMeshData(cubic_vertices(), np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64))

    assert tri.num_vertices == 3
    assert tri.num_elements == 1
    assert tri.mesh_type == pgo.mesh.MeshDataType.Triangle
    assert np.array_equal(tri.elements, np.array([[0, 1, 2]], dtype=np.int64))

    assert tet.num_vertices == 4
    assert tet.num_elements == 1
    assert tet.mesh_type == pgo.mesh.MeshDataType.Tet
    assert tet.element_vtx_id(0, 3) == 3

    assert cubic.num_vertices == 8
    assert cubic.num_elements == 1
    assert cubic.mesh_type == pgo.mesh.MeshDataType.Cubic
    assert cubic.element_vtx_id(0, 7) == 7


def test_mesh_data_copies_numpy_inputs():
    vertices = tri_vertices()
    elements = np.array([[0, 1, 2]], dtype=np.int64)
    data = pgo.mesh.TriMeshData(vertices, elements)

    vertices[0, 0] = 99.0
    elements[0, 0] = 99

    assert data.vertices[0, 0] == 0.0
    assert data.elements[0, 0] == 0


def test_mesh_data_rejects_invalid_shapes_and_indices():
    vertices = tri_vertices()

    with pytest.raises((ValueError, TypeError)):
        pgo.mesh.TriMeshData(vertices, np.array([[0, 1]], dtype=np.int64))

    with pytest.raises((ValueError, TypeError)):
        pgo.mesh.TriMeshData(vertices, np.array([[0, 1, 99]], dtype=np.int64))

    with pytest.raises((TypeError, ValueError)):
        pgo.mesh.TriMeshData(vertices, np.array([[0.0, 1.0, 2.0]], dtype=np.float64))


def test_mesh_geo_facades_query_and_convert_to_mesh_data():
    tri = pgo.mesh.geometry.TriMeshGeo(tri_vertices(), np.array([[0, 1, 2]], dtype=np.int64))
    tet = pgo.mesh.geometry.TetMeshGeo(tet_vertices(), np.array([[0, 1, 2, 3]], dtype=np.int64))
    cubic = pgo.mesh.geometry.CubicMeshGeo(cubic_vertices(), np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64))

    assert tri.num_triangles == 1
    assert np.array_equal(tri.triangles, np.array([[0, 1, 2]], dtype=np.int64))
    assert tri.tri_vtx_id(0, 2) == 2
    assert isinstance(tri.to_mesh_data(), pgo.mesh.TriMeshData)

    assert tet.num_tets == 1
    assert np.array_equal(tet.tets, np.array([[0, 1, 2, 3]], dtype=np.int64))
    assert tet.tet_vtx_id(0, 3) == 3
    assert isinstance(tet.to_mesh_data(), pgo.mesh.TetMeshData)

    assert cubic.num_cubes == 1
    assert np.array_equal(cubic.cubes, np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64))
    assert cubic.cube_vtx_id(0, 7) == 7
    assert isinstance(cubic.to_mesh_data(), pgo.mesh.CubicMeshData)


def test_mesh_geo_constructs_from_mesh_data():
    tri_data = pgo.mesh.TriMeshData(tri_vertices(), np.array([[0, 1, 2]], dtype=np.int64))
    tet_data = pgo.mesh.TetMeshData(tet_vertices(), np.array([[0, 1, 2, 3]], dtype=np.int64))
    cubic_data = pgo.mesh.CubicMeshData(cubic_vertices(), np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64))

    assert np.array_equal(pgo.mesh.geometry.TriMeshGeo.from_mesh_data(tri_data).triangles, tri_data.elements)
    assert np.array_equal(pgo.mesh.geometry.TetMeshGeo.from_mesh_data(tet_data).tets, tet_data.elements)
    assert np.array_equal(pgo.mesh.geometry.CubicMeshGeo.from_mesh_data(cubic_data).cubes, cubic_data.elements)

    with pytest.raises(TypeError):
        pgo.mesh.geometry.TetMeshGeo.from_mesh_data(tri_data)


def test_material_spec_constructs_from_init_and_defaults():
    material = pgo.mesh.volume.ENuMaterial(E=1e6, nu=0.33, density=1200.0)
    assert material.E == 1e6
    assert material.nu == 0.33
    assert material.density == 1200.0

    default_material = pgo.mesh.volume.ENuMaterial()
    assert default_material.E == 1e9
    assert default_material.nu == 0.45
    assert default_material.density == 1000.0


def test_volumemesh_constructs_from_volume_mesh_data_only():
    tet_data = pgo.mesh.TetMeshData(tet_vertices(), np.array([[0, 1, 2, 3]], dtype=np.int64))
    cubic_data = pgo.mesh.CubicMeshData(cubic_vertices(), np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64))
    material = pgo.mesh.volume.ENuMaterial("rubber", E=1e6, nu=0.33, density=1200.0)

    tet_volume = pgo.mesh.volume.VolumeMesh.create_from_single_material(tet_data, material)
    assert tet_volume.num_vertices == 4
    assert tet_volume.num_elements == 1
    assert tet_volume.mesh_data is tet_data
    assert tet_volume.geometry is tet_data
    assert tet_volume.material == material

    cubic_volume = pgo.mesh.volume.VolumeMesh.create_from_single_material(cubic_data, material)
    assert cubic_volume.num_vertices == 8
    assert cubic_volume.num_elements == 1


def test_volumemesh_rejects_geo_and_tri_data():
    material = pgo.mesh.volume.ENuMaterial()
    tri_data = pgo.mesh.TriMeshData(tri_vertices(), np.array([[0, 1, 2]], dtype=np.int64))
    tet_geo = pgo.mesh.geometry.TetMeshGeo(tet_vertices(), np.array([[0, 1, 2, 3]], dtype=np.int64))
    cubic_geo = pgo.mesh.geometry.CubicMeshGeo(cubic_vertices(), np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64))

    with pytest.raises(TypeError, match="TetMeshData or CubicMeshData"):
        pgo.mesh.volume.VolumeMesh.create_from_single_material(tri_data, material)
    with pytest.raises(TypeError, match="TetMeshData or CubicMeshData"):
        pgo.mesh.volume.VolumeMesh.create_from_single_material(tet_geo, material)
    with pytest.raises(TypeError, match="TetMeshData or CubicMeshData"):
        pgo.mesh.volume.VolumeMesh.create_from_single_material(cubic_geo, material)


def test_volumemesh_to_veg_file_and_back_roundtrip(tmp_path):
    vertices = tet_vertices()
    elements = np.array([[0, 1, 2, 3]], dtype=np.int64)
    tet_data = pgo.mesh.TetMeshData(vertices, elements)
    material = pgo.mesh.volume.ENuMaterial("foam", E=2e9, nu=0.4, density=900.0)
    volume = pgo.mesh.volume.VolumeMesh.create_from_single_material(tet_data, material)

    veg_file = str(tmp_path / "roundtrip.veg")
    veg = volume.to_veg_file()
    pgo.mesh.volume.write_veg(veg_file, veg)

    loaded_veg = pgo.mesh.volume.read_veg(veg_file)
    loaded = pgo.mesh.volume.VolumeMesh.from_veg_file(loaded_veg)

    assert loaded.num_vertices == 4
    assert loaded.num_elements == 1
    assert isinstance(loaded.mesh_data, pgo.mesh.TetMeshData)
    assert np.allclose(loaded.mesh_data.vertices, vertices)
    assert np.array_equal(loaded.mesh_data.elements, elements)
    assert loaded.material == material

    # Direct to_veg_file → from_veg_file roundtrip (no file I/O).
    direct = pgo.mesh.volume.VolumeMesh.from_veg_file(volume.to_veg_file())
    assert direct.num_vertices == 4
    assert direct.material == material


def test_old_public_names_are_removed():
    with pytest.raises(ModuleNotFoundError):
        __import__("pypgo.mesh_geo")
    assert not hasattr(pgo, "io")


def test_volume_mesh_regions_validate_partition():
    vertices = np.vstack([tet_vertices(), [[1.0, 1.0, 1.0]]])
    elements = np.array([[0, 1, 2, 3], [1, 2, 3, 4]], dtype=np.int64)
    tet_data = pgo.mesh.TetMeshData(vertices, elements)

    regions = [
        ("soft", pgo.mesh.volume.ENuMaterial("soft"), [0]),
        ("stiff", pgo.mesh.volume.MooneyRivlinMaterial("stiff", mu01=1.0), [1]),
    ]
    volume = pgo.mesh.volume.VolumeMesh(tet_data, regions)
    assert volume.num_elements == 2

    with pytest.raises(ValueError, match="assigned to both"):
        pgo.mesh.volume.VolumeMesh(tet_data, regions=[
            ("a", pgo.mesh.volume.ENuMaterial("a"), [0]),
            ("b", pgo.mesh.volume.ENuMaterial("b"), [0, 1]),
        ])

    with pytest.raises(ValueError, match="not assigned"):
        pgo.mesh.volume.VolumeMesh(tet_data, regions=[
            ("a", pgo.mesh.volume.ENuMaterial("a"), [0]),
        ])

    with pytest.raises(ValueError, match="duplicate region name"):
        pgo.mesh.volume.VolumeMesh(tet_data, regions=[
            ("a", pgo.mesh.volume.ENuMaterial("a"), [0]),
            ("a", pgo.mesh.volume.ENuMaterial("b"), [1]),
        ])


def test_material_dataclasses_are_python_payloads():
    mat = pgo.mesh.volume.ENuMaterial("cloth", E=1e6, nu=0.25, density=10.0)
    assert not hasattr(mat, "_handle")
    assert mat.lam == pytest.approx(4e5)
    assert mat.mu == pytest.approx(4e5)
