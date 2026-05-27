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
    tet_data = pgo.mesh.TetMeshData(vertices, elements)
    material = pgo.mesh.veg.ENuMaterial("rubber", E=1e6, nu=0.33, density=1200.0)

    veg_file = str(tmp_path / "temp_mesh.veg")
    pgo.mesh.veg.write_veg(veg_file, pgo.mesh.veg.VegFile.from_single_material(tet_data, material))
    loaded = pgo.mesh.veg.read_veg(veg_file)
    loaded_data = loaded.mesh_data
    loaded_material = loaded.first_material()

    assert isinstance(loaded_data, pgo.mesh.TetMeshData)
    assert loaded_data.num_vertices == 4
    assert loaded_data.num_elements == 1
    assert np.allclose(loaded_data.vertices, vertices)
    assert np.array_equal(loaded_data.elements, elements)

    assert isinstance(loaded_material, pgo.mesh.veg.ENuMaterial)
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
    surface_data = pgo.mesh.TriMeshData(vertices, elements)

    obj_file = str(tmp_path / "temp_surface.obj")
    pgo.mesh.write_obj(obj_file, surface_data)
    loaded_surface = pgo.mesh.read_obj(obj_file)

    assert isinstance(loaded_surface, pgo.mesh.TriMeshData)
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
    tet_data = pgo.mesh.TetMeshData(vertices, elements)
    material = pgo.mesh.veg.ENuMaterial("foam", E=2e9, nu=0.4, density=900.0)

    veg_file = str(tmp_path / "temp2.veg")
    pgo.mesh.veg.write_veg(veg_file, pgo.mesh.veg.VegFile.from_single_material(tet_data, material))
    veg = pgo.mesh.veg.read_veg(veg_file)

    volume = pgo.mesh.veg.VolumeMesh(veg.mesh_data, regions=veg.to_volume_regions())
    assert volume.num_vertices == 4
    assert volume.num_elements == 1


def test_io_rejects_mesh_geo_facades(tmp_path):
    material = pgo.mesh.veg.ENuMaterial()
    tri_geo = pgo.mesh.geo.TriMeshGeo(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64),
        np.array([[0, 1, 2]], dtype=np.int64),
    )
    tet_geo = pgo.mesh.geo.TetMeshGeo(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )

    with pytest.raises(TypeError, match="TriMeshData"):
        pgo.mesh.write_obj(str(tmp_path / "surface.obj"), tri_geo)
    with pytest.raises(TypeError, match="TetMeshData or CubicMeshData"):
        pgo.mesh.veg.write_veg(
            str(tmp_path / "mesh.veg"),
            pgo.mesh.veg.VegFile.from_single_material(tet_geo, material),
        )


def test_old_io_module_is_not_public():
    assert not hasattr(pgo, "io")
    with pytest.raises(ModuleNotFoundError):
        __import__("pypgo.io")
