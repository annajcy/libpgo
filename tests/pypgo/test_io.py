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
    material = pgo.mesh.volume.ENuMaterial("rubber", E=1e6, nu=0.33, density=1200.0)

    veg_file = str(tmp_path / "temp_mesh.veg")
    pgo.mesh.volume.write_veg(veg_file, pgo.mesh.volume.VegFile.from_single_material(tet_data, material))
    loaded = pgo.mesh.volume.read_veg(veg_file)
    loaded_data = loaded.mesh_data
    loaded_material = loaded.first_material()

    assert isinstance(loaded_data, pgo.mesh.TetMeshData)
    assert loaded_data.num_vertices == 4
    assert loaded_data.num_elements == 1
    assert np.allclose(loaded_data.vertices, vertices)
    assert np.array_equal(loaded_data.elements, elements)

    assert isinstance(loaded_material, pgo.mesh.volume.ENuMaterial)
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
    material = pgo.mesh.volume.ENuMaterial("foam", E=2e9, nu=0.4, density=900.0)

    veg_file = str(tmp_path / "temp2.veg")
    pgo.mesh.volume.write_veg(veg_file, pgo.mesh.volume.VegFile.from_single_material(tet_data, material))
    veg = pgo.mesh.volume.read_veg(veg_file)

    volume = pgo.mesh.volume.VolumeMesh(veg)
    assert volume.num_vertices == 4
    assert volume.num_elements == 1


def test_veg_roundtrip_preserves_multiple_material_payloads(tmp_path):
    vertices = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
        ],
        dtype=np.float64,
    )
    elements = np.array([[0, 1, 2, 3], [1, 2, 3, 4]], dtype=np.int64)
    tet_data = pgo.mesh.TetMeshData(vertices, elements)
    veg = pgo.mesh.volume.VegFile(
        mesh_data=tet_data,
        materials=[
            pgo.mesh.volume.ENuMaterial("soft", density=1000.0, E=2e6, nu=0.35),
            pgo.mesh.volume.MooneyRivlinMaterial("insert", density=1200.0, mu01=3.0, mu10=4.0, v1=0.2),
            pgo.mesh.volume.OrthotropicMaterial(
                "unusedOrtho",
                density=800.0,
                E1=3e6,
                E2=2e6,
                E3=1e6,
                nu12=0.2,
                nu23=0.25,
                nu31=0.3,
                G12=0.7e6,
                G23=0.6e6,
                G31=0.5e6,
                rotation=np.array(
                    [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
                ),
            ),
        ],
        sets=[
            pgo.mesh.volume.MeshSet("allElements", [0, 1]),
            pgo.mesh.volume.MeshSet("softSet", [0]),
            pgo.mesh.volume.MeshSet("insertSet", [1]),
        ],
        regions=[
            pgo.mesh.volume.MeshRegion(0, 1),
            pgo.mesh.volume.MeshRegion(1, 2),
        ],
    )

    path = str(tmp_path / "multi.veg")
    pgo.mesh.volume.write_veg(path, veg)
    loaded = pgo.mesh.volume.read_veg(path)

    assert isinstance(loaded.mesh_data, pgo.mesh.TetMeshData)
    assert np.allclose(loaded.mesh_data.vertices, vertices)
    assert np.array_equal(loaded.mesh_data.elements, elements)
    assert [s.name for s in loaded.sets] == ["allElements", "softSet", "insertSet"]
    assert [s.elements for s in loaded.sets] == [[0, 1], [0], [1]]
    assert [(r.material_index, r.set_index) for r in loaded.regions] == [(0, 1), (1, 2)]
    assert isinstance(loaded.materials[0], pgo.mesh.volume.ENuMaterial)
    assert loaded.materials[0].E == pytest.approx(2e6)
    assert isinstance(loaded.materials[1], pgo.mesh.volume.MooneyRivlinMaterial)
    assert loaded.materials[1].mu10 == pytest.approx(4.0)
    assert isinstance(loaded.materials[2], pgo.mesh.volume.OrthotropicMaterial)
    assert loaded.materials[2].name == "unusedOrtho"
    assert np.array_equal(
        loaded.materials[2].rotation,
        np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
    )


def test_io_rejects_mesh_geo_facades(tmp_path):
    material = pgo.mesh.volume.ENuMaterial()
    tri_geo = pgo.mesh.geometry.TriMeshGeo(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64),
        np.array([[0, 1, 2]], dtype=np.int64),
    )
    tet_geo = pgo.mesh.geometry.TetMeshGeo(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )

    with pytest.raises(TypeError, match="TriMeshData"):
        pgo.mesh.write_obj(str(tmp_path / "surface.obj"), tri_geo)
    with pytest.raises(TypeError, match="TetMeshData or CubicMeshData"):
        pgo.mesh.volume.write_veg(
            str(tmp_path / "mesh.veg"),
            pgo.mesh.volume.VegFile.from_single_material(tet_geo, material),
        )


def test_old_io_module_is_not_public():
    assert not hasattr(pgo, "io")
    with pytest.raises(ModuleNotFoundError):
        __import__("pypgo.io")
