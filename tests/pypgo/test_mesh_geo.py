import numpy as np
import pytest
import pypgo as pgo


# --- TriCellMeshGeo ---

def test_tricellmesh_geo_constructs_from_numpy_arrays():
    vertices = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0]
    ], dtype=np.float64)
    cells = np.array([[0, 1, 2]], dtype=np.int64)

    geo = pgo.mesh_geo.TriCellMeshGeo(vertices, cells)
    assert geo.num_vertices == 3
    assert geo.num_cells == 1
    assert geo.cell_type == pgo.mesh_geo.CellMeshType.Triangle
    assert np.allclose(geo.vertices, vertices)
    assert np.array_equal(geo.cells, cells)

    # Modifying the original input arrays should not affect the geo
    vertices[0, 0] = 99.0
    cells[0, 0] = 99
    assert geo.vertices[0, 0] == 0.0
    assert geo.cells[0, 0] == 0


# --- TetCellMeshGeo ---

def test_tetcellmesh_geo_constructs_from_numpy_arrays():
    vertices = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)
    cells = np.array([[0, 1, 2, 3]], dtype=np.int64)

    geo = pgo.mesh_geo.TetCellMeshGeo(vertices, cells)
    assert geo.num_vertices == 4
    assert geo.num_cells == 1
    assert geo.cell_type == pgo.mesh_geo.CellMeshType.Tet
    assert np.allclose(geo.vertices, vertices)
    assert np.array_equal(geo.cells, cells)


# --- CubicCellMeshGeo ---

def test_cubiccellmesh_geo_constructs_from_numpy_arrays():
    vertices = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 1.0, 0.0],
        [1.0, 0.0, 1.0],
        [0.0, 1.0, 1.0],
        [1.0, 1.0, 1.0],
    ], dtype=np.float64)
    cells = np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64)

    geo = pgo.mesh_geo.CubicCellMeshGeo(vertices, cells)
    assert geo.num_vertices == 8
    assert geo.num_cells == 1
    assert geo.cell_type == pgo.mesh_geo.CellMeshType.Cubic
    assert np.allclose(geo.vertices, vertices)
    assert np.array_equal(geo.cells, cells)


# --- Validation ---

def test_mesh_geo_rejects_invalid_shapes_and_indices():
    vertices = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0]
    ], dtype=np.float64)

    # Cells with wrong number of columns (must be 3 for triangle)
    cells_wrong = np.array([[0, 1]], dtype=np.int64)
    with pytest.raises((ValueError, TypeError)):
        pgo.mesh_geo.TriCellMeshGeo(vertices, cells_wrong)

    # Indices out of bounds
    cells_out_of_bounds = np.array([[0, 1, 99]], dtype=np.int64)
    with pytest.raises((ValueError, TypeError)):
        pgo.mesh_geo.TriCellMeshGeo(vertices, cells_out_of_bounds)

    # Non-integer indices should raise TypeError or ValueError
    cells_float = np.array([[0.0, 1.0, 2.0]], dtype=np.float64)
    with pytest.raises((TypeError, ValueError)):
        pgo.mesh_geo.TriCellMeshGeo(vertices, cells_float)


# --- MaterialSpec ---

def test_material_spec_constructs_from_init():
    material = pgo.mesh.MaterialSpec(1e6, 0.33, 1200.0)
    assert material.E == 1e6
    assert material.nu == 0.33
    assert material.density == 1200.0


def test_material_spec_defaults():
    material = pgo.mesh.MaterialSpec()
    assert material.E == 1e9
    assert material.nu == 0.45
    assert material.density == 1000.0


# --- VolumeMesh ---

def test_volumemesh_constructs_from_tet_cellmesh():
    vertices = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)
    cells = np.array([[0, 1, 2, 3]], dtype=np.int64)

    tet_geo = pgo.mesh_geo.TetCellMeshGeo(vertices, cells)
    material = pgo.mesh.MaterialSpec(1e6, 0.33, 1200.0)
    vol = pgo.mesh.VolumeMesh(tet_geo, material)

    assert vol.num_vertices == 4
    assert vol.num_elements == 1
    assert vol.geometry is tet_geo
    assert vol.cell_mesh is tet_geo
    assert vol.material is material
    assert vol.material_spec is material


def test_volumemesh_constructs_from_cubic_cellmesh():
    vertices = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 1.0, 0.0],
        [1.0, 0.0, 1.0],
        [0.0, 1.0, 1.0],
        [1.0, 1.0, 1.0],
    ], dtype=np.float64)
    cells = np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64)

    cubic_geo = pgo.mesh_geo.CubicCellMeshGeo(vertices, cells)
    material = pgo.mesh.MaterialSpec(2e9, 0.3, 800.0)
    vol = pgo.mesh.VolumeMesh(cubic_geo, material)

    assert vol.num_vertices == 8
    assert vol.num_elements == 1


def test_volumemesh_load_and_save_roundtrip(tmp_path):
    vertices = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)
    cells = np.array([[0, 1, 2, 3]], dtype=np.int64)

    tet_geo = pgo.mesh_geo.TetCellMeshGeo(vertices, cells)
    material = pgo.mesh.MaterialSpec(2e9, 0.4, 900.0)
    vol = pgo.mesh.VolumeMesh(tet_geo, material)

    veg_file = str(tmp_path / "roundtrip.veg")
    vol.save(veg_file)

    loaded = pgo.mesh.VolumeMesh.load(veg_file)

    assert loaded.num_vertices == 4
    assert loaded.num_elements == 1
    assert isinstance(loaded.geometry, pgo.mesh_geo.TetCellMeshGeo)
    assert isinstance(loaded.cell_mesh, pgo.mesh_geo.TetCellMeshGeo)
    assert loaded.geometry.num_vertices == 4
    assert loaded.geometry.num_cells == 1
    assert np.allclose(loaded.geometry.vertices, vertices)
    assert np.array_equal(loaded.geometry.cells, cells)
    assert loaded.material.E == material.E
    assert loaded.material.nu == material.nu
    assert loaded.material.density == material.density
    assert loaded.material_spec.E == material.E


def test_volumemesh_rejects_tri_cellmesh():
    vertices = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0]
    ], dtype=np.float64)
    cells = np.array([[0, 1, 2]], dtype=np.int64)

    tri_geo = pgo.mesh_geo.TriCellMeshGeo(vertices, cells)
    material = pgo.mesh.MaterialSpec()

    with pytest.raises(TypeError, match="TetCellMeshGeo or CubicCellMeshGeo"):
        pgo.mesh.VolumeMesh(tri_geo, material)
