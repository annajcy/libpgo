import numpy as np
import pytest

import pypgo as pgo
from pypgo.tools.mesh import QualityReport, check_surface_quality


def standard_cube_vertices():
    return np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ],
        dtype=np.float64,
    )


def test_mesh_data_bbox_take_elements_and_concatenate():
    vertices = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [2.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    elements = np.array([[0, 1, 2], [0, 3, 1], [1, 4, 2]], dtype=np.int64)
    tri = pgo.mesh.TriMeshData(vertices, elements)

    bmin, bmax = tri.bbox
    assert bmin.shape == (3,)
    assert bmax.shape == (3,)
    assert np.allclose(bmin, [0.0, 0.0, 0.0])
    assert np.allclose(bmax, [2.0, 1.0, 1.0])

    subset = tri.take_elements([0, 2])
    assert isinstance(subset, pgo.mesh.TriMeshData)
    assert subset.num_elements == 2
    assert np.array_equal(subset.vertices, tri.vertices)
    assert np.array_equal(subset.elements, elements[[0, 2]])

    shifted = pgo.mesh.TriMeshData(vertices + [10.0, 0.0, 0.0], elements[:1])
    merged = pgo.mesh.TriMeshData.concatenate([tri, shifted])
    assert merged.num_vertices == tri.num_vertices + shifted.num_vertices
    assert merged.num_elements == tri.num_elements + shifted.num_elements
    assert np.array_equal(merged.elements[-1], shifted.elements[0] + tri.num_vertices)


def test_volume_mesh_data_volume_and_center_of_mass():
    tet = pgo.mesh.TetMeshData(
        np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )
    assert tet.volume == pytest.approx(1.0 / 6.0)
    assert np.allclose(tet.center_of_mass, [0.25, 0.25, 0.25])

    cube = pgo.mesh.CubicMeshData(standard_cube_vertices(), np.array([[0, 1, 2, 3, 4, 5, 6, 7]]))
    assert cube.volume == pytest.approx(1.0)
    assert np.allclose(cube.center_of_mass, [0.5, 0.5, 0.5])


def test_tri_mesh_geo_area_and_normals():
    tri = pgo.mesh.geo.TriMeshGeo(
        np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int64),
    )

    assert tri.face_areas.shape == (2,)
    assert np.allclose(tri.face_areas, [0.5, 0.5])
    assert tri.face_normals.shape == (2, 3)
    assert np.allclose(tri.face_normals, [[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]])
    assert tri.vertex_normals.shape == (4, 3)
    assert np.allclose(tri.vertex_normals, np.tile([0.0, 0.0, 1.0], (4, 1)))


def test_surface_quality_report_clean_and_degenerate_meshes():
    clean = pgo.mesh.TriMeshData(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64),
        np.array([[0, 1, 2]], dtype=np.int64),
    )
    clean_report = check_surface_quality(clean, short_edge_threshold=1e-6)
    assert isinstance(clean_report, QualityReport)
    assert clean_report.is_clean
    assert clean_report.degenerate_tris == []
    assert clean_report.short_edges == []
    assert not clean_report.has_self_intersections

    bad = pgo.mesh.TriMeshData(
        np.array([[0.0, 0.0, 0.0], [1e-8, 0.0, 0.0], [2e-8, 0.0, 0.0]], dtype=np.float64),
        np.array([[0, 1, 2]], dtype=np.int64),
    )
    bad_report = check_surface_quality(bad, short_edge_threshold=1e-6)
    assert not bad_report.is_clean
    assert bad_report.degenerate_tris == [0]
    assert (0, 1) in bad_report.short_edges


def test_surface_quality_detects_non_manifold_flipped_and_intersections():
    non_manifold = pgo.mesh.TriMeshData(
        np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.0, 0.0, -1.0],
            ],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2], [1, 0, 3], [0, 1, 4]], dtype=np.int64),
    )
    report = check_surface_quality(non_manifold)
    assert (0, 1) in report.non_manifold_edges

    flipped = pgo.mesh.TriMeshData(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]]),
        np.array([[0, 1, 2], [0, 2, 3], [2, 0, 3]], dtype=np.int64),
    )
    assert check_surface_quality(flipped).flipped_tris == [2]

    crossing = pgo.mesh.TriMeshData(
        np.array(
            [
                [0.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [0.0, 2.0, 0.0],
                [0.5, -0.25, -0.5],
                [0.5, 1.25, -0.5],
                [0.5, 0.5, 0.5],
            ],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2], [3, 4, 5]], dtype=np.int64),
    )
    assert check_surface_quality(crossing).has_self_intersections
    assert pgo._core.check_self_intersections(crossing._core_obj)


def test_volume_mesh_extract_surface_mesh_for_tet_and_cubic():
    tet = pgo.mesh.TetMeshData(
        np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )
    material = pgo.mesh.veg.ENuMaterial()
    tet_surface = pgo.mesh.veg.VolumeMesh.create_from_single_material(tet, material).extract_surface_mesh()
    assert isinstance(tet_surface, pgo.mesh.TriMeshData)
    assert tet_surface.num_vertices == 4
    assert tet_surface.num_elements == 4

    cube = pgo.mesh.CubicMeshData(standard_cube_vertices(), np.array([[0, 1, 2, 3, 4, 5, 6, 7]]))
    cubic_surface = pgo.mesh.veg.VolumeMesh.create_from_single_material(cube, material).extract_surface_mesh()
    assert isinstance(cubic_surface, pgo.mesh.TriMeshData)
    assert cubic_surface.num_vertices == 8
    assert cubic_surface.num_elements == 12


def test_shape_factories_return_triangle_mesh_data():
    box = pgo.mesh.create_box(bmin=(0.0, 0.0, 0.0), bmax=(1.0, 2.0, 3.0))
    assert isinstance(box, pgo.mesh.TriMeshData)
    assert box.num_vertices == 8
    assert box.num_elements == 12
    assert np.allclose(box.bbox[1], [1.0, 2.0, 3.0])

    sphere = pgo.mesh.create_sphere(radius=1.0, axis_subdiv=8, height_subdiv=4)
    cylinder = pgo.mesh.create_cylinder(radius=1.0, height=2.0, axis_subdiv=8, height_subdiv=2)
    torus = pgo.mesh.create_torus(radial_res=8, tubular_res=6, radius=1.0, thickness=0.25)

    for mesh in (sphere, cylinder, torus):
        assert isinstance(mesh, pgo.mesh.TriMeshData)
        assert mesh.num_vertices > 0
        assert mesh.num_elements > 0


def test_barycentric_embedding_matrix_and_deform():
    tet = pgo.mesh.TetMeshData(
        np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )
    volume = pgo.mesh.veg.VolumeMesh.create_from_single_material(tet, pgo.mesh.veg.ENuMaterial())
    embedding = pgo.mesh.geo.BarycentricEmbedding(np.array([[0.25, 0.25, 0.25]], dtype=np.float64), volume)

    matrix = embedding.interpolation_matrix
    assert isinstance(matrix, pgo.sparse.SparseMatrix)
    assert matrix.shape == (3, 12)
    rows, cols, values = matrix.to_coo()
    assert matrix.nnz == len(values)
    assert rows.dtype == np.int64
    assert cols.dtype == np.int64
    assert values.dtype == np.float64

    alias_rows, alias_cols, alias_values = embedding.interpolation_matrix_coo()
    assert np.array_equal(alias_rows, rows)
    assert np.array_equal(alias_cols, cols)
    assert np.allclose(alias_values, values)

    volume_disp = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    ).ravel()
    assert np.allclose(embedding.deform(volume_disp), [0.25, 0.25, 0.25])


def test_surface_embedding_deforms_surface_from_volume_displacement():
    tet = pgo.mesh.TetMeshData(
        np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )
    volume = pgo.mesh.veg.VolumeMesh.create_from_single_material(tet, pgo.mesh.veg.ENuMaterial())
    surface = pgo.mesh.TriMeshData(
        np.array(
            [
                [0.25, 0.25, 0.25],
                [0.50, 0.25, 0.25],
                [0.25, 0.50, 0.25],
            ],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2]], dtype=np.int64),
    )

    embedding = pgo.mesh.SurfaceEmbedding(surface, volume)
    volume_disp = tet.vertices.copy()

    assert embedding.rest_surface is surface
    assert embedding.interpolation_matrix.shape == (3 * surface.num_vertices, 3 * tet.num_vertices)
    assert np.allclose(embedding.displacement(volume_disp), surface.vertices)

    deformed = embedding.deform(volume_disp.ravel())
    assert isinstance(deformed, pgo.mesh.TriMeshData)
    assert np.array_equal(deformed.elements, surface.elements)
    assert np.allclose(deformed.vertices, 2.0 * surface.vertices)

    with pytest.raises(ValueError, match="volume_displacement"):
        embedding.deform(np.zeros(5, dtype=np.float64))
