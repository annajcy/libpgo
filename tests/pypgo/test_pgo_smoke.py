"""Smoke test for source-build-tree and installed pypgo distributions."""

from __future__ import annotations

import numpy as np


def test_package_version_and_core_import():
    import pypgo

    assert pypgo.__version__ == "0.0.4"

    import pypgo._core as core

    info = core.build_info()
    assert info["module"] == "pypgo._core"
    assert info["binding"] == "nanobind"
    assert info["mesh_geo"] is True


def test_numpy_blas_lapack_runtime():
    matrix = np.arange(256 * 256, dtype=np.float64).reshape(256, 256)
    product = matrix @ matrix.T
    assert np.isfinite(product).all()

    solution = np.linalg.solve(
        np.array([[4.0, 1.0], [2.0, 3.0]]),
        np.array([1.0, 1.0]),
    )
    assert np.allclose(solution, [0.2, 0.2])


def test_native_mesh_and_sparse_round_trip():
    import pypgo._core as core

    vertices = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    tets = np.array([[0, 1, 2, 3]], dtype=np.int32)
    mesh_data = core.create_tet_mesh_data(vertices.ravel(), tets.ravel())
    material = core.PyMaterialSpec(1e6, 0.45, 1000.0)
    volume_mesh = core.create_volume_mesh(mesh_data, material)
    mass = core.compute_mass_matrix(volume_mesh, True)
    assert mass.nnz() > 0
    rows, cols, values = mass.to_coo()
    assert len(rows) == len(cols) == len(values) == mass.nnz()
    assert np.isfinite(np.asarray(values, dtype=np.float64)).all()
