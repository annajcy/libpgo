import json

import numpy as np

import pypgo as pgo


def test_sparse_matrix_exports_numpy_coo():
    core = pgo._core.create_sparse_matrix(
        3,
        4,
        [2, 0, 2],
        [1, 3, 1],
        [5.0, 1.5, 2.5],
    )
    matrix = pgo.sparse.SparseMatrix(core)

    assert matrix.shape == (3, 4)
    assert matrix.nnz == 2
    rows, cols, values = matrix.to_coo()
    assert rows.dtype == np.int64
    assert cols.dtype == np.int64
    assert values.dtype == np.float64
    assert np.array_equal(rows, np.array([0, 2], dtype=np.int64))
    assert np.array_equal(cols, np.array([3, 1], dtype=np.int64))
    assert np.allclose(values, np.array([1.5, 7.5]))


def test_sparse_matrix_constructs_from_coo_helper():
    matrix = pgo.sparse.SparseMatrix.from_coo(
        (3, 4),
        [2, 0, 2],
        [1, 3, 1],
        [5.0, 1.5, 2.5],
    )

    assert matrix.shape == (3, 4)
    assert matrix.nnz == 2
    assert np.allclose(matrix.to_dense(), np.array(
        [
            [0.0, 0.0, 0.0, 1.5],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 7.5, 0.0, 0.0],
        ],
        dtype=np.float64,
    ))


def test_sparse_matmul_matches_dense_reference():
    rng = np.random.default_rng(0)
    dense = rng.standard_normal((5, 4))
    dense[dense < 0.5] = 0.0  # exercise the sparse (zeros-dropped) path
    rows, cols = np.nonzero(dense)
    matrix = pgo.sparse.SparseMatrix.from_coo((5, 4), rows, cols, dense[rows, cols])

    # matvec (1-D) and matmat (2-D) both delegate to the C++ multiply.
    vector = rng.standard_normal(4)
    block = rng.standard_normal((4, 3))
    assert np.allclose(matrix @ vector, dense @ vector)
    assert np.allclose(matrix @ block, dense @ block)

    # repeated multiply reuses the cached Eigen matrix.
    assert np.allclose(matrix @ vector, dense @ vector)

    # non-contiguous (strided) input is handled.
    assert np.allclose(matrix @ block[::1, ::1], dense @ block)


def test_sparse_matmul_rejects_shape_mismatch():
    matrix = pgo.sparse.SparseMatrix.from_coo((3, 4), [0, 2], [1, 3], [1.0, 2.0])
    for bad in (np.ones(3), np.ones((3, 2))):
        with np.testing.assert_raises(ValueError):
            matrix @ bad


def test_shell_spec_io_roundtrip(tmp_path):
    tri = pgo.mesh.TriMeshData(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64),
        np.array([[0, 1, 2]], dtype=np.int64),
    )
    mat = pgo.sim.KoiterStVKShellMaterial(
        "cloth", thickness=0.01, E_membrane=1e6, nu_membrane=0.35
    )

    shell_path = tmp_path / "cloth.shell.json"
    pgo.sim.write_shell(shell_path, tri, mat)
    loaded_tri, loaded_mat = pgo.sim.read_shell(shell_path)

    assert isinstance(loaded_tri, pgo.mesh.TriMeshData)
    assert np.allclose(loaded_tri.vertices, tri.vertices)
    assert np.array_equal(loaded_tri.elements, tri.elements)
    assert loaded_mat == mat

    payload = json.loads(shell_path.read_text())
    assert payload["mesh_obj"] == "cloth.obj"
