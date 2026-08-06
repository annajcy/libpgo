"""Tests for the public ``SparseMatrix.solve`` sparse linear solver."""

import numpy as np
import pytest

import pypgo.solver as solver
import pypgo.sparse as sparse


def test_sparse_solve_matches_dense():
    rng = np.random.default_rng(123)
    a = rng.normal(size=(15, 15))
    a = a @ a.T + 5.0 * np.eye(15)
    matrix = sparse.as_sparse_matrix(a)
    rhs = rng.normal(size=15)

    x = matrix.solve(rhs)
    np.testing.assert_allclose(a @ x, rhs, atol=1e-12)

    x_ldlt = matrix.solve(rhs, backend=solver.EigenLDLT())
    np.testing.assert_allclose(x, x_ldlt, atol=1e-12)

    x_auto = matrix.solve(rhs, backend=solver.Auto())
    np.testing.assert_allclose(a @ x_auto, rhs, atol=1e-12)


def test_sparse_solve_preserves_free_block_identity_mask():
    """The fixed-row identity mask used by equilibrium adjoints must solve K_ff."""
    rng = np.random.default_rng(7)
    n = 10
    a = rng.normal(size=(n, n))
    a = a @ a.T + n * np.eye(n)
    fixed = np.array([1, 4, 7], dtype=np.int64)
    free = np.setdiff1d(np.arange(n), fixed)
    rhs = rng.normal(size=n)

    rows, cols, values = sparse.as_sparse_matrix(a).to_coo()
    values = values.copy()
    mask = np.zeros(n, dtype=bool)
    mask[fixed] = True
    values[mask[rows] | mask[cols]] = 0.0
    masked = sparse.SparseMatrix.from_coo(
        (n, n),
        np.concatenate([rows, fixed]),
        np.concatenate([cols, fixed]),
        np.concatenate([values, np.ones(fixed.size)]),
    )
    rhs2 = rhs.copy()
    rhs2[fixed] = 0.0
    lam = masked.solve(rhs2, backend=solver.EigenLDLT())

    expected = np.zeros(n)
    expected[free] = np.linalg.solve(a[np.ix_(free, free)], rhs[free])
    np.testing.assert_allclose(lam, expected, atol=1e-10)


def test_sparse_solve_rejects_bad_inputs():
    matrix = sparse.as_sparse_matrix(np.eye(4))
    with pytest.raises(ValueError, match="rhs size"):
        matrix.solve(np.zeros(3))
    with pytest.raises(ValueError, match="1-D"):
        matrix.solve(np.zeros((4, 2)))
    with pytest.raises(TypeError, match="backend"):
        matrix.solve(np.zeros(4), backend="eigen")

    rectangular = sparse.as_sparse_matrix(np.ones((3, 4)))
    with pytest.raises(ValueError, match="square"):
        rectangular.solve(np.zeros(3))


def test_sparse_solve_reports_singular_factorization():
    singular = sparse.as_sparse_matrix(
        np.array([[1.0, 0.0], [0.0, 0.0]], dtype=np.float64))
    with pytest.raises(RuntimeError, match="factorization"):
        singular.solve(np.ones(2), backend=solver.EigenLDLT())
