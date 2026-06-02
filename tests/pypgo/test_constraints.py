import gc

import numpy as np
import pytest

import pypgo
import pypgo.constraints as pc
import pypgo.energy as pe
import pypgo.sparse as ps


def _sparse(rows, cols, row_indices, col_indices, values):
    return ps.SparseMatrix(
        pypgo._core.create_sparse_matrix(rows, cols, row_indices, col_indices, values)
    )


def test_constraints_module_is_lazy_importable():
    assert "constraints" in pypgo.__all__
    assert pypgo.constraints.__name__ == "pypgo.constraints"


def test_linear_values_jacobian_and_hessian_match_numpy():
    A = _sparse(2, 3, [0, 0, 1], [0, 2, 1], [2.0, -1.0, 3.0])
    c = pc.Linear(A, offset=np.array([5.0, -4.0]))
    x = np.array([1.0, 2.0, 3.0])

    assert c.num_dofs == 3
    assert c.num_constraints == 2
    assert c.is_linear is True
    assert np.allclose(c.value(x), A @ x + np.array([5.0, -4.0]))
    assert np.allclose(c.jacobian(x).to_dense(), A.to_dense())

    H = c.hessian(x, np.ones(2))
    assert H.shape == (3, 3)
    assert H.nnz == 0


def test_constraint_function_set_concatenates_rows_and_owns_children():
    first = pc.Linear(_sparse(1, 3, [0], [0], [2.0]), offset=[1.0])
    second = pc.Linear(_sparse(2, 3, [0, 1], [1, 2], [3.0, 4.0]), offset=[5.0, 6.0])
    constraint_set = pc.ConstraintFunctionSet([first, second])

    del first, second
    gc.collect()

    x = np.array([10.0, 20.0, 30.0])
    assert constraint_set.num_dofs == 3
    assert constraint_set.num_constraints == 3
    assert np.allclose(constraint_set.value(x), [21.0, 65.0, 126.0])
    assert np.allclose(
        constraint_set.jacobian(x).to_dense(),
        np.array([[2.0, 0.0, 0.0], [0.0, 3.0, 0.0], [0.0, 0.0, 4.0]]),
    )


def test_bounded_broadcasts_scalar_bounds_and_validates_shapes():
    c = pc.Linear(_sparse(2, 3, [0, 1], [0, 2], [1.0, 1.0]), offset=[0.0, 0.0])
    bounded = pc.Bounded(c, lower=0.0, upper=np.array([1.0, 2.0]))

    assert bounded.functions is c
    assert np.allclose(bounded.lower, [0.0, 0.0])
    assert np.allclose(bounded.upper, [1.0, 2.0])

    with pytest.raises(ValueError, match="lower"):
        pc.Bounded(c, lower=np.zeros(3), upper=1.0)
    with pytest.raises(ValueError, match="upper"):
        pc.Bounded(c, lower=0.0, upper=np.zeros(3))


def test_linear_owns_python_sparse_and_offset_inputs():
    A = _sparse(1, 2, [0], [0], [2.0])
    offset = np.array([3.0])
    c = pc.Linear(A, offset=offset)

    offset[0] = 99.0
    del A, offset
    gc.collect()

    assert np.allclose(c.value(np.array([4.0, 0.0])), [11.0])


def test_constraint_penalty_matches_zero_residual_quadratic_and_energy_set():
    c = pc.Linear(_sparse(2, 2, [0, 1], [0, 1], [2.0, 3.0]), offset=[1.0, -1.0])
    penalty = pe.ConstraintPenalty(c, weight=4.0)
    x = np.array([1.0, 2.0])
    residual = c.value(x)

    assert penalty.num_dofs == 2
    assert penalty.value(x) == pytest.approx(0.5 * 4.0 * float(residual @ residual))
    assert np.allclose(penalty.gradient(x), 4.0 * c.jacobian(x).to_dense().T @ residual)

    total = pe.EnergySet([(penalty, 0.5)])
    assert total.value(x) == pytest.approx(0.5 * penalty.value(x))


def test_constraint_violation_penalty_supports_equality_and_bounds():
    c = pc.Linear(_sparse(3, 3, [0, 1, 2], [0, 1, 2], [1.0, 1.0, 1.0]))
    bounded = pc.Bounded(
        c,
        lower=np.array([2.0, 0.0, -1.0]),
        upper=np.array([2.0, np.inf, 1.0]),
    )
    penalty = pe.ConstraintViolationPenalty(bounded, weight=10.0)

    x = np.array([1.5, -0.5, 0.25], dtype=np.float64)
    # equality violation: x0 - 2 = -0.5
    # lower-bound violation: x1 - 0 = -0.5
    # interval satisfied: 0
    assert penalty.value(x) == pytest.approx(0.5 * 10.0 * (0.25 + 0.25))

    expected_grad = 10.0 * np.array([-0.5, -0.5, 0.0], dtype=np.float64)
    assert np.allclose(penalty.gradient(x), expected_grad)

    H = penalty.hessian(x).to_dense()
    assert np.allclose(H, np.diag([10.0, 10.0, 0.0]))

    total = pe.EnergySet([(penalty, 0.25)])
    assert total.value(x) == pytest.approx(0.25 * penalty.value(x))


def test_constraint_violation_penalty_raises_for_inverted_bounds():
    c = pc.Linear(_sparse(1, 1, [0], [0], [1.0]))
    bounded = pc.Bounded(c, lower=2.0, upper=1.0)

    with pytest.raises(ValueError, match="lower.*upper"):
        pe.ConstraintViolationPenalty(bounded)
