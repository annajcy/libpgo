"""Tests for pypgo.energy types — PotentialEnergy handle, Linear/Quadratic,
EnergySet, state_kind, and zero_state.

Task E4: PotentialEnergy handle basic methods.
Task E5: LinearEnergy / QuadraticEnergy bindings.
"""

import gc

import numpy as np
import pypgo._core as _core
import pypgo.energy as pe
import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_quadratic_energy(n=6):
    """Return a PotentialEnergy wrapping a simple diagonal quadratic.

    1/2 x^T diag(2, 3, 4, 5, 6, 7) x  →  value(ones) = (2+3+4+5+6+7)/2 = 13.5
    gradient(x) = diag(2,3,4,5,6,7) x
    """
    rows = n
    cols = n
    row_indices = list(range(n))
    col_indices = list(range(n))
    values = [float(i + 2) for i in range(n)]  # diag 2, 3, 4, 5, 6, 7

    return _core._create_quadratic_energy_for_test(
        rows, cols, row_indices, col_indices, values,
    )


# ---------------------------------------------------------------------------
# PotentialEnergy handle basic methods
# ---------------------------------------------------------------------------

class TestPotentialEnergyHandleBasicMethods:
    def test_num_dofs(self):
        e = _make_quadratic_energy(6)
        assert e.num_dofs == 6

    def test_dofs_returns_int64_ndarray(self):
        e = _make_quadratic_energy(4)
        d = e.dofs()
        assert isinstance(d, np.ndarray)
        assert d.dtype == np.int64
        assert d.shape == (4,)
        assert list(d) == [0, 1, 2, 3]

    def test_state_kind_is_generic(self):
        e = _make_quadratic_energy(3)
        # Quadratic energy is generic, not displacement
        assert e.state_kind == "generic"

    def test_zero_state_returns_zeros(self):
        e = _make_quadratic_energy(5)
        z = e.zero_state()
        assert isinstance(z, np.ndarray)
        assert z.dtype == np.float64
        assert z.shape == (5,)
        assert np.all(z == 0.0)

    def test_value_at_ones(self):
        e = _make_quadratic_energy(6)
        x = np.ones(6, dtype=np.float64)
        # A = diag(2,3,4,5,6,7) → 1/2 x^T A x = (2+3+4+5+6+7)/2 = 13.5
        assert e.value(x) == pytest.approx(13.5, rel=1e-12)

    def test_value_at_zero(self):
        e = _make_quadratic_energy(6)
        x = np.zeros(6, dtype=np.float64)
        assert e.value(x) == pytest.approx(0.0, abs=1e-15)

    def test_gradient_at_ones(self):
        e = _make_quadratic_energy(6)
        x = np.ones(6, dtype=np.float64)
        g = e.gradient(x)
        assert isinstance(g, np.ndarray)
        assert g.dtype == np.float64
        assert g.shape == (6,)
        # diag(2,3,4,5,6,7) * ones = [2,3,4,5,6,7]
        expected = np.array([2, 3, 4, 5, 6, 7], dtype=np.float64)
        assert np.allclose(g, expected)

    def test_gradient_at_zero(self):
        e = _make_quadratic_energy(6)
        x = np.zeros(6, dtype=np.float64)
        g = e.gradient(x)
        assert np.all(g == 0.0)

    def test_hessian_is_sparse_matrix(self):
        e = _make_quadratic_energy(6)
        x = np.ones(6, dtype=np.float64)
        H = e.hessian(x)
        assert H.rows() == 6
        assert H.cols() == 6
        assert H.nnz() == 6  # diagonal
        rows, cols, vals = H.to_coo()
        assert len(vals) == 6
        for i in range(6):
            # diagonal entries 2,3,4,5,6,7
            assert np.isclose(vals[i], float(i + 2))

    def test_hessian_constant_for_quadratic(self):
        e = _make_quadratic_energy(6)
        x1 = np.zeros(6, dtype=np.float64)
        x2 = np.ones(6, dtype=np.float64)

        _, _, vals1 = e.hessian(x1).to_coo()
        _, _, vals2 = e.hessian(x2).to_coo()
        assert np.allclose(vals1, vals2)

    def test_max_step_default_unconstrained(self):
        e = _make_quadratic_energy(4)
        x = np.zeros(4, dtype=np.float64)
        dx = np.ones(4, dtype=np.float64)
        result = e.max_step(x, dx)
        assert result.alpha == 1.0
        assert result.clamped == False

    def test_repr(self):
        e = _make_quadratic_energy(6)
        r = repr(e)
        assert "PotentialEnergy" in r
        assert "6 DOFs" in r

    def test_wrong_input_size_raises(self):
        e = _make_quadratic_energy(6)
        x = np.ones(3, dtype=np.float64)
        with pytest.raises(ValueError, match="State size mismatch"):
            e.value(x)
        with pytest.raises(ValueError, match="State size mismatch"):
            e.gradient(x)
        with pytest.raises(ValueError, match="State size mismatch"):
            e.hessian(x)

    def test_gradient_is_contiguous(self):
        e = _make_quadratic_energy(6)
        x = np.ones(6, dtype=np.float64)
        g = e.gradient(x)
        assert g.flags["C_CONTIGUOUS"]

    def test_zero_state_is_contiguous(self):
        e = _make_quadratic_energy(6)
        z = e.zero_state()
        assert z.flags["C_CONTIGUOUS"]


# ---------------------------------------------------------------------------
# Lifetime: energy survives input array deletion
# ---------------------------------------------------------------------------

class TestPotentialEnergyLifetime:
    def test_quadratic_energy_owns_A_data(self):
        """After factory input data is freed, energy still evaluates correctly."""
        rows = 3
        cols = 3
        row_indices = [0, 1, 2]
        col_indices = [0, 1, 2]
        values = [10.0, 20.0, 30.0]

        e = _core._create_quadratic_energy_for_test(
            rows, cols, row_indices, col_indices, values,
        )

        # Delete the Python lists
        del row_indices, col_indices, values
        gc.collect()

        x = np.ones(3, dtype=np.float64)
        # value = (10 + 20 + 30) / 2 = 30
        assert e.value(x) == pytest.approx(30.0, rel=1e-12)
        g = e.gradient(x)
        assert np.allclose(g, [10, 20, 30])

    def test_energy_survives_wrapper_deletion(self):
        """After Python wrapper is deleted, another reference still works."""
        e1 = _make_quadratic_energy(4)
        e2 = e1  # second reference
        x = np.ones(4, dtype=np.float64)
        val1 = e1.value(x)
        del e1
        gc.collect()
        val2 = e2.value(x)
        assert val1 == pytest.approx(val2, rel=1e-15)


# ---------------------------------------------------------------------------
# Task E5: LinearEnergy
# ---------------------------------------------------------------------------

class TestLinearEnergy:
    def test_construct_and_num_dofs(self):
        b = np.array([1.0, 2.0, 3.0], dtype=np.float64)
        e = pe.LinearEnergy(b)
        assert e.num_dofs == 3
        assert e.state_kind == "generic"

    def test_value_is_b_dot_x(self):
        b = np.array([1.0, 2.0, 3.0], dtype=np.float64)
        e = pe.LinearEnergy(b)
        x = np.array([4.0, 5.0, 6.0], dtype=np.float64)
        # b^T x = 1*4 + 2*5 + 3*6 = 32
        assert e.value(x) == pytest.approx(32.0, rel=1e-12)

    def test_gradient_is_b(self):
        b = np.array([1.0, -2.0, 0.5], dtype=np.float64)
        e = pe.LinearEnergy(b)
        x = np.array([100.0, 200.0, 300.0], dtype=np.float64)
        g = e.gradient(x)
        assert np.allclose(g, b)

    def test_hessian_is_empty(self):
        b = np.array([1.0, 2.0], dtype=np.float64)
        e = pe.LinearEnergy(b)
        H = e.hessian(np.array([0.0, 0.0], dtype=np.float64))
        assert H.nnz == 0
        # Linear energy Hessian is identically zero (empty 0×0 sparse matrix)

    def test_zero_state(self):
        e = pe.LinearEnergy(np.ones(4, dtype=np.float64))
        z = e.zero_state()
        assert isinstance(z, np.ndarray)
        assert z.shape == (4,)
        assert np.all(z == 0.0)

    def test_repr(self):
        e = pe.LinearEnergy(np.ones(6, dtype=np.float64))
        r = repr(e)
        assert "LinearEnergy" in r
        assert "6 DOFs" in r

    def test_owns_b_data(self):
        b = np.array([5.0, 10.0, 15.0], dtype=np.float64)
        e = pe.LinearEnergy(b)
        # mutate and delete original
        b[0] = 999.0
        del b
        gc.collect()
        x = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        assert e.value(x) == pytest.approx(5.0, rel=1e-12)

    def test_wrong_shape_b_raises(self):
        with pytest.raises(ValueError):
            pe.LinearEnergy(np.ones((2, 3), dtype=np.float64))

    def test_wrong_input_size_raises(self):
        e = pe.LinearEnergy(np.ones(4, dtype=np.float64))
        with pytest.raises(ValueError, match="State size mismatch"):
            e.value(np.ones(3, dtype=np.float64))


# ---------------------------------------------------------------------------
# Task E5: QuadraticEnergy
# ---------------------------------------------------------------------------

class TestQuadraticEnergy:
    def test_construct_from_coo_tuple(self):
        # A = diag(2, 3), so 1/2 x^T A x = 0.5*(2+3) = 2.5 at x=[1,1]
        e = pe.QuadraticEnergy(
            (2, 2,
             [0, 1],
             [0, 1],
             np.array([2.0, 3.0], dtype=np.float64)),
        )
        assert e.num_dofs == 2
        assert e.state_kind == "generic"
        x = np.ones(2, dtype=np.float64)
        assert e.value(x) == pytest.approx(2.5, rel=1e-12)

    def test_construct_with_b(self):
        # A = [[4, 0], [0, 1]], b = [1, 2]
        # value(ones) = 0.5*(4+1) + 1+2 = 2.5 + 3 = 5.5
        e = pe.QuadraticEnergy(
            (2, 2,
             [0, 1],
             [0, 1],
             np.array([4.0, 1.0], dtype=np.float64)),
            b=np.array([1.0, 2.0], dtype=np.float64),
        )
        x = np.ones(2, dtype=np.float64)
        assert e.value(x) == pytest.approx(5.5, rel=1e-12)

    def test_gradient(self):
        e = pe.QuadraticEnergy(
            (3, 3,
             [0, 1, 2],
             [0, 1, 2],
             np.array([2.0, 4.0, 6.0], dtype=np.float64)),
        )
        x = np.array([1.0, 2.0, 3.0], dtype=np.float64)
        g = e.gradient(x)
        # diag(2,4,6) * [1,2,3] = [2,8,18]
        assert np.allclose(g, [2.0, 8.0, 18.0])

    def test_hessian_has_correct_entries(self):
        e = pe.QuadraticEnergy(
            (3, 3,
             [0, 1, 1, 2],
             [0, 0, 1, 2],
             np.array([1.0, 2.0, 3.0, 4.0], dtype=np.float64)),
        )
        H = e.hessian(np.ones(3, dtype=np.float64))
        assert H.nnz == 4

    def test_zero_state(self):
        e = pe.QuadraticEnergy(
            (4, 4,
             [0, 1, 2, 3],
             [0, 1, 2, 3],
             np.ones(4, dtype=np.float64)),
        )
        z = e.zero_state()
        assert z.shape == (4,)
        assert np.all(z == 0.0)

    def test_repr(self):
        e = pe.QuadraticEnergy(
            (5, 5,
             [0, 1, 2, 3, 4],
             [0, 1, 2, 3, 4],
             np.ones(5, dtype=np.float64)),
        )
        r = repr(e)
        assert "QuadraticEnergy" in r
        assert "5 DOFs" in r

    def test_owns_A_data(self):
        row_indices = [0, 1]
        col_indices = [0, 1]
        values = np.array([10.0, 20.0], dtype=np.float64)
        e = pe.QuadraticEnergy((2, 2, row_indices, col_indices, values))
        # mutate and delete
        values[0] = 999.0
        del row_indices, col_indices, values
        gc.collect()
        x = np.array([1.0, 0.0], dtype=np.float64)
        # value = 0.5 * 10 * 1^2 = 5.0
        assert e.value(x) == pytest.approx(5.0, rel=1e-12)

    def test_five_tuple_int_float_coercion(self):
        e = pe.QuadraticEnergy(
            (2, 2,
             [0, 1],
             [0, 1],
             [2.0, 4.0]),  # plain Python list for values
        )
        x = np.ones(2, dtype=np.float64)
        assert e.value(x) == pytest.approx(3.0, rel=1e-12)

    def test_wrong_lengths_raises(self):
        with pytest.raises(ValueError):
            pe.QuadraticEnergy(
                (2, 2, [0], [0, 1], np.array([1.0, 2.0], dtype=np.float64)),
            )

    def test_wrong_input_size_raises(self):
        e = pe.QuadraticEnergy(
            (3, 3,
             [0, 1, 2],
             [0, 1, 2],
             np.ones(3, dtype=np.float64)),
        )
        with pytest.raises(ValueError, match="State size mismatch"):
            e.value(np.ones(2, dtype=np.float64))


# ---------------------------------------------------------------------------
# Task E9b: VertexAttachment
# ---------------------------------------------------------------------------

class TestVertexAttachment:
    def test_construct_and_basic_properties(self):
        n = 6
        koff = (n, n, list(range(n)), list(range(n)), [1.0] * n)
        rest = np.zeros(n, dtype=np.float64)
        vtx = np.array([0, 1], dtype=np.int64)
        tgt = np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0], dtype=np.float64)

        e = pe.VertexAttachment(
            koff=koff,
            vertex_indices=vtx,
            target_positions=tgt,
            coeff=100.0,
        )
        assert e.num_dofs == 6
        assert e.state_kind == "displacement"
        assert "VertexAttachment" in repr(e)

    def test_value_and_gradient(self):
        n = 6
        koff = (n, n, list(range(n)), list(range(n)), [1.0] * n)
        vtx = np.array([1], dtype=np.int64)  # pin vertex 1
        tgt = np.array([0.1, 0.2, 0.3], dtype=np.float64)

        e = pe.VertexAttachment(
            koff=koff,
            vertex_indices=vtx,
            target_positions=tgt,
            coeff=10.0,
        )
        x = np.zeros(6, dtype=np.float64)
        # E = 0.5 * 10 * (0^2 + 0^2 + 0^2 + (-0.1)^2 + (-0.2)^2 + (-0.3)^2) = 5 * 0.14 = 0.7
        # Actually: diff = [0,0,0, -0.1,-0.2,-0.3]; E = 0.5 * 10 * (0.01+0.04+0.09) = 5 * 0.14 = 0.7
        assert e.value(x) == pytest.approx(0.7, rel=1e-12)

        # gradient: coeff * diff for pinned vertices
        g = e.gradient(x)
        assert np.allclose(g[3:6], [-1.0, -2.0, -3.0])
        assert np.allclose(g[0:3], [0.0, 0.0, 0.0])

    def test_hessian_has_diagonal_entries(self):
        n = 3
        koff = (n, n, list(range(n)), list(range(n)), [1.0] * n)
        e = pe.VertexAttachment(
            koff=koff,
            vertex_indices=np.array([0], dtype=np.int64),
            target_positions=np.zeros(3, dtype=np.float64),
            coeff=5.0,
        )
        H = e.hessian(np.zeros(3, dtype=np.float64))
        assert H.nnz == 3  # diagonal-only Koff

    def test_generic_state_kind_when_not_displacement(self):
        n = 3
        koff = (n, n, list(range(n)), list(range(n)), [1.0] * n)
        e = pe.VertexAttachment(
            koff=koff,
            vertex_indices=np.array([0], dtype=np.int64),
            target_positions=np.zeros(3, dtype=np.float64),
            coeff=1.0,
            is_displacement=False,
        )
        assert e.state_kind == "generic"

    def test_set_targets_updates_energy_in_place(self):
        n = 3
        koff = (n, n, list(range(n)), list(range(n)), [1.0] * n)
        e = pe.VertexAttachment(
            koff=koff,
            vertex_indices=np.array([0], dtype=np.int64),
            target_positions=np.zeros(3, dtype=np.float64),
            coeff=2.0,
        )
        handle = e._handle
        x = np.zeros(3, dtype=np.float64)

        e.set_targets(np.array([1.0, 2.0, 2.0], dtype=np.float64))

        assert e._handle is handle
        assert e.value(x) == pytest.approx(9.0, rel=1e-12)


# ---------------------------------------------------------------------------
# Task E6: EnergySet
# ---------------------------------------------------------------------------

class TestEnergySet:
    def test_construct_and_num_terms(self):
        q = pe.QuadraticEnergy(
            (3, 3, [0, 1, 2], [0, 1, 2], np.ones(3, dtype=np.float64)),
        )
        es = pe.EnergySet([(q, 1.0)])
        assert es.num_terms == 1
        assert es.num_dofs == 3

    def test_sum_of_two_energies(self):
        q = pe.QuadraticEnergy(
            (2, 2, [0, 1], [0, 1], np.array([2.0, 0.0], dtype=np.float64)),
        )
        lin = pe.LinearEnergy(np.array([1.0, 0.0], dtype=np.float64))
        es = pe.EnergySet([(q, 1.0), (lin, 1.0)])

        x = np.array([1.0, 0.0], dtype=np.float64)
        # q: 0.5 * 2 * 1^2 = 1.0
        # lin: 1.0 * 1 + 0 = 1.0
        # total = 2.0
        assert es.value(x) == pytest.approx(2.0, rel=1e-12)

    def test_set_weight(self):
        q = pe.QuadraticEnergy(
            (2, 2, [0, 1], [0, 1], np.array([2.0, 0.0], dtype=np.float64)),
        )
        lin = pe.LinearEnergy(np.array([1.0, 0.0], dtype=np.float64))
        es = pe.EnergySet([(q, 1.0), (lin, 1.0)])

        x = np.array([1.0, 0.0], dtype=np.float64)
        val_before = es.value(x)

        es.set_weight(1, 0.0)  # disable linear term
        val_after = es.value(x)
        assert val_after < val_before

    def test_state_kind_composition(self):
        # displacement + displacement → displacement
        n = 3
        koff = (n, n, list(range(n)), list(range(n)), [1.0] * n)
        va = pe.VertexAttachment(
            koff=koff,
            vertex_indices=np.array([0], dtype=np.int64),
            target_positions=np.zeros(3, dtype=np.float64),
        )
        es = pe.EnergySet([(va, 1.0), (va, 1.0)])
        assert es.state_kind == "displacement"

        # displacement + generic → generic
        lin = pe.LinearEnergy(np.zeros(3, dtype=np.float64))
        es2 = pe.EnergySet([(va, 1.0), (lin, 1.0)])
        assert es2.state_kind == "generic"

    def test_repr(self):
        q = pe.QuadraticEnergy(
            (4, 4, [0, 1, 2, 3], [0, 1, 2, 3], np.ones(4, dtype=np.float64)),
        )
        es = pe.EnergySet([(q, 0.5), (q, 2.0)])
        r = repr(es)
        assert "EnergySet" in r
        assert "2 terms" in r
        assert "4 DOFs" in r
        assert "state_kind" in r

    def test_hessian(self):
        q = pe.QuadraticEnergy(
            (3, 3, [0, 1, 2], [0, 1, 2], np.ones(3, dtype=np.float64)),
        )
        es = pe.EnergySet([(q, 1.0)])
        H = es.hessian(np.ones(3, dtype=np.float64))
        assert H.nnz == 3

    def test_zero_terms_raises(self):
        with pytest.raises(ValueError, match="at least one term"):
            pe.EnergySet([])

    def test_mismatched_dofs_raises(self):
        q3 = pe.QuadraticEnergy(
            (3, 3, [0, 1, 2], [0, 1, 2], np.ones(3, dtype=np.float64)),
        )
        q4 = pe.QuadraticEnergy(
            (4, 4, [0, 1, 2, 3], [0, 1, 2, 3], np.ones(4, dtype=np.float64)),
        )
        with pytest.raises(Exception):  # ValueError from C++
            pe.EnergySet([(q3, 1.0), (q4, 1.0)])

    def test_lifetime_after_child_deletion(self):
        q = pe.QuadraticEnergy(
            (2, 2, [0, 1], [0, 1], np.array([2.0, 3.0], dtype=np.float64)),
        )
        es = pe.EnergySet([(q, 1.0)])
        del q
        gc.collect()
        x = np.ones(2, dtype=np.float64)
        assert es.value(x) == pytest.approx(2.5, rel=1e-12)


# ---------------------------------------------------------------------------
# Task E7: state_kind across types
# ---------------------------------------------------------------------------

class TestStateKind:
    def test_linear_is_generic(self):
        assert pe.LinearEnergy(np.ones(3)).state_kind == "generic"

    def test_quadratic_is_generic(self):
        assert pe.QuadraticEnergy(
            (2, 2, [0, 1], [0, 1], np.ones(2)),
        ).state_kind == "generic"

    def test_vertex_attachment_displacement_default(self):
        n = 6
        koff = (n, n, list(range(n)), list(range(n)), [1.0] * n)
        e = pe.VertexAttachment(
            koff=koff,
            vertex_indices=np.array([0], dtype=np.int64),
            target_positions=np.zeros(3, dtype=np.float64),
        )
        assert e.state_kind == "displacement"

    def test_energy_set_single_term_passthrough(self):
        va = pe.VertexAttachment(
            koff=(3, 3, [0, 1, 2], [0, 1, 2], [1.0] * 3),
            vertex_indices=np.array([0], dtype=np.int64),
            target_positions=np.zeros(3, dtype=np.float64),
        )
        es = pe.EnergySet([(va, 1.0)])
        assert es.state_kind == "displacement"

    def test_zero_state_is_zeros(self):
        for e in [
            pe.LinearEnergy(np.ones(4, dtype=np.float64)),
            pe.QuadraticEnergy(
                (4, 4, [0, 1, 2, 3], [0, 1, 2, 3], np.ones(4))),
        ]:
            z = e.zero_state()
            assert isinstance(z, np.ndarray)
            assert np.all(z == 0.0)
            assert z.dtype == np.float64


# ---------------------------------------------------------------------------
# Handle architecture: concrete peers
# ---------------------------------------------------------------------------

def test_energy_handles_are_concrete_peers_and_abstract_peers():
    linear = pe.LinearEnergy([1.0, 2.0])
    quadratic = pe.QuadraticEnergy(
        (2, 2, [0, 1], [0, 1], np.array([1.0, 2.0], dtype=np.float64))
    )
    total = pe.EnergySet([(linear, 1.0), (quadratic, 2.0)])

    assert isinstance(linear._handle, _core.PyOwnedPotentialEnergy)
    assert isinstance(quadratic._handle, _core.PyOwnedPotentialEnergy)
    assert isinstance(total._handle, _core.PyEnergySet)
    assert isinstance(linear._handle, _core.PyPotentialEnergy)
    assert isinstance(total._handle, _core.PyPotentialEnergy)
    assert not hasattr(linear, "_potential_handle")
    assert not hasattr(linear._handle, "as_potential_energy")


class TestEmbeddedVertexAttachment:
    def test_identity_matches_vertex_attachment_value(self):
        """With identity embedding the energy equals VertexAttachment at rest target."""
        import pypgo.energy as pe

        n_verts = 4
        n = 3 * n_verts
        idx = [1, 3]
        coeff = 250.0
        emb = pe.EmbeddedVertexAttachment(
            embedding=None, vertex_indices=idx, coeff=coeff, num_dofs=n)
        rng = np.random.default_rng(7)
        u = rng.normal(size=n)
        expected = coeff * sum(
            float(np.dot(u[3*i:3*i+3], u[3*i:3*i+3])) for i in idx)
        assert emb.value(u) == pytest.approx(expected, rel=1e-12)

    def test_embedded_value_matches_dense_formula(self):
        """E = coeff * ||(W u)_S||^2 against a dense reference computation."""
        import pypgo.energy as pe
        from pypgo.sparse import as_sparse_matrix

        rng = np.random.default_rng(3)
        n, m = 12, 5  # sim dofs, embedded vertices
        W_dense = rng.normal(size=(3 * m, n)) * (rng.random((3 * m, n)) < 0.4)
        W = as_sparse_matrix(W_dense)
        idx = [0, 2, 4]
        coeff = 11.0
        e = pe.EmbeddedVertexAttachment(embedding=W, vertex_indices=idx, coeff=coeff)
        u = rng.normal(size=n)
        rows = np.concatenate([[3*i, 3*i+1, 3*i+2] for i in idx])
        expected = coeff * float(np.sum((W_dense[rows] @ u) ** 2))
        assert e.value(u) == pytest.approx(expected, rel=1e-10)
        assert e.num_dofs == n

    def test_requires_num_dofs_without_embedding(self):
        import pypgo.energy as pe

        with pytest.raises(ValueError, match="num_dofs"):
            pe.EmbeddedVertexAttachment(embedding=None, vertex_indices=[0], coeff=1.0)

    def test_out_of_range_rejected(self):
        import pypgo.energy as pe

        with pytest.raises(ValueError, match="range"):
            pe.EmbeddedVertexAttachment(
                embedding=None, vertex_indices=[5], coeff=1.0, num_dofs=9)
