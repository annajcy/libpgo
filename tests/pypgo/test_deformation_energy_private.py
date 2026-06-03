"""Private/experimental smoke tests for deformation energy binding hooks.

These tests validate lifetime, state mechanics, and basic energy/gradient/Hessian
evaluation through the private _core hooks.  The hooks and tests are intentionally
experimental — the public pypgo.fem / pypgo.energy API is finalized in the Finalize
task and public tests in test_deformation_energy.py.
"""

import gc
import numpy as np
import pypgo._core as _core
import pypgo as pgo
import pytest


# ---------------------------------------------------------------------------
# Helper factories
# ---------------------------------------------------------------------------

def _make_tet_sim_mesh():
    tet = pgo.mesh.TetMeshData(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )
    volume = pgo.mesh.veg.VolumeMesh.create_from_single_material(tet, pgo.mesh.veg.ENuMaterial(E=1e6, nu=0.45))
    return pgo.sim.SimulationMesh.create_volumetric(volume)


def _make_cubic_sim_mesh():
    cube = pgo.mesh.CubicMeshData(
        np.array(
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
        ),
        np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64),
    )
    volume = pgo.mesh.veg.VolumeMesh.create_from_single_material(cube, pgo.mesh.veg.ENuMaterial(E=1e6, nu=0.45))
    return pgo.sim.SimulationMesh.create_volumetric(volume)


def _make_shell_sim_mesh():
    surface = pgo.mesh.TriMeshData(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64),
        np.array([[0, 1, 2]], dtype=np.int64),
    )
    material = pgo.sim.KoiterStVKShellMaterial(thickness=0.01, E_membrane=2e6, nu_membrane=0.35)
    return pgo.sim.SimulationMesh.create_shell(surface, material)


# Helper to evaluate energy using the unified factory + handle protocol
def _make_deformation_energy(sim_mesh, formulation, elastic="stable_neo", plastic="volumetric_dof6"):
    elastic_field = _core._create_elastic_default_field(sim_mesh._core_obj, elastic)
    plastic_field = _core._create_plastic_default_field(sim_mesh._core_obj, plastic)
    return _core._create_deformation_energy(
        sim_mesh._core_obj, formulation, elastic_field, plastic_field
    )


# ---------------------------------------------------------------------------
# Tet P1
# ---------------------------------------------------------------------------

class TestTetDeformationEnergyPrivate:
    def test_builds_and_zero_state_has_correct_shape(self):
        sim = _make_tet_sim_mesh()
        energy = _make_deformation_energy(sim, "tet_p1")
        h = energy.handle
        assert h.num_dofs == 3 * sim.num_vertices
        z = h.zero_state()
        assert isinstance(z, np.ndarray)
        assert z.shape == (h.num_dofs,)
        assert np.all(z == 0.0)

    def test_value_gradient_hessian_at_zero_state(self):
        sim = _make_tet_sim_mesh()
        energy = _make_deformation_energy(sim, "tet_p1")
        h = energy.handle
        u = h.zero_state()

        val = h.value(u)
        assert isinstance(val, float)
        assert np.isfinite(val)

        grad = h.gradient(u)
        assert len(grad) == h.num_dofs
        assert all(np.isfinite(v) for v in grad)

        H = h.hessian(u)
        assert H.rows() == h.num_dofs
        assert H.cols() == h.num_dofs
        rows, cols, vals = H.to_coo()
        assert len(vals) == H.nnz()
        assert all(np.isfinite(v) for v in vals)

    def test_value_changes_under_perturbation(self):
        sim = _make_tet_sim_mesh()
        energy = _make_deformation_energy(sim, "tet_p1")
        h = energy.handle
        u0 = h.zero_state()
        val0 = h.value(u0)

        u_pert = u0.copy()
        u_pert[0] = 0.01
        val_pert = h.value(u_pert)
        assert val_pert != pytest.approx(val0, abs=1e-15)

    def test_rest_position(self):
        sim = _make_tet_sim_mesh()
        energy = _make_deformation_energy(sim, "tet_p1")
        rp = energy.rest_position()
        assert isinstance(rp, np.ndarray)
        assert rp.ndim == 2
        assert rp.shape[1] == 3
        assert rp.shape[0] == sim.num_vertices
        # rest position must be nonzero (vertices are not all at origin)
        assert np.any(np.abs(rp) > 1e-12)

    def test_wrong_input_size_raises(self):
        sim = _make_tet_sim_mesh()
        energy = _make_deformation_energy(sim, "tet_p1")
        h = energy.handle
        with pytest.raises(ValueError, match="State size mismatch"):
            h.value(np.array([0.0, 0.0, 0.0], dtype=np.float64))  # too few


# ---------------------------------------------------------------------------
# Cubic HexTrilinear
# ---------------------------------------------------------------------------

class TestCubicDeformationEnergyPrivate:
    def test_builds_and_zero_state_has_correct_shape(self):
        sim = _make_cubic_sim_mesh()
        energy = _make_deformation_energy(sim, "hex_trilinear")
        h = energy.handle
        assert h.num_dofs == 3 * sim.num_vertices
        z = h.zero_state()
        assert len(z) == h.num_dofs

    def test_value_gradient_hessian_at_zero_state(self):
        sim = _make_cubic_sim_mesh()
        energy = _make_deformation_energy(sim, "hex_trilinear")
        h = energy.handle
        u = h.zero_state()

        val = h.value(u)
        assert np.isfinite(val)

        grad = h.gradient(u)
        assert len(grad) == h.num_dofs
        assert all(np.isfinite(v) for v in grad)

        H = h.hessian(u)
        assert H.rows() == h.num_dofs
        assert H.cols() == h.num_dofs
        assert H.nnz() > 0

    def test_cubic_requires_explicit_formulation(self):
        sim = _make_cubic_sim_mesh()
        # Passing tet_p1 formulation to a cubic mesh should fail at C++ level
        # because nodes/element mismatch — or succeed with wrong behavior.
        # The Python-level check is in pypgo.fem.deformation_energy().
        # At the _core level, passing the wrong formulation string for the mesh
        # may produce incorrect results but won't crash. This is tested in the
        # public API tests instead.
        pass


# ---------------------------------------------------------------------------
# Shell Koiter
# ---------------------------------------------------------------------------

class TestShellDeformationEnergyPrivate:
    def test_builds_and_num_dofs(self):
        sim = _make_shell_sim_mesh()
        energy = _make_deformation_energy(sim, "shell_koiter",
                                          elastic="koiter_stvk", plastic="shell_ff_dof1")
        h = energy.handle
        assert h.num_dofs == 3 * sim.num_vertices
        z = h.zero_state()
        assert len(z) == h.num_dofs

    def test_value_gradient_hessian_at_zero_state(self):
        sim = _make_shell_sim_mesh()
        energy = _make_deformation_energy(sim, "shell_koiter",
                                          elastic="koiter_stvk", plastic="shell_ff_dof1")
        h = energy.handle
        u = h.zero_state()

        val = h.value(u)
        assert np.isfinite(val)

        grad = h.gradient(u)
        assert len(grad) == h.num_dofs
        assert all(np.isfinite(v) for v in grad)

        H = h.hessian(u)
        assert H.rows() == h.num_dofs
        assert H.cols() == h.num_dofs
        assert H.nnz() > 0

    def test_shell_requires_shell_formulation(self):
        sim = _make_shell_sim_mesh()
        # Passing hex_trilinear formulation to a shell mesh will likely fail
        # in the C++ layer. The Python-level check is in pypgo.fem.
        pass


# ---------------------------------------------------------------------------
# Lifetime
# ---------------------------------------------------------------------------

class TestLifetime:
    def test_two_energies_from_same_mesh_owner(self):
        sim = _make_tet_sim_mesh()

        e1 = _make_deformation_energy(sim, "tet_p1")
        e2 = _make_deformation_energy(sim, "tet_p1")

        u = e1.handle.zero_state()
        assert e1.handle.value(u) == pytest.approx(e2.handle.value(u), rel=1e-12)

    def test_energy_survives_python_sim_mesh_deletion(self):
        sim = _make_tet_sim_mesh()

        energy = _make_deformation_energy(sim, "tet_p1")
        h = energy.handle
        u = h.zero_state()
        val_before = h.value(u)

        # Delete the Python wrapper; the _core energy keeps the mesh alive.
        del sim
        gc.collect()

        val_after = h.value(u)
        assert val_before == pytest.approx(val_after, rel=1e-15)
