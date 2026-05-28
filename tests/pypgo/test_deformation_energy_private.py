"""Private/experimental smoke tests for deformation energy binding hooks.

These tests validate lifetime, state mechanics, and basic energy/gradient/Hessian
evaluation through the private _core hooks.  The hooks and tests are intentionally
experimental — the public pypgo.fem / pypgo.energy API will be finalized in Task 10
after C++ formulation, material recipe, and DofLayout boundaries stabilize.

DO NOT depend on _create_*_deformation_energy_for_test names or signatures in
production code.
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


# ---------------------------------------------------------------------------
# Tet P1
# ---------------------------------------------------------------------------

class TestTetDeformationEnergyPrivate:
    def test_builds_and_zero_state_has_correct_shape(self):
        sim = _make_tet_sim_mesh()
        energy = _core._create_tet_deformation_energy_for_test(
            sim._core_obj, formulation="tet_p1",
        )
        assert energy.num_dofs() == 3 * sim.num_vertices
        z = energy.zero_state()
        assert isinstance(z, list)
        assert len(z) == energy.num_dofs()
        assert all(v == 0.0 for v in z)

    def test_value_gradient_hessian_at_zero_state(self):
        sim = _make_tet_sim_mesh()
        energy = _core._create_tet_deformation_energy_for_test(
            sim._core_obj, formulation="tet_p1",
        )
        u = energy.zero_state()

        val = energy.value(u)
        assert isinstance(val, float)
        assert np.isfinite(val)

        grad = energy.gradient(u)
        assert len(grad) == energy.num_dofs()
        assert all(np.isfinite(v) for v in grad)

        H = energy.hessian(u)
        assert H.rows() == energy.num_dofs()
        assert H.cols() == energy.num_dofs()
        rows, cols, vals = H.to_coo()
        assert len(vals) == H.nnz()
        assert all(np.isfinite(v) for v in vals)

    def test_value_changes_under_perturbation(self):
        sim = _make_tet_sim_mesh()
        energy = _core._create_tet_deformation_energy_for_test(
            sim._core_obj, formulation="tet_p1",
        )
        u0 = energy.zero_state()
        val0 = energy.value(u0)

        # Perturb only one node in x so the deformation gradient is not identity.
        u_pert = list(u0)
        u_pert[0] = 0.01
        val_pert = energy.value(u_pert)
        assert val_pert != pytest.approx(val0, abs=1e-15)

    def test_rest_position_and_zero_state_convention(self):
        sim = _make_tet_sim_mesh()
        energy = _core._create_tet_deformation_energy_for_test(
            sim._core_obj, formulation="tet_p1",
        )
        rest = energy.rest_position_flat()
        assert len(rest) == energy.num_dofs()
        # rest position must be nonzero (vertices are not all at origin)
        assert any(abs(v) > 1e-12 for v in rest)

        z = energy.zero_state()
        # zero_state is displacement, not absolute position
        assert all(v == 0.0 for v in z)

    def test_wrong_input_size_raises(self):
        sim = _make_tet_sim_mesh()
        energy = _core._create_tet_deformation_energy_for_test(
            sim._core_obj, formulation="tet_p1",
        )
        with pytest.raises(ValueError, match="must equal num_dofs"):
            energy.value([0.0, 0.0, 0.0])  # too few


# ---------------------------------------------------------------------------
# Cubic HexTrilinear
# ---------------------------------------------------------------------------

class TestCubicDeformationEnergyPrivate:
    def test_builds_and_zero_state_has_correct_shape(self):
        sim = _make_cubic_sim_mesh()
        energy = _core._create_cubic_deformation_energy_for_test(
            sim._core_obj, formulation="hex_trilinear",
        )
        assert energy.num_dofs() == 3 * sim.num_vertices
        z = energy.zero_state()
        assert len(z) == energy.num_dofs()

    def test_value_gradient_hessian_at_zero_state(self):
        sim = _make_cubic_sim_mesh()
        energy = _core._create_cubic_deformation_energy_for_test(
            sim._core_obj, formulation="hex_trilinear",
        )
        u = energy.zero_state()

        val = energy.value(u)
        assert np.isfinite(val)

        grad = energy.gradient(u)
        assert len(grad) == energy.num_dofs()
        assert all(np.isfinite(v) for v in grad)

        H = energy.hessian(u)
        assert H.rows() == energy.num_dofs()
        assert H.cols() == energy.num_dofs()
        assert H.nnz() > 0

    def test_cubic_requires_explicit_hex_trilinear(self):
        sim = _make_cubic_sim_mesh()
        with pytest.raises(ValueError, match="Unknown cubic formulation"):
            _core._create_cubic_deformation_energy_for_test(
                sim._core_obj, formulation="tet_p1",
            )


# ---------------------------------------------------------------------------
# Shell Koiter
# ---------------------------------------------------------------------------

class TestShellDeformationEnergyPrivate:
    def test_builds_and_num_dofs(self):
        sim = _make_shell_sim_mesh()
        energy = _core._create_shell_deformation_energy_for_test(
            sim._core_obj, formulation="shell_koiter",
        )
        assert energy.num_dofs() == 3 * sim.num_vertices
        z = energy.zero_state()
        assert len(z) == energy.num_dofs()

    def test_value_gradient_hessian_at_zero_state(self):
        sim = _make_shell_sim_mesh()
        energy = _core._create_shell_deformation_energy_for_test(
            sim._core_obj, formulation="shell_koiter",
        )
        u = energy.zero_state()

        val = energy.value(u)
        assert np.isfinite(val)

        grad = energy.gradient(u)
        assert len(grad) == energy.num_dofs()
        assert all(np.isfinite(v) for v in grad)

        H = energy.hessian(u)
        assert H.rows() == energy.num_dofs()
        assert H.cols() == energy.num_dofs()
        assert H.nnz() > 0

    def test_shell_requires_explicit_shell_koiter(self):
        sim = _make_shell_sim_mesh()
        with pytest.raises(ValueError, match="Unknown shell formulation"):
            _core._create_shell_deformation_energy_for_test(
                sim._core_obj, formulation="hex_trilinear",
            )


# ---------------------------------------------------------------------------
# Lifetime
# ---------------------------------------------------------------------------

class TestLifetime:
    def test_two_energies_from_same_mesh_owner(self):
        sim = _make_tet_sim_mesh()
        core = sim._core_obj

        e1 = _core._create_tet_deformation_energy_for_test(core, formulation="tet_p1")
        e2 = _core._create_tet_deformation_energy_for_test(core, formulation="tet_p1")

        u = e1.zero_state()
        assert e1.value(u) == pytest.approx(e2.value(u), rel=1e-12)

    def test_energy_survives_python_sim_mesh_deletion(self):
        sim = _make_tet_sim_mesh()
        core = sim._core_obj

        energy = _core._create_tet_deformation_energy_for_test(core, formulation="tet_p1")
        u = energy.zero_state()
        val_before = energy.value(u)

        # Delete the Python wrapper; the _core energy keeps the mesh alive.
        del sim
        gc.collect()

        val_after = energy.value(u)
        assert val_before == pytest.approx(val_after, rel=1e-15)
