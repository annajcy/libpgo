"""Private smoke tests for state-based deformation _core hooks."""

import gc

import numpy as np
import pypgo as pgo
import pypgo._core as _core
import pytest


def _make_tet_sim_mesh():
    tet = pgo.mesh.TetMeshData(
        np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )
    volume = pgo.mesh.volume.VolumeMesh.create_from_single_material(
        tet, pgo.mesh.volume.ENuMaterial(E=1e6, nu=0.45)
    )
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
    volume = pgo.mesh.volume.VolumeMesh.create_from_single_material(
        cube, pgo.mesh.volume.ENuMaterial(E=1e6, nu=0.45)
    )
    return pgo.sim.SimulationMesh.create_volumetric(volume)


def _make_state(sim, elastic="stable_neo", plastic="volumetric_dof6", plastic_values=None):
    return _core._create_deformation_model_state(
        sim._handle,
        elastic,
        None,
        plastic,
        plastic_values,
    )


def _make_deformation_energy(sim, formulation, elastic="stable_neo", plastic="volumetric_dof6"):
    state = _make_state(sim, elastic=elastic, plastic=plastic)
    return _core._create_deformation_energy(state, formulation)


class TestCoreState:
    def test_elastic_num_channels_uses_cpp_parameter_spec(self):
        tet_sim = _make_tet_sim_mesh()
        assert _core._elastic_num_channels(tet_sim._handle, "stable_neo") == 0

        shell_sim = pgo.sim.SimulationMesh.create_shell(
            pgo.mesh.TriMeshData(
                np.array(
                    [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
                    dtype=np.float64,
                ),
                np.array([[0, 1, 2]], dtype=np.int64),
            ),
            pgo.sim.KoiterStVKShellMaterial(
                thickness=0.01, E_membrane=2e6, nu_membrane=0.35
            ),
        )
        assert _core._elastic_num_channels(shell_sim._handle, "koiter_stvk") == 5
        assert _core._elastic_num_channels(shell_sim._handle, "koiter_fabric") == 12

    def test_state_exposes_state_owned_fields(self):
        sim = _make_tet_sim_mesh()
        state = _make_state(sim)

        assert state.elastic_model == "stable_neo"
        assert state.plastic_model == "volumetric_dof6"
        assert state.num_elements == sim.num_elements
        assert state.elastic_field.num_channels == 0
        assert state.elastic_field.values().shape == (0, 0)
        assert state.plastic_field.values().shape == (sim.num_elements, 6)

    def test_state_setters_update_fields(self):
        sim = _make_tet_sim_mesh()
        state = _make_state(sim)
        values = np.array([[1.05, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64)
        state.set_plastic_values(values.ravel())
        assert np.allclose(state.plastic_field.values(), values)

    def test_wrong_size_rejected(self):
        sim = _make_tet_sim_mesh()
        with pytest.raises(ValueError):
            _make_state(sim, plastic_values=np.zeros(5, dtype=np.float64))

    def test_old_field_factories_are_not_exposed(self):
        assert not hasattr(_core, "_create_elastic_default_field")
        assert not hasattr(_core, "_create_plastic_default_field")


class TestCoreEnergy:
    def test_tet_value_gradient_hessian_at_zero_state(self):
        sim = _make_tet_sim_mesh()
        energy = _make_deformation_energy(sim, "tet_p1")
        h = energy
        u = h.zero_state()

        assert h.num_dofs == 3 * sim.num_vertices
        assert np.isfinite(h.value(u))
        assert h.gradient(u).shape == (h.num_dofs,)
        H = h.hessian(u)
        assert H.rows() == h.num_dofs
        assert H.cols() == h.num_dofs
        assert H.nnz() > 0

    def test_cubic_value_gradient_hessian_at_zero_state(self):
        sim = _make_cubic_sim_mesh()
        energy = _make_deformation_energy(sim, "hex_trilinear")
        h = energy
        u = h.zero_state()

        assert np.isfinite(h.value(u))
        assert h.gradient(u).shape == (h.num_dofs,)
        assert h.hessian(u).nnz() > 0

    def test_energy_observes_state_updates(self):
        sim = _make_tet_sim_mesh()
        state = _make_state(sim, elastic="stvk")
        energy = _core._create_deformation_energy(state, "tet_p1")
        h = energy
        u = h.zero_state()

        before = h.value(u)
        state.set_plastic_values(
            np.array([[1.05, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64).ravel()
        )
        after = h.value(u)
        assert after != pytest.approx(before, abs=1e-15)

    def test_energy_survives_mesh_and_state_deletion(self):
        sim = _make_tet_sim_mesh()
        state = _make_state(sim)
        energy = _core._create_deformation_energy(state, "tet_p1")
        h = energy
        u = h.zero_state()
        before = h.value(u)

        del state
        del sim
        gc.collect()

        assert h.value(u) == pytest.approx(before, rel=1e-15)

    def test_unknown_formulation_raises(self):
        sim = _make_tet_sim_mesh()
        state = _make_state(sim)
        with pytest.raises(ValueError, match="Unknown formulation"):
            _core._create_deformation_energy(state, "bad_formulation")
