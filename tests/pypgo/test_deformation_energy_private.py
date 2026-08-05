"""Private smoke tests for assignment-based deformation _core hooks."""

import gc

import numpy as np
import pypgo as pgo
import pypgo._core as _core
import pypgo.fem as pf
import pytest
from tests.pypgo.material_helpers import direct_assignment


def _make_tet_sim_mesh():
    tet = pgo.mesh.TetMeshData(
        np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )
    volume = pgo.mesh.volume.VolumeMesh(
        tet, pgo.mesh.volume.ENuMaterial(E=1e6, nu=0.45)
    )
    return pgo.fem.SimulationImportResult(volume)


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
    volume = pgo.mesh.volume.VolumeMesh(
        cube, pgo.mesh.volume.ENuMaterial(E=1e6, nu=0.45)
    )
    return pgo.fem.SimulationImportResult(volume)


def _make_deformation_energy(sim, formulation, elastic=None, plastic=None, plastic_values=None):
    formulation_handle = {
        "tet_linear": pf.TetLinear,
        "cubic_linear": pf.CubicLinear,
        "cubic_tricubic_hermite": pf.CubicTricubicHermite,
        "shell_koiter": pf.KoiterShell,
    }.get(formulation, lambda: None)()
    if formulation_handle is None:
        raise ValueError(f"Unknown formulation: {formulation}")
    elastic = elastic or pf.StableNeoDefinition()
    plastic = plastic or pf.VolumetricPlasticityDefinition(dofs=6)
    assignment = direct_assignment(
        sim, elastic, plastic,
        pf.ElementwiseParameterLayout, pf.ElementwiseParameterLayout,
        None, plastic_values)
    operator = _core._create_deformation_energy_operator(
        assignment._handle,
        formulation_handle._handle,
        None,
        True,
        True,
    )
    return _core._create_deformation_potential_energy(
        operator, assignment._handle.initial_material_state)


class TestCoreDeformationEnergyOperator:
    def test_energy_exposes_shared_parameters(self):
        sim = _make_tet_sim_mesh()
        energy = _make_deformation_energy(sim, "tet_linear")

        operator = energy.energy_operator
        state = energy.material_state
        assert operator.elastic_definition.name == "stable_neo"
        assert operator.plastic_definition.name == "volumetric_dof6"
        assert state.elastic_field.num_material_channels == 0
        assert state.elastic_values.shape == (0, 0)
        assert state.plastic_values.shape == (sim.num_elements, 6)

    def test_material_state_has_no_mutating_setters(self):
        sim = _make_tet_sim_mesh()
        energy = _make_deformation_energy(sim, "tet_linear")
        state = energy.material_state
        assert not hasattr(state, "set_plastic_values")
        assert not hasattr(state, "set_elastic_values")

    def test_wrong_size_rejected(self):
        sim = _make_tet_sim_mesh()
        with pytest.raises(ValueError):
            _make_deformation_energy(sim, "tet_linear", plastic_values=np.zeros(5, dtype=np.float64))

    def test_old_field_factories_are_not_exposed(self):
        assert not hasattr(_core, "_create_elastic_default_field")
        assert not hasattr(_core, "_create_plastic_default_field")


class TestCoreEnergy:
    def test_tet_value_gradient_hessian_at_zero_state(self):
        sim = _make_tet_sim_mesh()
        energy = _make_deformation_energy(sim, "tet_linear")
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
        energy = _make_deformation_energy(sim, "cubic_linear")
        h = energy
        u = h.zero_state()

        assert np.isfinite(h.value(u))
        assert h.gradient(u).shape == (h.num_dofs,)
        assert h.hessian(u).nnz() > 0

    def test_new_potential_uses_new_explicit_state(self):
        sim = _make_tet_sim_mesh()
        energy = _make_deformation_energy(sim, "tet_linear", elastic=pf.StVKDefinition())
        h = energy
        u = h.zero_state()

        before = h.value(u)
        state = energy.material_state.with_plastic_values(
            np.array([[1.05, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64).ravel())
        changed = _core._create_deformation_potential_energy(
            energy.energy_operator, state)
        after = changed.value(u)
        assert after != pytest.approx(before, abs=1e-15)

    def test_energy_survives_mesh_and_state_deletion(self):
        sim = _make_tet_sim_mesh()
        energy = _make_deformation_energy(sim, "tet_linear")
        h = energy
        u = h.zero_state()
        before = h.value(u)

        del sim
        gc.collect()

        assert h.value(u) == pytest.approx(before, rel=1e-15)

    def test_unknown_formulation_raises(self):
        sim = _make_tet_sim_mesh()
        with pytest.raises(ValueError, match="Unknown formulation"):
            _make_deformation_energy(sim, "bad_formulation")
