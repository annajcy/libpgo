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
    return _core._create_deformation_energy(
        assignment._handle,
        formulation_handle._handle,
        None,
        True,
        True,
    )


class TestCoreDeformationEnergy:
    def test_energy_exposes_shared_parameters(self):
        sim = _make_tet_sim_mesh()
        energy = _make_deformation_energy(sim, "tet_linear")

        assert energy.elastic_definition.name == "stable_neo"
        assert energy.plastic_definition.name == "volumetric_dof6"
        assert energy.optimizable_parameters.elastic_field.num_material_channels == 0
        assert energy.optimizable_parameters.elastic_values.shape == (0, 0)
        assert energy.optimizable_parameters.plastic_values.shape == (sim.num_elements, 6)

    def test_parameter_owner_setters_update_committed_values(self):
        sim = _make_tet_sim_mesh()
        energy = _make_deformation_energy(sim, "tet_linear")
        values = np.array([[1.05, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64)
        energy.optimizable_parameters.set_plastic_values(values.ravel())
        assert np.allclose(energy.optimizable_parameters.plastic_values, values)

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

    def test_energy_observes_state_updates(self):
        sim = _make_tet_sim_mesh()
        energy = _make_deformation_energy(sim, "tet_linear", elastic=pf.StVKDefinition())
        h = energy
        u = h.zero_state()

        before = h.value(u)
        energy.optimizable_parameters.set_plastic_values(
            np.array([[1.05, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64).ravel()
        )
        after = h.value(u)
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
