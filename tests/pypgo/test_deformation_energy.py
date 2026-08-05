"""Public tests for the config-based pypgo.fem deformation API."""

import gc

import numpy as np
import pypgo as pgo
import pypgo.energy as pe
import pypgo.fem as pf
import pypgo.solver as ps
import pytest
from tests.pypgo.material_helpers import direct_material


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
    return volume


def _make_mooney_rivlin_tet_sim_mesh():
    tet = pgo.mesh.TetMeshData(
        np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )
    volume = pgo.mesh.volume.VolumeMesh(
        tet,
        pgo.mesh.volume.MooneyRivlinMaterial(mu01=0.5, mu10=0.3, v1=0.1),
    )
    return volume


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
    return volume


def _make_shell_sim_mesh():
    surface = pgo.mesh.TriMeshData(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64),
        np.array([[0, 1, 2]], dtype=np.int64),
    )
    return pgo.fem.SimulationMesh(surface)


def test_material_state_stores_physical_channels_directly():
    state = pf.MaterialState(
        elastic_values=np.array([3.0]),
        plastic_values=np.array([4.0, 5.0]))

    np.testing.assert_array_equal(state.elastic_values, [3.0])
    np.testing.assert_array_equal(state.plastic_values, [4.0, 5.0])


def test_material_binding_exposes_formal_domain_objects():
    elastic = pf.StableNeoDefinition()
    plastic = pf.VolumetricPlasticityDefinition(dofs=3)

    elastic_domain = pf.ElasticMaterialBinding(
        elastic, 1, [[1.0e6, 0.45]],
    )
    plastic_domain = pf.PlasticMaterialBinding(
        plastic, 1, np.empty((1, 0)),
    )
    binding = pf.MaterialBinding(elastic_domain, plastic_domain)

    assert binding.elastic is elastic_domain
    assert binding.plastic is plastic_domain
    assert elastic_domain.definition is elastic
    assert plastic_domain.definition is plastic
    assert elastic_domain.num_elements == 1
    assert plastic_domain.num_optimizable_channels == 3
    assert binding.num_elements == 1


def test_primary_axes_construct_elementwise_material_frames():
    frames = pf.material_frames_from_primary_axes(
        np.array([[1.0, 0.0, 0.0], [1.0, 2.0, 3.0]]))
    assert isinstance(frames, pf.MaterialFrames)
    assert frames.num_elements == 2
    with pytest.raises(ValueError, match="non-zero"):
        pf.material_frames_from_primary_axes(np.zeros((1, 3)))


def _make_energy(
    sim,
    formulation=pf.TetLinear(),
    elastic=None,
    plastic=None,
    elastic_dof_layout=None,
    plastic_dof_layout=None,
    elastic_values=None,
    plastic_values=None,
    options=None,
):
    elastic = elastic or pf.StableNeoDefinition()
    plastic = plastic or pf.VolumetricPlasticityDefinition(dofs=6)
    del elastic_dof_layout, plastic_dof_layout
    if elastic_values is None and elastic.name == "koiter_stvk":
        elastic_values = np.broadcast_to(
            np.array([[2e6, 0.35, 2e6, 0.35, 0.01]]),
            (sim.num_elements, 5)).copy()
    material = direct_material(
        sim, elastic, plastic, None, None,
        elastic_values, plastic_values)
    operator = pf.DeformationEnergyOperator(
        material.mesh, material.binding,
        formulation=formulation,
        options=options,
    )
    return pf.DeformationPotentialEnergy(
        operator, material.state)


def test_mooney_rivlin_config_builds_deformation_energy():
    sim = _make_mooney_rivlin_tet_sim_mesh()
    energy = _make_energy(
        sim,
        elastic=pf.MooneyRivlinDefinition(),
        plastic=pf.VolumetricPlasticityDefinition(dofs=0),
        formulation=pf.TetLinear(),
    )
    u = energy.zero_state()
    assert np.isclose(energy.value(u), 0.0)
    assert np.all(np.isfinite(energy.gradient(u)))
    assert energy.hessian(u).nnz > 0


def test_neo_hookean_config_builds_deformation_energy():
    energy = _make_energy(
        _make_tet_sim_mesh(),
        elastic=pf.NeoHookeanDefinition(),
        plastic=pf.VolumetricPlasticityDefinition(dofs=0),
        formulation=pf.TetLinear(),
    )
    u = energy.zero_state()
    assert np.isclose(energy.value(u), 0.0)
    assert np.all(np.isfinite(energy.gradient(u)))
    assert energy.hessian(u).nnz > 0


def test_systematic_poking_definition_builds_parameterized_tet_energy():
    stretch_knots = (0.4, 0.7, 1.0, 1.4, 2.0)
    volume_knots = tuple(np.exp((-1.0, -0.5, 0.0, 0.5, 1.0)))
    elastic = pf.SystematicPokingDefinition(
        stretch_knots, 2, volume_knots, 2)
    parameters = np.array(
        [[800.0, 1200.0, 2100.0, 1900.0, 2800.0, 3700.0]],
        dtype=np.float64,
    )
    energy = _make_energy(
        _make_tet_sim_mesh(),
        elastic=elastic,
        plastic=pf.VolumetricPlasticityDefinition(dofs=0),
        formulation=pf.TetLinear(),
            elastic_values=parameters,
    )

    assert energy.elastic_definition is elastic
    assert energy.num_elastic_params == parameters.shape[1]
    assert energy.num_elastic_values == parameters.shape[1]
    np.testing.assert_allclose(
        energy.material_state.elastic_values,
        parameters.ravel(),
    )

    displacement = np.zeros(energy.num_dofs, dtype=np.float64)
    displacement.reshape(-1, 3)[1] = (0.08, 0.01, -0.01)
    displacement.reshape(-1, 3)[2] = (0.02, -0.04, 0.01)
    displacement.reshape(-1, 3)[3] = (-0.01, 0.02, 0.06)
    assert np.isfinite(energy.value(displacement))
    assert np.all(np.isfinite(energy.gradient(displacement)))
    assert energy.hessian(displacement).nnz > 0

    analytic = energy.dE_de(displacement)
    finite_difference = np.empty(parameters.shape[1], dtype=np.float64)
    step = 1e-4
    for parameter_index in range(parameters.shape[1]):
        plus = parameters.copy()
        minus = parameters.copy()
        plus[0, parameter_index] += step
        minus[0, parameter_index] -= step
        plus_state = energy.material_state.with_elastic_values(plus)
        minus_state = energy.material_state.with_elastic_values(minus)
        plus_energy = energy.energy_operator.value(displacement, plus_state)
        minus_energy = energy.energy_operator.value(displacement, minus_state)
        finite_difference[parameter_index] = (
            plus_energy - minus_energy) / (2.0 * step)
    np.testing.assert_allclose(
        analytic, finite_difference, rtol=2e-7, atol=2e-10)


class TestWrappers:
    def test_formulations(self):
        assert pf.TetLinear().name == "tet_linear"
        assert pf.CubicLinear().name == "cubic_linear"
        assert pf.KoiterShell().name == "shell_koiter"
        assert pf.TetLinear().num_basis_functions_per_element() == 4
        assert pf.CubicLinear().num_basis_functions_per_element() == 8
        assert pf.CubicTricubicHermite().num_basis_functions_per_element() == 64
        assert pf.KoiterShell().num_basis_functions_per_element() == 6

    def test_material_ids(self):
        assert pf.StableNeoDefinition().name == "stable_neo"
        assert pf.NeoHookeanDefinition().name == "neo_hookean"
        assert pf.StVKDefinition().name == "stvk"
        assert pf.StVKVolumeDefinition().name == "stvk_vol"
        assert pf.LinearElasticDefinition().name == "linear"
        assert pf.MooneyRivlinDefinition().name == "mooney_rivlin"
        assert pf.KoiterStVKDefinition().name == "koiter_stvk"
        assert pf.VolumetricPlasticityDefinition(dofs=6).name == "volumetric_dof6"
        assert pf.VolumetricPlasticityDefinition(dofs=3).name == "volumetric_dof3"
        assert pf.VolumetricPlasticityDefinition(dofs=0).name == "volumetric_dof0"
        assert pf.ShellPlasticityDefinition(dofs=1).name == "shell_ff_dof1"
        assert pf.ShellPlasticityDefinition(dofs=0).name == "shell_ff_dof0"

    def test_systematic_poking_definition(self):
        stretch_knots = (0.4, 0.7, 1.0, 1.4, 2.0)
        volume_knots = tuple(np.exp((-1.0, -0.5, 0.0, 0.5, 1.0)))
        definition = pf.SystematicPokingDefinition(
            stretch_knots, 2, volume_knots, 2)

        assert definition.name == "systematic_poking"
        assert definition.num_fixed_channels == 0
        assert definition.num_optimizable_channels == 6
        assert definition.stretch_knots == stretch_knots
        assert definition.stretch_rest_knot_index == 2
        assert definition.volume_knots == volume_knots
        assert definition.volume_rest_knot_index == 2

        with pytest.raises(ValueError, match="rest knot must equal one"):
            pf.SystematicPokingDefinition(
                stretch_knots, 1, volume_knots, 2)

    def test_invalid_plastic_dofs(self):
        with pytest.raises(ValueError):
            pf.VolumetricPlasticityDefinition(dofs=1)
        with pytest.raises(ValueError):
            pf.ShellPlasticityDefinition(dofs=2)


class TestDeformationEnergyParameters:
    def test_potential_binds_explicit_material_state(self):
        sim = _make_tet_sim_mesh()
        energy = _make_energy(sim)

        assert isinstance(energy, pf.DeformationPotentialEnergy)
        assert energy.elastic_definition.name == "stable_neo"
        assert energy.plastic_definition.name == "volumetric_dof6"
        assert energy.material_binding.elastic.num_optimizable_channels == 0
        assert energy.material_state.elastic_values.shape == (0,)
        assert energy.material_state.plastic_values.shape == (6,)
        assert np.allclose(
            energy.material_state.plastic_values,
            [1.0, 0.0, 0.0, 1.0, 0.0, 1.0],
        )

    def test_given_plastic_values_are_copied_and_state_is_immutable(self):
        sim = _make_tet_sim_mesh()
        params = np.array([[1.05, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64)
        energy = _make_energy(sim, plastic_values=params)

        params[0, 0] = 9.0
        assert energy.material_state.plastic_values[0] == pytest.approx(1.05)

        updated = np.array([[0.95, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64)
        updated_state = energy.material_state.with_plastic_values(updated)
        assert np.allclose(updated_state.plastic_values, updated)
        assert energy.material_state.plastic_values[0] == pytest.approx(1.05)

    def test_binding_defaults_to_identity_material_frames(self):
        sim = _make_shell_sim_mesh()
        valid = _make_energy(
            sim,
            elastic=pf.KoiterStVKDefinition(),
            plastic=pf.ShellPlasticityDefinition(dofs=1),
            formulation=pf.KoiterShell(),
        )
        binding = pf.MaterialBinding(
            valid.material_binding.elastic,
            valid.material_binding.plastic,
        )
        assert binding.material_frames is None

        mismatched = pf.MaterialFrames(
            np.repeat(np.eye(3)[None, :, :], 2, axis=0))
        with pytest.raises(ValueError, match="frame count"):
            pf.MaterialBinding(
                valid.material_binding.elastic,
                valid.material_binding.plastic,
                mismatched,
            )

    def test_material_state_is_always_elementwise(self):
        sim = _make_tet_sim_mesh()
        params = np.array([[1.05, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64)

        elastic = pf.StableNeoDefinition()
        plastic = pf.VolumetricPlasticityDefinition(dofs=6)
        material = direct_material(
            sim, elastic, plastic, None, None,
            np.empty(0, dtype=np.float64), params)
        operator = pf.DeformationEnergyOperator(
            material.mesh, material.binding,
            formulation=pf.TetLinear(),
        )
        energy = pf.DeformationPotentialEnergy(
            operator, material.state)

        assert energy.num_plastic_values == 6
        assert energy.material_state.plastic_values.shape == (6,)
        assert np.allclose(energy.material_state.plastic_values, params.ravel())

    def test_rejects_wrong_field_shape(self):
        sim = _make_tet_sim_mesh()
        with pytest.raises(ValueError, match="elementwise size"):
            _make_energy(sim, plastic_values=np.ones((sim.num_elements, 3), dtype=np.float64))

    def test_legacy_material_wrappers_do_not_create_fields(self):
        assert not hasattr(pf.StableNeoDefinition(), "default_field")
        assert not hasattr(pf.VolumetricPlasticityDefinition(dofs=6), "elementwise_field")
        assert not hasattr(pf, "ParameterField")
        assert not hasattr(pf, "ElementwiseField")
        assert not hasattr(pf, "ConstantField")

        energy = _make_energy(_make_tet_sim_mesh())
        assert not hasattr(energy, "elastic_field")
        assert not hasattr(energy, "plastic_field")


class TestDeformationPotentialEnergy:
    def test_legacy_parameter_derivative_names_are_not_exposed(self):
        energy = _make_energy(_make_tet_sim_mesh())
        legacy_names = (
            "elastic_gradient",
            "plastic_gradient",
            "elastic_hessian",
            "plastic_hessian",
            "plastic_elastic_hessian",
            "elastic_jacobian",
            "plastic_jacobian",
        )
        assert all(not hasattr(energy, name) for name in legacy_names)

    def test_factory_uses_formulation_handle_not_name(self):
        class MisleadingTetLinear(pf.TetLinear):
            @property
            def name(self):
                return "not_a_formulation"

        sim = _make_tet_sim_mesh()
        energy = _make_energy(sim, formulation=MisleadingTetLinear())
        assert energy.num_dofs == 3 * sim.num_vertices

    def test_element_weights_are_forwarded_to_builder(self):
        sim = _make_tet_sim_mesh()
        energy = _make_energy(
            sim,
            options=pf.DeformationOptions(element_weights=np.array([0.0])),
        )
        u = energy.zero_state()
        u[0] = 0.1

        assert energy.value(u) == pytest.approx(0.0)
        assert np.allclose(energy.gradient(u), 0.0)

    def test_element_weights_require_one_value_per_element(self):
        sim = _make_tet_sim_mesh()
        with pytest.raises(ValueError, match="element_weights size"):
            _make_energy(
                sim,
                options=pf.DeformationOptions(element_weights=np.array([1.0, 1.0])),
            )

    def test_tet_energy_evaluates(self):
        sim = _make_tet_sim_mesh()
        energy = _make_energy(sim)

        assert isinstance(energy, pf.DeformationPotentialEnergy)
        assert isinstance(energy, pe.PotentialEnergy)
        assert energy.num_dofs == 3 * sim.num_vertices
        assert energy.state_kind == "displacement"

        u = energy.zero_state()
        assert u.shape == (energy.num_dofs,)
        assert np.isfinite(energy.value(u))
        assert energy.gradient(u).shape == (energy.num_dofs,)
        H = energy.hessian(u)
        assert H.shape == (energy.num_dofs, energy.num_dofs)
        assert H.nnz > 0
        assert energy.vertex_rest_positions.shape == (sim.num_vertices, 3)
        assert energy.num_vertices == sim.num_vertices

    def test_cubic_energy_evaluates(self):
        sim = _make_cubic_sim_mesh()
        energy = _make_energy(sim, formulation=pf.CubicLinear())
        u = energy.zero_state()
        assert np.isfinite(energy.value(u))
        assert energy.hessian(u).nnz > 0

    def test_hermite_rest_state_and_vertex_positions_are_distinct(self):
        sim = _make_cubic_sim_mesh()
        energy = _make_energy(sim, formulation=pf.CubicTricubicHermite())

        assert energy.rest_state.shape == (energy.num_dofs,)
        assert energy.rest_state.shape == (sim.num_vertices * 24,)
        assert energy.vertex_rest_positions.shape == (sim.num_vertices, 3)

    def test_cubic_energy_exposes_plastic_derivatives(self):
        sim = _make_cubic_sim_mesh()
        plastic = np.array([[1.01, 0.004, -0.003, 0.994, 0.005, 1.008]], dtype=np.float64)
        energy = _make_energy(
            sim,
            formulation=pf.CubicLinear(),
            options=pf.DeformationOptions(project_hessian_psd=False, enable_material_max_step=False),
            plastic_values=plastic,
        )
        u = energy.zero_state()
        for vi in range(sim.num_vertices):
            u[3 * vi + 0] = 5e-3 * np.sin(0.9 * vi + 0.1)
            u[3 * vi + 1] = 4e-3 * np.cos(0.7 * vi + 0.3)
            u[3 * vi + 2] = 3e-3 * np.sin(1.3 * vi + 0.5)

        dE_dp = energy.dE_dp(u)
        adjoint = np.linspace(-0.2, 0.3, energy.num_dofs)
        direct_vjp = energy.plastic_material_vjp(u, adjoint)
        assert dE_dp.shape == (6,)
        assert np.linalg.norm(dE_dp) > 0.0
        assert direct_vjp.shape == (6,)

    def test_shell_energy_exposes_elastic_derivatives(self):
        sim = _make_shell_sim_mesh()
        elastic = np.array([[20000.0, 0.45, 10000.0, 0.3, 1e-3]], dtype=np.float64)
        energy = _make_energy(
            sim,
            formulation=pf.KoiterShell(),
            elastic=pf.KoiterStVKDefinition(),
            elastic_values=elastic,
            plastic=pf.ShellPlasticityDefinition(dofs=1),
            options=pf.DeformationOptions(project_hessian_psd=False, enable_material_max_step=False),
        )
        u = energy.zero_state()
        for vi in range(sim.num_vertices):
            u[3 * vi + 0] = 5e-3 * np.sin(0.9 * vi + 0.1)
            u[3 * vi + 1] = 4e-3 * np.cos(0.7 * vi + 0.3)
            u[3 * vi + 2] = 3e-3 * np.sin(1.3 * vi + 0.5)

        dE_de = energy.dE_de(u)
        adjoint = np.linspace(0.1, 0.4, energy.num_dofs)
        direct_vjp = energy.elastic_material_vjp(u, adjoint)
        assert dE_de.shape == (5,)
        assert np.linalg.norm(dE_de) > 0.0
        assert direct_vjp.shape == (5,)
        combined = energy.material_vjp(u, adjoint)
        assert combined.elastic == pytest.approx(direct_vjp)
        assert combined.plastic.shape == (energy.num_plastic_values,)

    def test_shell_energy_evaluates(self):
        sim = _make_shell_sim_mesh()
        energy = _make_energy(
            sim,
            formulation=pf.KoiterShell(),
            elastic=pf.KoiterStVKDefinition(),
            plastic=pf.ShellPlasticityDefinition(dofs=1),
        )
        u = energy.zero_state()
        assert np.isfinite(energy.value(u))
        assert energy.hessian(u).nnz > 0
        assert energy.dE_de(u).shape == (energy.num_elastic_values,)
        assert energy.material_vjp(
            u, np.ones(energy.num_dofs)).elastic.shape == (
                energy.num_elastic_values,)

    def test_new_state_does_not_mutate_bound_potential(self):
        sim = _make_tet_sim_mesh()
        energy = _make_energy(sim, elastic=pf.StVKDefinition())

        x0 = energy.zero_state()
        before = energy.value(x0)
        changed_state = energy.material_state.with_plastic_values(
            np.array([[1.05, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64)
        )
        after = energy.energy_operator.value(x0, changed_state)
        assert after != pytest.approx(before, abs=1e-15)
        assert energy.value(x0) == pytest.approx(before)

    def test_static_solve_uses_given_plastic_params(self):
        sim = _make_tet_sim_mesh()
        energy = _make_energy(
            sim,
            elastic=pf.StVKDefinition(),
            plastic_values=np.array([[1.05, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64),
        )

        x0 = energy.zero_state()
        fixed_dofs = [dof for dof in range(energy.num_dofs) if dof != 3]
        problem = ps.OptimizationProblem(objective=energy)
        problem.fix_variables(fixed_dofs, x0[fixed_dofs], num_dofs=x0.size)
        result = ps.NewtonOptimizer(max_iterations=50, gradient_tolerance=1e-8).solve(problem, x0)

        assert result.converged
        assert result.x[3] > 1e-4
        assert np.allclose(result.x[fixed_dofs], 0.0)

    def test_energy_set_compatible(self):
        sim = _make_tet_sim_mesh()
        energy = _make_energy(sim)
        es = pe.EnergySet([(energy, 1.0)])
        assert es.num_terms == 1
        assert es.num_dofs == energy.num_dofs


class TestLifetimeAndErrors:
    def test_energy_survives_mesh_and_state_deletion(self):
        sim = _make_tet_sim_mesh()
        energy = _make_energy(sim)
        u = energy.zero_state()
        before = energy.value(u)

        del sim
        gc.collect()

        assert energy.value(u) == pytest.approx(before, rel=1e-15)

    def test_wrong_inputs_raise(self):
        sim = _make_tet_sim_mesh()
        valid = _make_energy(sim)
        mesh = valid.mesh
        binding = valid.material_binding

        with pytest.raises(TypeError, match="material_binding"):
            pf.DeformationEnergyOperator(sim, formulation=pf.TetLinear())
        with pytest.raises(ValueError, match="formulation is required"):
            pf.DeformationEnergyOperator(mesh, binding, formulation=None)
        with pytest.raises(TypeError, match="formulation must be"):
            pf.DeformationEnergyOperator(
                mesh, binding,
                formulation="tet_linear",
            )
        with pytest.raises(TypeError, match="options must be"):
            pf.DeformationEnergyOperator(
                mesh, binding,
                formulation=pf.TetLinear(),
                options={},
            )
        with pytest.raises(TypeError):
            pf.DeformationEnergyOperator(mesh, object(), formulation=pf.TetLinear())

    def test_no_legacy_energy_keywords(self):
        sim = _make_tet_sim_mesh()
        energy = _make_energy(sim)
        with pytest.raises(TypeError):
            pf.DeformationEnergyOperator(
                energy.mesh, energy.material_binding,
                formulation=pf.TetLinear(),
                elastic=pf.StableNeoDefinition(),
            )


class TestModuleSurface:
    def test_module_surface(self):
        assert hasattr(pgo, "fem")
        assert hasattr(pgo, "energy")
        assert not hasattr(pe, "deformation_energy")
        assert not hasattr(pe, "TetLinear")
        assert not hasattr(pe, "StableNeoDefinition")
        assert hasattr(pe, "PotentialEnergy")
        # FEM energy classes live with their domain module (pypgo.fem),
        # not in the generic pypgo.energy namespace.
        assert not hasattr(pe, "DeformationEnergy")
        assert hasattr(pf, "DeformationEnergyOperator")
        assert hasattr(pf, "DeformationPotentialEnergy")
        assert hasattr(pf, "MaterialState")
        assert not hasattr(pf, "DeformationEnergy")
        assert not hasattr(pf, "OptimizableParameters")
        assert not hasattr(pf, "deformation_energy")
        assert not hasattr(pf, "DeformationModelConfig")
        assert not hasattr(pf, "deformation_model_config")


# ---------------------------------------------------------------------------
# Handle architecture: FEM energy peer
# ---------------------------------------------------------------------------


def test_deformation_energy_handle_is_concrete_peer():
    import pypgo._core as _core

    sim = _make_tet_sim_mesh()
    e = _make_energy(sim)

    assert isinstance(e._handle, _core.PyDeformationPotentialEnergy)
    assert isinstance(e._handle, _core.PyPotentialEnergy)
    assert isinstance(
        e.energy_operator._handle, _core.PyDeformationEnergyOperator)


# ---------------------------------------------------------------------------
# Task 11: element_von_mises binding tests
# ---------------------------------------------------------------------------


class TestElementVonMises:
    """Tests for DeformationPotentialEnergy.element_von_mises."""

    def test_zero_displacement_gives_near_zero_stress(self):
        """Zero displacement -> all von Mises stresses should be ~0."""
        sim = _make_tet_sim_mesh()
        energy = _make_energy(sim)
        num_elements = sim.num_elements
        u_zero = np.zeros(energy.num_dofs, dtype=np.float64)

        stresses = energy.element_von_mises(u_zero)

        assert stresses.shape == (num_elements,), (
            f"Expected ({num_elements},), got {stresses.shape}"
        )
        assert np.all(stresses < 1e-6), (
            f"Expected near-zero stresses, got max={stresses.max()}"
        )

    def test_nonzero_displacement_gives_positive_stress(self):
        """Non-symmetric stretch displacement -> all stresses > 0.

        A pure isotropic stretch has zero deviatoric (von Mises) stress,
        so we use a uniaxial stretch (vertex 1 displaced along x) which
        produces a nonzero deviatoric state.
        """
        sim = _make_tet_sim_mesh()
        energy = _make_energy(sim)
        num_elements = sim.num_elements

        # Displace vertex 1 by 0.1 along x (uniaxial stretch — nonzero von Mises)
        u = np.zeros(energy.num_dofs, dtype=np.float64)
        u[3] = 0.1  # vertex 1, x-DOF

        stresses = energy.element_von_mises(u)

        assert stresses.shape == (num_elements,), (
            f"Expected ({num_elements},), got {stresses.shape}"
        )
        assert np.all(stresses > 0), (
            f"Expected all stresses > 0 under uniaxial stretch, got min={stresses.min()}"
        )

    def test_output_length_equals_num_elements(self):
        """Output length matches sim_mesh.num_elements."""
        sim = _make_tet_sim_mesh()
        energy = _make_energy(sim)
        u = np.zeros(energy.num_dofs, dtype=np.float64)

        stresses = energy.element_von_mises(u)
        assert len(stresses) == sim.num_elements


# ---------------------------------------------------------------------------
# Task 12: shell von Mises stress recovery tests
# ---------------------------------------------------------------------------

SHELL_OBJ = (
    __import__("pathlib").Path(__file__).parent.parent.parent
    / "examples" / "assets" / "obj" / "shell.obj"
)


def _make_shell_energy_full():
    """Shell energy on shell.obj with KoiterStVKDefinition for von Mises testing."""
    surface = pgo.mesh.read_obj(str(SHELL_OBJ))
    sim = pf.SimulationMesh(surface)
    energy = _make_energy(
        sim,
        elastic=pf.KoiterStVKDefinition(),
        plastic=pf.ShellPlasticityDefinition(dofs=0),
        elastic_values=np.array(
            [[1e6, 0.3, 1e6, 0.3, 0.01]], dtype=np.float64),
        formulation=pf.KoiterShell(),
    )
    return sim, energy, surface.vertices


class TestShellVonMisesStress:
    """Task 12: shell von Mises stress recovery (KoiterStVKDefinition).

    These tests FAIL before the C++ implementation and pass after rebuild.
    """

    def test_rest_gives_near_zero_stress(self):
        """Zero displacement -> all element stresses ~ 0."""
        sim, energy, _verts = _make_shell_energy_full()
        u = energy.zero_state()
        stresses = energy.element_von_mises(u)

        assert stresses.shape == (sim.num_elements,)
        assert np.all(stresses < 1e-6), (
            f"Expected near-zero stresses at rest, got max={stresses.max()}"
        )

    def test_inplane_stretch_gives_positive_stress(self):
        """In-plane stretch u_x = 0.05 * x per vertex -> all stresses > 0.

        shell.obj lies in the z=0 plane with x in [-0.5, 0.5].
        Membrane strain ~ 0.05, so stresses should be O(E_m * 0.05) ~ 5e4.
        We require each element stress to be between 1e3 and 1e6.
        """
        sim, energy, verts = _make_shell_energy_full()
        u = np.zeros(energy.num_dofs, dtype=np.float64)
        for vi in range(sim.num_vertices):
            u[3 * vi + 0] = 0.05 * verts[vi, 0]  # u_x = 0.05 * x

        stresses = energy.element_von_mises(u)

        assert stresses.shape == (sim.num_elements,)
        assert np.all(stresses > 0), (
            f"Expected all stresses > 0 under in-plane stretch, got min={stresses.min()}"
        )
        assert np.all(stresses > 1e3), (
            f"Expected stresses > 1e3 (O(E_m*strain)), got min={stresses.min()}"
        )
        assert np.all(stresses < 1e6), (
            f"Expected stresses < 1e6, got max={stresses.max()}"
        )

    def test_pure_bending_gives_positive_stress(self):
        """Pure bending u_z = 0.1 * x^2 -> max stress > 0.

        shell.obj lies in z=0; moving vertices out-of-plane quadratically
        creates curvature (bending), which the StVKDefinition shell bending energy
        should convert to nonzero von Mises stress.
        """
        sim, energy, verts = _make_shell_energy_full()
        u = np.zeros(energy.num_dofs, dtype=np.float64)
        for vi in range(sim.num_vertices):
            u[3 * vi + 2] = 0.1 * verts[vi, 0] ** 2  # u_z = 0.1 * x^2

        stresses = energy.element_von_mises(u)

        assert stresses.shape == (sim.num_elements,)
        assert np.max(stresses) > 0, (
            f"Expected max stress > 0 under bending, got max={stresses.max()}"
        )
