"""Public tests for the config-based pypgo.fem deformation API."""

import gc

import numpy as np
import pypgo as pgo
import pypgo.energy as pe
import pypgo.fem as pf
import pypgo.solver as ps
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
    return pgo.fem.SimulationMesh.create_volumetric(volume)


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
    return pgo.fem.SimulationMesh.create_volumetric(volume)


def _make_shell_sim_mesh():
    surface = pgo.mesh.TriMeshData(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64),
        np.array([[0, 1, 2]], dtype=np.int64),
    )
    material = pgo.fem.KoiterStVKShellMaterial(thickness=0.01, E_membrane=2e6, nu_membrane=0.35)
    return pgo.fem.SimulationMesh.create_shell(surface, material)


def _make_energy(
    sim,
    formulation=pf.TetLinear(),
    elastic=None,
    plastic=None,
    elastic_layout=None,
    plastic_layout=None,
    elastic_values=None,
    plastic_values=None,
    options=None,
):
    elastic = elastic or pf.StableNeo()
    plastic = plastic or pf.VolumetricPlasticity(dofs=6)
    return pf.deformation_energy(
        sim,
        elastic=elastic,
        elastic_layout=elastic_layout or pf.ElementwiseDofLayout(),
        elastic_values=elastic_values,
        plastic=plastic,
        plastic_layout=plastic_layout or pf.ElementwiseDofLayout(),
        plastic_values=plastic_values,
        formulation=formulation,
        options=options,
    )


class TestWrappers:
    def test_formulations(self):
        assert pf.TetLinear().name == "tet_linear"
        assert pf.CubicLinear().name == "cubic_linear"
        assert pf.KoiterShell().name == "shell_koiter"

    def test_material_ids(self):
        assert pf.StableNeo().name == "stable_neo"
        assert pf.StVK().name == "stvk"
        assert pf.StVKVolume().name == "stvk_vol"
        assert pf.LinearElastic().name == "linear"
        assert pf.MooneyRivlin().name == "mooney_rivlin"
        assert pf.KoiterStVK().name == "koiter_stvk"
        assert pf.VolumetricPlasticity(dofs=6).name == "volumetric_dof6"
        assert pf.VolumetricPlasticity(dofs=3).name == "volumetric_dof3"
        assert pf.VolumetricPlasticity(dofs=0).name == "volumetric_dof0"
        assert pf.ShellPlasticity(dofs=1).name == "shell_ff_dof1"
        assert pf.ShellPlasticity(dofs=0).name == "shell_ff_dof0"

    def test_invalid_plastic_dofs(self):
        with pytest.raises(ValueError):
            pf.VolumetricPlasticity(dofs=1)
        with pytest.raises(ValueError):
            pf.ShellPlasticity(dofs=2)


class TestDeformationEnergyParameters:
    def test_default_energy_owns_parameters(self):
        sim = _make_tet_sim_mesh()
        energy = _make_energy(sim)

        assert isinstance(energy, pf.DeformationEnergy)
        assert energy.elastic_model == "stable_neo"
        assert energy.plastic_model == "volumetric_dof6"
        assert energy.parameters.space.elastic.kind == "elastic"
        assert energy.parameters.space.elastic.num_channels == 0
        assert energy.parameters.elastic_values.shape == (0, 0)
        assert energy.parameters.space.plastic.kind == "plastic"
        assert energy.parameters.plastic_values.shape == (sim.num_elements, 6)
        assert np.allclose(
            energy.parameters.plastic_values,
            [[1.0, 0.0, 0.0, 1.0, 0.0, 1.0]],
        )

    def test_given_plastic_values_are_copied_and_mutable_through_energy(self):
        sim = _make_tet_sim_mesh()
        params = np.array([[1.05, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64)
        energy = _make_energy(sim, plastic_values=params)

        params[0, 0] = 9.0
        assert energy.parameters.plastic_values[0, 0] == pytest.approx(1.05)

        updated = np.array([[0.95, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64)
        energy.parameters.set_plastic_values(updated)
        assert np.allclose(energy.parameters.plastic_values, updated)

    def test_given_elastic_values_use_cpp_channel_count(self):
        class KoiterFabric:
            def _to_string(self):
                return "koiter_fabric"

        sim = _make_shell_sim_mesh()
        params = np.array(
            [[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1000.0, 1000.0, 1000.0, 1.0, 0.01]],
            dtype=np.float64,
        )

        energy = pf.deformation_energy(
            sim,
            elastic=KoiterFabric(),
            elastic_layout=pf.ElementwiseDofLayout(),
            elastic_values=params,
            plastic=pf.ShellPlasticity(dofs=1),
            plastic_layout=pf.ElementwiseDofLayout(),
            formulation=pf.KoiterShell(),
        )

        assert energy.elastic_model == "koiter_fabric"
        assert energy.parameters.elastic_values.shape == (sim.num_elements, 12)
        assert np.allclose(energy.parameters.elastic_values, params)

    def test_constant_field_reports_shared_value_row(self):
        sim = _make_tet_sim_mesh()
        params = np.array([[1.05, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64)

        energy = pf.deformation_energy(
            sim,
            elastic=pf.StableNeo(),
            elastic_layout=pf.ElementwiseDofLayout(),
            plastic=pf.VolumetricPlasticity(dofs=6),
            plastic_layout=pf.ConstantDofLayout(),
            plastic_values=params,
            formulation=pf.TetLinear(),
        )

        assert energy.parameters.space.plastic.num_value_rows == 1
        assert energy.parameters.plastic_values.shape == (1, 6)
        assert np.allclose(energy.parameters.plastic_values, params)

    def test_rejects_wrong_field_shape(self):
        sim = _make_tet_sim_mesh()
        with pytest.raises(ValueError, match="shape must be"):
            _make_energy(sim, plastic_values=np.ones((sim.num_elements, 3), dtype=np.float64))

    def test_legacy_material_wrappers_do_not_create_fields(self):
        assert not hasattr(pf.StableNeo(), "default_field")
        assert not hasattr(pf.VolumetricPlasticity(dofs=6), "elementwise_field")
        assert not hasattr(pf, "ParameterField")
        assert not hasattr(pf, "ElementwiseField")
        assert not hasattr(pf, "ConstantField")

        energy = _make_energy(_make_tet_sim_mesh())
        assert not hasattr(energy, "elastic_field")
        assert not hasattr(energy, "plastic_field")


class TestDeformationEnergy:
    def test_tet_energy_evaluates(self):
        sim = _make_tet_sim_mesh()
        energy = _make_energy(sim)

        assert isinstance(energy, pf.DeformationEnergy)
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
        assert energy.rest_position.shape == (sim.num_vertices, 3)
        assert energy.num_vertices == sim.num_vertices

    def test_cubic_energy_evaluates(self):
        sim = _make_cubic_sim_mesh()
        energy = _make_energy(sim, formulation=pf.CubicLinear())
        u = energy.zero_state()
        assert np.isfinite(energy.value(u))
        assert energy.hessian(u).nnz > 0

    def test_cubic_energy_exposes_plastic_derivatives_and_material_energy(self):
        sim = _make_cubic_sim_mesh()
        plastic = np.array([[1.01, 0.004, -0.003, 0.994, 0.005, 1.008]], dtype=np.float64)
        energy = _make_energy(
            sim,
            formulation=pf.CubicLinear(),
            options=pf.DeformationOptions(enforce_spd=False, enable_material_max_step=False),
            plastic_values=plastic,
        )
        u = energy.zero_state()
        for vi in range(sim.num_vertices):
            u[3 * vi + 0] = 5e-3 * np.sin(0.9 * vi + 0.1)
            u[3 * vi + 1] = 4e-3 * np.cos(0.7 * vi + 0.3)
            u[3 * vi + 2] = 3e-3 * np.sin(1.3 * vi + 0.5)

        grad = energy.plastic_gradient(u)
        hess = energy.plastic_hessian(u)
        jac = energy.plastic_jacobian(u)
        assert grad.shape == (6,)
        assert hess.shape == (6, 6)
        assert jac.shape == (energy.num_dofs, 6)
        assert np.linalg.norm(grad) > 0.0
        assert hess.nnz > 0
        assert jac.nnz > 0

        material_energy = pf.plastic_material_energy(energy, fixed_displacement=u)
        assert isinstance(material_energy, pe.PotentialEnergy)
        assert material_energy.num_dofs == 6
        assert material_energy.state_kind == "generic"
        assert np.isclose(material_energy.value(plastic.ravel()), energy.value(u))
        assert np.allclose(material_energy.gradient(plastic.ravel()), grad)
        assert np.allclose(material_energy.hessian(plastic.ravel()).to_dense(), hess.to_dense())

    def test_shell_energy_exposes_elastic_material_energy(self):
        sim = _make_shell_sim_mesh()
        elastic = np.array([[20000.0, 0.45, 10000.0, 0.3, 1e-3]], dtype=np.float64)
        energy = _make_energy(
            sim,
            formulation=pf.KoiterShell(),
            elastic=pf.KoiterStVK(),
            elastic_layout=pf.ConstantDofLayout(),
            elastic_values=elastic,
            plastic=pf.ShellPlasticity(dofs=1),
            plastic_layout=pf.ElementwiseDofLayout(),
            options=pf.DeformationOptions(enforce_spd=False, enable_material_max_step=False),
        )
        u = energy.zero_state()
        for vi in range(sim.num_vertices):
            u[3 * vi + 0] = 5e-3 * np.sin(0.9 * vi + 0.1)
            u[3 * vi + 1] = 4e-3 * np.cos(0.7 * vi + 0.3)
            u[3 * vi + 2] = 3e-3 * np.sin(1.3 * vi + 0.5)

        grad = energy.elastic_gradient(u)
        hess = energy.elastic_hessian(u)
        material_energy = pf.elastic_material_energy(energy, fixed_displacement=u)

        assert isinstance(material_energy, pe.PotentialEnergy)
        assert isinstance(material_energy, pf.ElasticMaterialEnergy)
        assert material_energy.num_dofs == 5
        assert material_energy.state_kind == "generic"
        assert np.isclose(material_energy.value(elastic.ravel()), energy.value(u))
        assert np.allclose(material_energy.gradient(elastic.ravel()), grad)
        assert np.allclose(material_energy.hessian(elastic.ravel()).to_dense(), hess.to_dense())

    def test_shell_energy_evaluates(self):
        sim = _make_shell_sim_mesh()
        energy = _make_energy(
            sim,
            formulation=pf.KoiterShell(),
            elastic=pf.KoiterStVK(),
            plastic=pf.ShellPlasticity(dofs=1),
        )
        u = energy.zero_state()
        assert np.isfinite(energy.value(u))
        assert energy.hessian(u).nnz > 0
        assert energy.elastic_gradient(u).shape == (energy.num_elastic_dofs,)
        assert energy.elastic_hessian(u).shape == (
            energy.num_elastic_dofs,
            energy.num_elastic_dofs,
        )
        assert energy.plastic_elastic_hessian(u).shape == (
            energy.num_plastic_dofs,
            energy.num_elastic_dofs,
        )

    def test_energy_observes_committed_parameter_update(self):
        sim = _make_tet_sim_mesh()
        energy = _make_energy(sim, elastic=pf.StVK())

        x0 = energy.zero_state()
        before = energy.value(x0)
        energy.parameters.set_plastic_values(
            np.array([[1.05, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64)
        )
        after = energy.value(x0)
        assert after != pytest.approx(before, abs=1e-15)

    def test_static_solve_uses_given_plastic_params(self):
        sim = _make_tet_sim_mesh()
        energy = _make_energy(
            sim,
            elastic=pf.StVK(),
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

        with pytest.raises(TypeError, match="elastic"):
            pf.deformation_energy(sim, formulation=pf.TetLinear())
        with pytest.raises(ValueError, match="formulation is required"):
            pf.deformation_energy(
                sim,
                elastic=pf.StableNeo(),
                plastic=pf.VolumetricPlasticity(dofs=6),
            )
        with pytest.raises(TypeError, match="formulation must be"):
            pf.deformation_energy(
                sim,
                elastic=pf.StableNeo(),
                plastic=pf.VolumetricPlasticity(dofs=6),
                formulation="tet_linear",
            )
        with pytest.raises(TypeError, match="options must be"):
            pf.deformation_energy(
                sim,
                elastic=pf.StableNeo(),
                plastic=pf.VolumetricPlasticity(dofs=6),
                formulation=pf.TetLinear(),
                options={},
            )
        with pytest.raises(TypeError, match="ParameterDofLayout"):
            pf.deformation_energy(
                sim,
                elastic=pf.StableNeo(),
                elastic_layout=object(),
                plastic=pf.VolumetricPlasticity(dofs=6),
                formulation=pf.TetLinear(),
            )

    def test_no_legacy_energy_keywords(self):
        sim = _make_tet_sim_mesh()
        with pytest.raises(TypeError):
            pf.deformation_energy(
                sim,
                elastic=pf.StableNeo(),
                plastic=pf.VolumetricPlasticity(dofs=6),
                formulation=pf.TetLinear(),
                elastic_field=object(),
            )


class TestModuleSurface:
    def test_module_surface(self):
        assert hasattr(pgo, "fem")
        assert hasattr(pgo, "energy")
        assert not hasattr(pe, "deformation_energy")
        assert not hasattr(pe, "TetLinear")
        assert not hasattr(pe, "StableNeo")
        assert hasattr(pe, "PotentialEnergy")
        # FEM energy classes live with their domain module (pypgo.fem),
        # not in the generic pypgo.energy namespace.
        assert not hasattr(pe, "DeformationEnergy")
        assert hasattr(pf, "DeformationEnergy")
        assert not hasattr(pf, "DeformationModelConfig")
        assert not hasattr(pf, "deformation_model_config")


# ---------------------------------------------------------------------------
# Handle architecture: FEM energy peer
# ---------------------------------------------------------------------------


def test_deformation_energy_handle_is_concrete_peer():
    import pypgo._core as _core

    sim = _make_tet_sim_mesh()
    e = _make_energy(sim)

    assert isinstance(e._handle, _core.PyDeformationEnergy)
    assert isinstance(e._handle, _core.PyPotentialEnergy)


# ---------------------------------------------------------------------------
# Task 11: element_von_mises binding tests
# ---------------------------------------------------------------------------


class TestElementVonMises:
    """Tests for DeformationEnergy.element_von_mises (new C++ binding)."""

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
    """Shell energy on shell.obj with KoiterStVK for von Mises testing."""
    surface = pgo.mesh.read_obj(str(SHELL_OBJ))
    material = pf.KoiterStVKShellMaterial(thickness=0.01, E_membrane=1e6, nu_membrane=0.3)
    sim = pf.SimulationMesh.create_shell(surface, material)
    energy = pf.deformation_energy(
        sim,
        elastic=pf.KoiterStVK(),
        plastic=pf.ShellPlasticity(dofs=0),
        formulation=pf.KoiterShell(),
    )
    return sim, energy, surface.vertices


class TestShellVonMisesStress:
    """Task 12: shell von Mises stress recovery (KoiterStVK).

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
        creates curvature (bending), which the StVK shell bending energy
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
