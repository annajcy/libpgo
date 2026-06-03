"""Public tests for pypgo.fem / pypgo.energy.DeformationEnergy.

Covers:
  - Formulation wrappers (TetP1, LinearCubic, KoiterShell)
  - Elastic law wrappers (StableNeo, StVK, etc.)
  - Plastic wrappers (VolumetricPlasticity, ShellPlasticity)
  - DeformationOptions
  - deformation_energy() factory
  - DeformationEnergy properties (rest_position, num_vertices)
  - Energy evaluation (value, gradient, hessian) at zero state
  - EnergySet compatibility (isinstance, composition)
  - state_kind == "displacement"
  - Two energies from same mesh
  - Mesh lifetime after energy creation
  - Error cases (missing formulation, wrong type, etc.)
"""

import gc

import numpy as np
import pypgo as pgo
import pypgo.energy as pe
import pypgo.fem as pf
import pypgo._core as _core
import pypgo.solver as ps
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
# Formulation wrappers
# ---------------------------------------------------------------------------

class TestFormulations:
    def test_tet_p1_default(self):
        f = pf.TetP1()
        assert f._to_string() == "tet_p1"

    def test_linear_cubic(self):
        f = pf.LinearCubic()
        assert f._to_string() == "hex_trilinear"

    def test_koiter_shell(self):
        f = pf.KoiterShell()
        assert f._to_string() == "shell_koiter"


# ---------------------------------------------------------------------------
# Elastic law wrappers
# ---------------------------------------------------------------------------

class TestElasticLaws:
    def test_stable_neo(self):
        assert pf.StableNeo()._to_string() == "stable_neo"

    def test_stvk(self):
        assert pf.StVK()._to_string() == "stvk"

    def test_stvk_volume(self):
        assert pf.StVKVolume()._to_string() == "stvk_vol"

    def test_linear_elastic(self):
        assert pf.LinearElastic()._to_string() == "linear"

    def test_mooney_rivlin(self):
        assert pf.MooneyRivlin()._to_string() == "mooney_rivlin"

    def test_koiter_stvk(self):
        assert pf.KoiterStVK()._to_string() == "koiter_stvk"


# ---------------------------------------------------------------------------
# Plastic wrappers
# ---------------------------------------------------------------------------

class TestPlasticParams:
    def test_volumetric_default(self):
        p = pf.VolumetricPlasticity()
        assert p.dofs == 6
        assert p._to_string() == "volumetric_dof6"

    def test_volumetric_dof3(self):
        p = pf.VolumetricPlasticity(dofs=3)
        assert p._to_string() == "volumetric_dof3"

    def test_volumetric_dof0(self):
        p = pf.VolumetricPlasticity(dofs=0)
        assert p._to_string() == "volumetric_dof0"

    def test_volumetric_invalid_raises(self):
        with pytest.raises(ValueError):
            pf.VolumetricPlasticity(dofs=1)

    def test_shell_default(self):
        p = pf.ShellPlasticity()
        assert p.dofs == 1
        assert p._to_string() == "shell_ff_dof1"

    def test_shell_dof0(self):
        p = pf.ShellPlasticity(dofs=0)
        assert p._to_string() == "shell_ff_dof0"

    def test_shell_invalid_raises(self):
        with pytest.raises(ValueError):
            pf.ShellPlasticity(dofs=2)


# ---------------------------------------------------------------------------
# DeformationOptions
# ---------------------------------------------------------------------------

class TestDeformationOptions:
    def test_defaults(self):
        opts = pf.DeformationOptions()
        assert opts.enforce_spd is True
        assert opts.enable_material_max_step is True

    def test_custom(self):
        opts = pf.DeformationOptions(enforce_spd=False, enable_material_max_step=False)
        assert opts.enforce_spd is False
        assert opts.enable_material_max_step is False


# ---------------------------------------------------------------------------
# Tet P1 deformation energy
# ---------------------------------------------------------------------------

class TestTetDeformationEnergy:
    def test_builds_with_default_formulation(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            formulation=pf.TetP1(),
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        assert isinstance(energy, pe.DeformationEnergy)
        assert energy.num_dofs == 3 * sim.num_vertices

    def test_omitted_formulation_defaults_to_tet_p1(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        assert isinstance(energy, pe.DeformationEnergy)
        assert energy.num_dofs == 3 * sim.num_vertices

    def test_state_kind_is_displacement(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        assert energy.state_kind == "displacement"

    def test_zero_state(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        u = energy.zero_state()
        assert isinstance(u, np.ndarray)
        assert u.dtype == np.float64
        assert u.shape == (energy.num_dofs,)
        assert np.all(u == 0.0)

    def test_value_at_zero_state(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        u = energy.zero_state()
        val = energy.value(u)
        assert isinstance(val, float)
        assert np.isfinite(val)

    def test_gradient_at_zero_state(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        u = energy.zero_state()
        g = energy.gradient(u)
        assert isinstance(g, np.ndarray)
        assert g.shape == (energy.num_dofs,)
        assert all(np.isfinite(v) for v in g)

    def test_hessian_at_zero_state(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        u = energy.zero_state()
        H = energy.hessian(u)
        assert H.shape[0] == energy.num_dofs
        assert H.shape[1] == energy.num_dofs
        assert H.nnz > 0

    def test_value_changes_under_perturbation(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        u0 = energy.zero_state()
        val0 = energy.value(u0)
        u_pert = u0.copy()
        u_pert[0] = 0.01
        val_pert = energy.value(u_pert)
        assert val_pert != pytest.approx(val0, abs=1e-15)

    def test_rest_position_shape(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        rp = energy.rest_position
        assert isinstance(rp, np.ndarray)
        assert rp.ndim == 2
        assert rp.shape == (sim.num_vertices, 3)
        assert rp.dtype == np.float64

    def test_num_vertices(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        assert energy.num_vertices == sim.num_vertices

    def test_isinstance_potential_energy(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        assert isinstance(energy, pe.PotentialEnergy)

    def test_with_stvk_material(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            formulation=pf.TetP1(),
            elastic=pf.StVK(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        u = energy.zero_state()
        val = energy.value(u)
        assert np.isfinite(val)

    def test_with_options(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
            options=pf.DeformationOptions(enforce_spd=False),
        )
        u = energy.zero_state()
        assert np.isfinite(energy.value(u))

    def test_wrong_input_size_raises(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        with pytest.raises(ValueError, match="State size mismatch"):
            energy.value(np.zeros(3, dtype=np.float64))

    def test_energy_set_compatible(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        # Can compose with EnergySet
        es = pe.EnergySet([(energy, 1.0)])
        assert es.num_terms == 1
        assert es.num_dofs == energy.num_dofs
        assert es.state_kind == "displacement"

    def test_plastic_params_roundtrip(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )

        default_params = energy.plastic_params
        assert default_params.shape == (sim.num_elements, 6)
        assert np.allclose(default_params, [[1.0, 0.0, 0.0, 1.0, 0.0, 1.0]])

        updated = np.array([[1.05, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64)
        energy.set_plastic_params(updated)

        assert np.allclose(energy.plastic_params, updated)
        updated[0, 0] = 2.0
        assert energy.plastic_params[0, 0] == pytest.approx(1.05)

    def test_factory_accepts_initial_plastic_params(self):
        sim = _make_tet_sim_mesh()
        params = np.array([[1.05, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64)

        energy = pf.deformation_energy(
            sim,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
            plastic_params=params,
        )

        assert np.allclose(energy.plastic_params, params)

    def test_static_solve_uses_given_plastic_params(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            elastic=pf.StVK(),
            plastic=pf.VolumetricPlasticity(dofs=6),
            plastic_params=np.array([[1.05, 0.0, 0.0, 1.0, 0.0, 1.0]], dtype=np.float64),
        )

        x0 = energy.zero_state()
        fixed_dofs = [dof for dof in range(energy.num_dofs) if dof != 3]
        problem = ps.OptimizationProblem(objective=energy)
        problem.fix_variables(fixed_dofs, x0[fixed_dofs], num_dofs=x0.size)
        result = ps.NewtonOptimizer(max_iterations=50, gradient_tolerance=1e-8).solve(problem, x0)

        assert result.converged
        assert result.x[3] > 1e-4
        assert np.allclose(result.x[fixed_dofs], 0.0)


# ---------------------------------------------------------------------------
# Cubic HexTrilinear deformation energy
# ---------------------------------------------------------------------------

class TestCubicDeformationEnergy:
    def test_builds_with_explicit_formulation(self):
        sim = _make_cubic_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            formulation=pf.LinearCubic(),
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        assert isinstance(energy, pe.DeformationEnergy)
        assert energy.num_dofs == 3 * sim.num_vertices

    def test_cubic_without_formulation_raises(self):
        sim = _make_cubic_sim_mesh()
        with pytest.raises(ValueError, match="requires an explicit formulation"):
            pf.deformation_energy(
                sim,
                elastic=pf.StableNeo(),
                plastic=pf.VolumetricPlasticity(dofs=6),
            )

    def test_cubic_wrong_formulation_raises(self):
        sim = _make_cubic_sim_mesh()
        with pytest.raises(ValueError, match="requires LinearCubic"):
            pf.deformation_energy(
                sim,
                formulation=pf.TetP1(),
                elastic=pf.StableNeo(),
                plastic=pf.VolumetricPlasticity(dofs=6),
            )

    def test_state_kind_displacement(self):
        sim = _make_cubic_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            formulation=pf.LinearCubic(),
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        assert energy.state_kind == "displacement"

    def test_value_gradient_hessian(self):
        sim = _make_cubic_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            formulation=pf.LinearCubic(),
            elastic=pf.StVK(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        u = energy.zero_state()
        assert np.isfinite(energy.value(u))
        g = energy.gradient(u)
        assert len(g) == energy.num_dofs
        H = energy.hessian(u)
        assert H.nnz > 0

    def test_rest_position(self):
        sim = _make_cubic_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            formulation=pf.LinearCubic(),
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        rp = energy.rest_position
        assert rp.shape == (sim.num_vertices, 3)
        assert np.any(np.abs(rp) > 1e-12)


# ---------------------------------------------------------------------------
# Shell Koiter deformation energy
# ---------------------------------------------------------------------------

class TestShellDeformationEnergy:
    def test_builds_with_shell_formulation(self):
        sim = _make_shell_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            formulation=pf.KoiterShell(),
            elastic=pf.KoiterStVK(),
            plastic=pf.ShellPlasticity(dofs=1),
        )
        assert isinstance(energy, pe.DeformationEnergy)
        assert energy.num_dofs == 3 * sim.num_vertices

    def test_shell_without_formulation_raises(self):
        sim = _make_shell_sim_mesh()
        with pytest.raises(ValueError, match="requires an explicit formulation"):
            pf.deformation_energy(
                sim,
                elastic=pf.KoiterStVK(),
                plastic=pf.ShellPlasticity(dofs=1),
            )

    def test_shell_wrong_formulation_raises(self):
        sim = _make_shell_sim_mesh()
        with pytest.raises(ValueError, match="requires KoiterShell"):
            pf.deformation_energy(
                sim,
                formulation=pf.TetP1(),
                elastic=pf.KoiterStVK(),
                plastic=pf.ShellPlasticity(dofs=1),
            )

    def test_state_kind_displacement(self):
        sim = _make_shell_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            formulation=pf.KoiterShell(),
            elastic=pf.KoiterStVK(),
            plastic=pf.ShellPlasticity(dofs=1),
        )
        assert energy.state_kind == "displacement"

    def test_value_gradient_hessian(self):
        sim = _make_shell_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            formulation=pf.KoiterShell(),
            elastic=pf.KoiterStVK(),
            plastic=pf.ShellPlasticity(dofs=1),
        )
        u = energy.zero_state()
        assert np.isfinite(energy.value(u))
        g = energy.gradient(u)
        assert len(g) == energy.num_dofs
        H = energy.hessian(u)
        assert H.nnz > 0

    def test_shell_plasticity_dof0(self):
        sim = _make_shell_sim_mesh()
        energy = pf.deformation_energy(
            sim,
            formulation=pf.KoiterShell(),
            elastic=pf.KoiterStVK(),
            plastic=pf.ShellPlasticity(dofs=0),
        )
        u = energy.zero_state()
        assert np.isfinite(energy.value(u))


# ---------------------------------------------------------------------------
# Lifetime
# ---------------------------------------------------------------------------

class TestLifetime:
    def test_two_energies_from_same_mesh(self):
        sim = _make_tet_sim_mesh()
        e1 = pf.deformation_energy(
            sim, elastic=pf.StableNeo(), plastic=pf.VolumetricPlasticity(dofs=6),
        )
        e2 = pf.deformation_energy(
            sim, elastic=pf.StableNeo(), plastic=pf.VolumetricPlasticity(dofs=6),
        )
        u = e1.zero_state()
        assert e1.value(u) == pytest.approx(e2.value(u), rel=1e-12)

    def test_energy_survives_mesh_deletion(self):
        sim = _make_tet_sim_mesh()
        energy = pf.deformation_energy(
            sim, elastic=pf.StableNeo(), plastic=pf.VolumetricPlasticity(dofs=6),
        )
        u = energy.zero_state()
        val_before = energy.value(u)
        del sim
        gc.collect()
        val_after = energy.value(u)
        assert val_before == pytest.approx(val_after, rel=1e-15)

    def test_energy_set_with_deformation_energy_and_other(self):
        sim = _make_tet_sim_mesh()
        deform = pf.deformation_energy(
            sim, elastic=pf.StableNeo(), plastic=pf.VolumetricPlasticity(dofs=6),
        )
        lin = pe.LinearEnergy(np.zeros(deform.num_dofs, dtype=np.float64))
        es = pe.EnergySet([(deform, 1.0), (lin, 0.0)])
        u = deform.zero_state()
        assert es.value(u) == pytest.approx(deform.value(u), rel=1e-12)


# ---------------------------------------------------------------------------
# Error cases
# ---------------------------------------------------------------------------

class TestErrorCases:
    def test_missing_elastic_raises(self):
        sim = _make_tet_sim_mesh()
        with pytest.raises(ValueError, match="elastic material is required"):
            pf.deformation_energy(sim, plastic=pf.VolumetricPlasticity(dofs=6))

    def test_missing_plastic_raises(self):
        sim = _make_tet_sim_mesh()
        with pytest.raises(ValueError, match="plastic parametrization is required"):
            pf.deformation_energy(sim, elastic=pf.StableNeo())

    def test_wrong_type_sim_mesh_raises(self):
        with pytest.raises(TypeError, match="SimulationMesh"):
            pf.deformation_energy(
                "not_a_mesh",
                elastic=pf.StableNeo(),
                plastic=pf.VolumetricPlasticity(dofs=6),
            )

    def test_wrong_type_formulation_raises(self):
        sim = _make_tet_sim_mesh()
        with pytest.raises(TypeError, match="formulation must be"):
            pf.deformation_energy(
                sim,
                formulation="tet_p1",  # string, not TetP1()
                elastic=pf.StableNeo(),
                plastic=pf.VolumetricPlasticity(dofs=6),
            )

    def test_wrong_type_options_raises(self):
        sim = _make_tet_sim_mesh()
        with pytest.raises(TypeError, match="options must be"):
            pf.deformation_energy(
                sim,
                elastic=pf.StableNeo(),
                plastic=pf.VolumetricPlasticity(dofs=6),
                options={"enforce_spd": False},  # dict, not DeformationOptions
            )


# ---------------------------------------------------------------------------
# Module surface check
# ---------------------------------------------------------------------------

class TestModuleSurface:
    def test_pypgo_lazy_loads_fem(self):
        import pypgo as pg
        assert hasattr(pg, "fem")

    def test_pypgo_lazy_loads_energy(self):
        import pypgo as pg
        assert hasattr(pg, "energy")

    def test_fem_does_not_expose_raw_enums(self):
        # Raw elastic/plastic enum strings should not be importable from fem
        assert not hasattr(pf, "stable_neo")
        assert not hasattr(pf, "volumetric_dof6")

    def test_energy_does_not_expose_fem_constructors(self):
        # Energy module should not have FEM construction helpers
        assert not hasattr(pe, "deformation_energy")
        assert not hasattr(pe, "TetP1")
        assert not hasattr(pe, "StableNeo")

    def test_energy_exports_potential_energy_class(self):
        # For isinstance checks
        assert hasattr(pe, "PotentialEnergy")

    def test_energy_exports_deformation_energy_class(self):
        assert hasattr(pe, "DeformationEnergy")
