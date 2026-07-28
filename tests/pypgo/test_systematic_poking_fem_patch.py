"""Affine FEM patch tests for classical NH and Systematic Poking."""

import numpy as np
import pypgo as pgo
import pypgo.fem as pf
import pytest

from tests.pypgo.material_helpers import direct_assignment


YOUNGS_MODULUS = 2.0e5
POISSON_RATIO = 0.35
MU = YOUNGS_MODULUS / (2.0 * (1.0 + POISSON_RATIO))
LAME_LAMBDA = (
    YOUNGS_MODULUS
    * POISSON_RATIO
    / ((1.0 + POISSON_RATIO) * (1.0 - 2.0 * POISSON_RATIO))
)
STRETCH_KNOTS = np.exp(np.linspace(np.log(0.5), np.log(2.0), 9))
VOLUME_KNOTS = np.exp(np.linspace(-1.0, 1.0, 9))
SYSTEMATIC_PARAMETERS = np.concatenate([
    MU * (1.0 + 1.0 / STRETCH_KNOTS**2),
    [LAME_LAMBDA],
])


def _unit_cube_import():
    cube = pgo.mesh.CubicMeshData(
        np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ], dtype=np.float64),
        np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64),
    )
    volume = pgo.mesh.volume.VolumeMesh(
        cube,
        pgo.mesh.volume.ENuMaterial(
            E=YOUNGS_MODULUS, nu=POISSON_RATIO),
    )
    return pf.SimulationImportResult(volume)


def _make_energy(elastic, elastic_values=None):
    imported = _unit_cube_import()
    assignment = direct_assignment(
        imported,
        elastic,
        pf.VolumetricPlasticityDefinition(dofs=0),
        pf.ConstantParameterLayout,
        pf.ConstantParameterLayout,
        elastic_values=elastic_values,
        plastic_values=np.empty((1, 0), dtype=np.float64),
    )
    return pf.DeformationEnergy(
        assignment,
        formulation=pf.CubicLinear(),
        options=pf.DeformationOptions(
            project_hessian_psd=False,
            enable_material_max_step=False,
        ),
    )


def _neo_hookean_energy_density(F):
    log_j = np.log(np.linalg.det(F))
    return (
        0.5 * MU * (np.sum(F * F) - 3.0)
        - MU * log_j
        + 0.5 * LAME_LAMBDA * log_j**2
    )


def _neo_hookean_stress(F):
    log_j = np.log(np.linalg.det(F))
    inverse_transpose = np.linalg.inv(F).T
    return (
        MU * F
        + (LAME_LAMBDA * log_j - MU) * inverse_transpose
    )


def _neo_hookean_tangent_action(F, dF):
    log_j = np.log(np.linalg.det(F))
    inverse_transpose = np.linalg.inv(F).T
    pressure = LAME_LAMBDA * log_j - MU
    d_log_j = np.sum(inverse_transpose * dF)
    d_inverse_transpose = (
        -inverse_transpose @ dF.T @ inverse_transpose
    )
    return (
        MU * dF
        + LAME_LAMBDA * d_log_j * inverse_transpose
        + pressure * d_inverse_transpose
    )


def _deformation_gradient_basis():
    deformation_gradient_basis = []
    for row in range(3):
        for col in range(3):
            dF = np.zeros((3, 3), dtype=np.float64)
            dF[row, col] = 1.0
            deformation_gradient_basis.append(dF)
    return deformation_gradient_basis


def _affine_basis(rest_positions):
    deformation_gradient_basis = _deformation_gradient_basis()
    basis = np.empty((rest_positions.size, 9), dtype=np.float64)
    for column, dF in enumerate(deformation_gradient_basis):
        basis[:, column] = (rest_positions @ dF.T).ravel()
    return basis, deformation_gradient_basis


def _affine_response(energy, F):
    rest_positions = energy.vertex_rest_positions
    displacement = (
        rest_positions @ (F - np.eye(3)).T
    ).ravel()
    gradient = energy.gradient(displacement)
    hessian = energy.hessian(displacement)
    basis, _ = _affine_basis(rest_positions)
    stress = (basis.T @ gradient).reshape((3, 3))
    tangent = basis.T @ (hessian @ basis)
    return energy.value(displacement), stress, tangent


def _analytic_neo_hookean_tangent(F):
    basis = _deformation_gradient_basis()
    tangent = np.empty((9, 9), dtype=np.float64)
    for column, dF in enumerate(basis):
        tangent[:, column] = _neo_hookean_tangent_action(
            F, dF).ravel()
    return tangent


PATCH_STATES = (
    pytest.param(np.diag([0.8, 1.0, 1.0]), id="uniaxial"),
    pytest.param(np.diag([1.15, 1.15, 1.0]), id="biaxial"),
    pytest.param(1.1 * np.eye(3), id="volumetric"),
    pytest.param(
        np.array([[1.0, 0.4, 0.0],
                  [0.0, 1.0, 0.0],
                  [0.0, 0.0, 1.0]]),
        id="simple-shear",
    ),
    pytest.param(
        np.array([[1.08, 0.13, -0.04],
                  [0.02, 0.91, 0.08],
                  [-0.03, 0.05, 1.04]]),
        id="distinct",
    ),
)


@pytest.mark.parametrize("F", PATCH_STATES)
def test_classical_neo_hookean_affine_patch_matches_continuum(F):
    energy = _make_energy(pf.NeoHookeanDefinition())
    fem_psi, fem_stress, fem_tangent = _affine_response(energy, F)

    np.testing.assert_allclose(
        fem_psi,
        _neo_hookean_energy_density(F),
        rtol=2e-12,
        atol=2e-10,
    )
    np.testing.assert_allclose(
        fem_stress,
        _neo_hookean_stress(F),
        rtol=2e-11,
        atol=2e-8,
    )
    np.testing.assert_allclose(
        fem_tangent,
        _analytic_neo_hookean_tangent(F),
        rtol=2e-10,
        atol=2e-7,
    )


@pytest.mark.parametrize("F", PATCH_STATES)
def test_systematic_poking_affine_patch_tracks_neo_hookean(F):
    neo_hookean = _make_energy(pf.NeoHookeanDefinition())
    systematic = _make_energy(
        pf.SystematicPokingDefinition(
            STRETCH_KNOTS,
            len(STRETCH_KNOTS) // 2,
            VOLUME_KNOTS,
            len(VOLUME_KNOTS) // 2,
        ),
        elastic_values=SYSTEMATIC_PARAMETERS[None, :],
    )

    neo_psi, neo_stress, neo_tangent = _affine_response(
        neo_hookean, F)
    systematic_psi, systematic_stress, systematic_tangent = (
        _affine_response(systematic, F)
    )

    assert (
        abs(systematic_psi - neo_psi) / YOUNGS_MODULUS
        < 6.0e-3
    )
    assert (
        np.linalg.norm(systematic_stress - neo_stress)
        / YOUNGS_MODULUS
        < 5.0e-2
    )
    assert (
        np.linalg.norm(systematic_tangent - neo_tangent)
        / YOUNGS_MODULUS
        < 3.5e-1
    )
