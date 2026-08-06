"""Tests for the differentiable deformation-energy PyTorch layer."""

import numpy as np
import pytest

torch = pytest.importorskip("torch")

import pypgo as pgo
import pypgo.fem as pf
from pypgo.fem.fields import MaterialState
from pypgo.fem.torch import StaticEnergyLayer


KNOTS = np.exp(np.linspace(np.log(0.6), -np.log(0.6), 5))


def make_cube(n=1):
    ids = np.arange((n + 1) ** 3).reshape((n + 1, n + 1, n + 1))
    vertices = np.array(
        [[i / n, j / n, k / n]
         for i in range(n + 1) for j in range(n + 1) for k in range(n + 1)],
        dtype=np.float64,
    )
    elements = np.array(
        [[ids[i, j, k], ids[i + 1, j, k], ids[i + 1, j + 1, k],
          ids[i, j + 1, k], ids[i, j, k + 1], ids[i + 1, j, k + 1],
          ids[i + 1, j + 1, k + 1], ids[i, j + 1, k + 1]]
         for i in range(n) for j in range(n) for k in range(n)],
        dtype=np.int64,
    )
    return vertices, elements


def make_operator(vertices, elements, *, elastic, plastic_dofs):
    volume = pgo.mesh.volume.VolumeMesh(
        pgo.mesh.CubicMeshData(vertices, elements),
        pgo.mesh.volume.ENuMaterial(E=2.0e5, nu=0.35),
    )
    mesh = pf.SimulationMesh(volume)
    plastic = pf.VolumetricPlasticityDefinition(dofs=plastic_dofs)
    if elastic.name == "neo_hookean":
        fixed = np.tile([2.0e5, 0.35], (mesh.num_elements, 1))
    else:
        fixed = np.empty((mesh.num_elements, 0), dtype=np.float64)
    binding = pf.MaterialBinding(
        pf.ElasticMaterialBinding(elastic, mesh.num_elements, fixed),
        pf.PlasticMaterialBinding(
            plastic, mesh.num_elements,
            np.empty((mesh.num_elements, 0), dtype=np.float64)),
    )
    return pf.DeformationEnergyOperator(
        mesh, binding,
        formulation=pf.CubicLinear(),
        options=pf.DeformationOptions(project_hessian_psd=False),
    )


def confined_uniaxial_displacement(vertices, stretch=0.85):
    """Full prescribed displacement for confined uniaxial compression."""
    displacement = np.zeros_like(vertices)
    displacement[:, 1] = (stretch - 1.0) * vertices[:, 1]
    return displacement.ravel()


def systematic_elastic_values():
    mu = 2.0e5 / (2 * (1 + 0.35))
    lam = 2.0e5 * 0.35 / ((1 + 0.35) * (1 - 2 * 0.35))
    return np.concatenate([mu * (1 + 1 / KNOTS ** 2), [lam]])


@pytest.fixture(scope="module")
def systematic_setup():
    vertices, elements = make_cube()
    operator = make_operator(
        vertices, elements,
        elastic=pf.SystematicPokingDefinition(
            KNOTS, len(KNOTS) // 2, KNOTS, len(KNOTS) // 2),
        plastic_dofs=0,
    )
    u = confined_uniaxial_displacement(vertices)
    e = systematic_elastic_values()
    return operator, u, e


def test_energy_value_matches_direct_evaluation(systematic_setup):
    operator, u, e = systematic_setup
    layer = StaticEnergyLayer(energy_operator=operator)
    energy_tensor = layer(
        torch.as_tensor(u, dtype=torch.float64),
        torch.as_tensor(e.reshape(1, -1), dtype=torch.float64),
        torch.empty((1, 0), dtype=torch.float64),
    )
    state = MaterialState(e, np.empty(0))
    direct = operator.value(u, state)
    assert float(energy_tensor.detach()) == pytest.approx(direct, rel=1e-12)


def test_energy_backward_grad_u_matches_operator_gradient(systematic_setup):
    operator, u, e = systematic_setup
    layer = StaticEnergyLayer(energy_operator=operator)
    u_tensor = torch.as_tensor(u, dtype=torch.float64).requires_grad_()
    energy_tensor = layer(
        u_tensor,
        torch.as_tensor(e.reshape(1, -1), dtype=torch.float64),
        torch.empty((1, 0), dtype=torch.float64),
    )
    energy_tensor.backward()
    state = MaterialState(e, np.empty(0))
    np.testing.assert_allclose(
        u_tensor.grad.numpy(), operator.gradient(u, state), rtol=1e-10)


def test_autograd_grad_force_matches_operator_gradient(systematic_setup):
    """The demo pattern: forces come from autograd.grad(E, u)."""
    operator, u, e = systematic_setup
    layer = StaticEnergyLayer(energy_operator=operator)
    u_tensor = torch.as_tensor(u, dtype=torch.float64).requires_grad_()
    energy_tensor = layer(
        u_tensor,
        torch.as_tensor(e.reshape(1, -1), dtype=torch.float64),
        torch.empty((1, 0), dtype=torch.float64),
    )
    force = torch.autograd.grad(energy_tensor, u_tensor)[0]
    state = MaterialState(e, np.empty(0))
    np.testing.assert_allclose(
        force.numpy(), operator.gradient(u, state), rtol=1e-10)


def test_energy_backward_elastic_grad_matches_dE_de(systematic_setup):
    operator, u, e = systematic_setup
    layer = StaticEnergyLayer(energy_operator=operator)
    e_tensor = torch.as_tensor(
        e.reshape(1, -1), dtype=torch.float64).requires_grad_()
    energy_tensor = layer(
        torch.as_tensor(u, dtype=torch.float64),
        e_tensor,
        torch.empty((1, 0), dtype=torch.float64),
    )
    energy_tensor.backward()
    state = MaterialState(e, np.empty(0))
    np.testing.assert_allclose(
        e_tensor.grad.numpy().reshape(-1),
        operator.dE_de(u, state),
        rtol=1e-10,
    )


def test_plastic_grad_matches_dE_dp_and_finite_differences():
    vertices, elements = make_cube()
    operator = make_operator(
        vertices, elements,
        elastic=pf.NeoHookeanDefinition(),
        plastic_dofs=6,
    )
    u = confined_uniaxial_displacement(vertices)
    p0 = np.array([[1.0, 0.0, 0.0, 1.0, 0.0, 1.0]])  # symmetric tensor identity
    layer = StaticEnergyLayer(energy_operator=operator)
    p_tensor = torch.tensor(p0, dtype=torch.float64).requires_grad_()
    energy_tensor = layer(
        torch.as_tensor(u, dtype=torch.float64),
        torch.empty((1, 0), dtype=torch.float64),
        p_tensor,
    )
    energy_tensor.backward()
    autograd_grad = p_tensor.grad.numpy().copy()

    state = MaterialState(np.empty(0), p0.reshape(-1))
    np.testing.assert_allclose(
        autograd_grad.reshape(-1), operator.dE_dp(u, state), rtol=1e-10)

    h = 1.0e-6
    fd = np.zeros(6)
    for i in range(6):
        plus, minus = p0.copy(), p0.copy()
        plus[0, i] += h
        minus[0, i] -= h
        y_plus = float(layer(
            torch.as_tensor(u, dtype=torch.float64),
            torch.empty((1, 0), dtype=torch.float64),
            torch.tensor(plus, dtype=torch.float64),
        ).detach())
        y_minus = float(layer(
            torch.as_tensor(u, dtype=torch.float64),
            torch.empty((1, 0), dtype=torch.float64),
            torch.tensor(minus, dtype=torch.float64),
        ).detach())
        fd[i] = (y_plus - y_minus) / (2 * h)
    np.testing.assert_allclose(autograd_grad, fd.reshape(1, -1), rtol=1e-4, atol=1e-2)


def test_elastic_and_plastic_grads_match_finite_differences():
    vertices, elements = make_cube()
    operator = make_operator(
        vertices, elements,
        elastic=pf.SystematicPokingDefinition(
            KNOTS, len(KNOTS) // 2, KNOTS, len(KNOTS) // 2),
        plastic_dofs=3,
    )
    u = confined_uniaxial_displacement(vertices)
    e0 = systematic_elastic_values()
    p0 = np.array([[1.0, 1.0, 1.0]])
    layer = StaticEnergyLayer(energy_operator=operator)
    e_tensor = torch.tensor(e0.reshape(1, -1), dtype=torch.float64).requires_grad_()
    p_tensor = torch.tensor(p0, dtype=torch.float64).requires_grad_()
    energy_tensor = layer(
        torch.as_tensor(u, dtype=torch.float64), e_tensor, p_tensor)
    energy_tensor.backward()
    grad_e = e_tensor.grad.numpy().copy()
    grad_p = p_tensor.grad.numpy().copy()

    h = 1.0e-6
    fd_e = np.zeros_like(grad_e)
    fd_p = np.zeros_like(grad_p)
    for i in range(grad_e.size):
        plus, minus = e0.copy(), e0.copy()
        plus[i] += h
        minus[i] -= h
        y_plus = float(layer(
            torch.as_tensor(u, dtype=torch.float64),
            torch.tensor(plus.reshape(1, -1), dtype=torch.float64),
            torch.tensor(p0, dtype=torch.float64),
        ).detach())
        y_minus = float(layer(
            torch.as_tensor(u, dtype=torch.float64),
            torch.tensor(minus.reshape(1, -1), dtype=torch.float64),
            torch.tensor(p0, dtype=torch.float64),
        ).detach())
        fd_e[0, i] = (y_plus - y_minus) / (2 * h)
    for i in range(grad_p.size):
        plus, minus = p0.copy(), p0.copy()
        plus[0, i] += h
        minus[0, i] -= h
        y_plus = float(layer(
            torch.as_tensor(u, dtype=torch.float64),
            torch.tensor(e0.reshape(1, -1), dtype=torch.float64),
            torch.tensor(plus, dtype=torch.float64),
        ).detach())
        y_minus = float(layer(
            torch.as_tensor(u, dtype=torch.float64),
            torch.tensor(e0.reshape(1, -1), dtype=torch.float64),
            torch.tensor(minus, dtype=torch.float64),
        ).detach())
        fd_p[0, i] = (y_plus - y_minus) / (2 * h)

    np.testing.assert_allclose(grad_e, fd_e, rtol=1e-4, atol=1e-2)
    np.testing.assert_allclose(grad_p, fd_p, rtol=1e-4, atol=1e-2)


def test_energy_layer_rejects_bad_inputs(systematic_setup):
    operator, u, e = systematic_setup
    layer = StaticEnergyLayer(energy_operator=operator)
    u_tensor = torch.as_tensor(u, dtype=torch.float64)
    e_tensor = torch.as_tensor(e.reshape(1, -1), dtype=torch.float64)
    plastic = torch.empty((1, 0), dtype=torch.float64)

    with pytest.raises(ValueError, match="1-D"):
        layer(torch.as_tensor(u.reshape(3, -1), dtype=torch.float64),
              e_tensor, plastic)
    with pytest.raises(ValueError, match="size must be"):
        layer(torch.as_tensor(u[:-1], dtype=torch.float64), e_tensor, plastic)
    with pytest.raises(ValueError, match="2-D"):
        layer(u_tensor, e_tensor[0], plastic)
    with pytest.raises(TypeError, match="float64"):
        layer(u_tensor.float(), e_tensor, plastic)
    with pytest.raises(TypeError, match="torch.Tensor"):
        layer(u, e_tensor, plastic)
    with pytest.raises(TypeError, match="energy_operator"):
        StaticEnergyLayer(energy_operator=object())
