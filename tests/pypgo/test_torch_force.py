"""Tests for the differentiable force (gradient-output) PyTorch layer."""

import numpy as np
import pytest

torch = pytest.importorskip("torch")

import pypgo as pgo
import pypgo.fem as pf
from pypgo.fem.fields import MaterialState
from pypgo.fem.torch import StaticForceLayer


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


def make_operator(vertices, elements):
    volume = pgo.mesh.volume.VolumeMesh(
        pgo.mesh.CubicMeshData(vertices, elements),
        pgo.mesh.volume.ENuMaterial(E=2.0e5, nu=0.35),
    )
    mesh = pf.SimulationMesh(volume)
    plastic = pf.VolumetricPlasticityDefinition(dofs=0)
    systematic = pf.SystematicPokingDefinition(
        KNOTS, len(KNOTS) // 2, KNOTS, len(KNOTS) // 2)
    binding = pf.MaterialBinding(
        pf.ElasticMaterialBinding(
            systematic, mesh.num_elements,
            np.empty((mesh.num_elements, 0), dtype=np.float64)),
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
    displacement = np.zeros_like(vertices)
    displacement[:, 1] = (stretch - 1.0) * vertices[:, 1]
    return displacement.ravel()


def systematic_elastic_values():
    mu = 2.0e5 / (2 * (1 + 0.35))
    lam = 2.0e5 * 0.35 / ((1 + 0.35) * (1 - 2 * 0.35))
    return np.concatenate([mu * (1 + 1 / KNOTS ** 2), [lam]])


@pytest.fixture(scope="module")
def setup():
    vertices, elements = make_cube()
    operator = make_operator(vertices, elements)
    u = confined_uniaxial_displacement(vertices)
    e = systematic_elastic_values()
    layer = StaticForceLayer(energy_operator=operator)
    return operator, u, e, layer


def test_forward_matches_operator_gradient(setup):
    operator, u, e, layer = setup
    force = layer(
        torch.as_tensor(u, dtype=torch.float64),
        torch.as_tensor(e.reshape(1, -1), dtype=torch.float64),
        torch.empty((1, 0), dtype=torch.float64),
    )
    state = MaterialState(e, np.empty(0))
    np.testing.assert_allclose(
        force.numpy(), operator.gradient(u, state), rtol=1e-10)


def test_backward_grad_u_matches_hessian_matvec(setup):
    operator, u, e, layer = setup
    u_tensor = torch.as_tensor(u, dtype=torch.float64).requires_grad_()
    force = layer(
        u_tensor,
        torch.as_tensor(e.reshape(1, -1), dtype=torch.float64),
        torch.empty((1, 0), dtype=torch.float64),
    )
    adjoint = np.linspace(0.1, 1.0, u.size)
    force.backward(torch.as_tensor(adjoint, dtype=torch.float64))
    state = MaterialState(e, np.empty(0))
    expected = operator.hessian(u, state) @ adjoint
    np.testing.assert_allclose(
        u_tensor.grad.numpy(), expected, rtol=1e-10)


def test_force_loss_gradient_matches_fd(setup):
    """Direct-calibration demo path: MSE of forces differentiable wrt theta."""
    operator, u, e, layer = setup
    target_state = MaterialState(e * 1.05, np.empty(0))
    target_force = operator.gradient(u, target_state)
    num_elements = operator.num_elements

    theta = torch.tensor(
        np.log(e / 2.0e5), dtype=torch.float64, requires_grad=True)
    values = (2.0e5 * torch.exp(theta)).expand(num_elements, -1)
    force = layer(
        torch.as_tensor(u, dtype=torch.float64),
        values,
        torch.empty((num_elements, 0), dtype=torch.float64),
    )
    loss = 0.5 * torch.mean(
        ((force - torch.as_tensor(target_force)) / 2.0e5) ** 2)
    loss.backward()
    autograd_grad = theta.grad.numpy().copy()

    h = 1.0e-5
    fd = np.zeros_like(autograd_grad)
    theta0 = np.log(e / 2.0e5)
    for i in range(fd.size):
        plus = theta0.copy()
        minus = theta0.copy()
        plus[i] += h
        minus[i] -= h
        state_plus = MaterialState(
            np.broadcast_to(2.0e5 * np.exp(plus), (num_elements, plus.size))
            .reshape(-1).copy(), np.empty(0))
        state_minus = MaterialState(
            np.broadcast_to(2.0e5 * np.exp(minus), (num_elements, minus.size))
            .reshape(-1).copy(), np.empty(0))
        f_plus = operator.gradient(u, state_plus)
        f_minus = operator.gradient(u, state_minus)
        loss_plus = 0.5 * np.mean(
            ((f_plus - target_force) / 2.0e5) ** 2)
        loss_minus = 0.5 * np.mean(
            ((f_minus - target_force) / 2.0e5) ** 2)
        fd[i] = (loss_plus - loss_minus) / (2 * h)

    np.testing.assert_allclose(autograd_grad, fd, rtol=1e-4, atol=1e-8)


def test_force_layer_rejects_bad_inputs(setup):
    operator, u, e, layer = setup
    u_tensor = torch.as_tensor(u, dtype=torch.float64)
    e_tensor = torch.as_tensor(e.reshape(1, -1), dtype=torch.float64)
    plastic = torch.empty((1, 0), dtype=torch.float64)

    with pytest.raises(ValueError, match="1-D"):
        layer(torch.as_tensor(u.reshape(3, -1), dtype=torch.float64),
              e_tensor, plastic)
    with pytest.raises(ValueError, match="2-D"):
        layer(u_tensor, e_tensor[0], plastic)
    with pytest.raises(TypeError, match="float64"):
        layer(u_tensor.float(), e_tensor, plastic)
    with pytest.raises(TypeError, match="energy_operator"):
        StaticForceLayer(energy_operator=object())
