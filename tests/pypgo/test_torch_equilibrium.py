"""Tests for the differentiable static-equilibrium PyTorch layer."""

import numpy as np
import pytest

torch = pytest.importorskip("torch")

import pypgo as pgo
import pypgo.fem as pf
import pypgo.solver as ps
from pypgo.fem.torch import StaticEquilibriumLayer


KNOTS = np.exp(np.linspace(np.log(0.6), -np.log(0.6), 5))
STRETCH = 0.85


def make_grid(n=2):
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


def make_plastic_operator(vertices, elements, plastic_dofs):
    """One-element operator with optimizable plastic channels."""
    volume = pgo.mesh.volume.VolumeMesh(
        pgo.mesh.CubicMeshData(vertices, elements),
        pgo.mesh.volume.ENuMaterial(E=2.0e5, nu=0.35),
    )
    mesh = pf.SimulationMesh(volume)
    plastic = pf.VolumetricPlasticityDefinition(dofs=plastic_dofs)
    binding = pf.MaterialBinding(
        pf.ElasticMaterialBinding(
            pf.NeoHookeanDefinition(), mesh.num_elements,
            np.tile([2.0e5, 0.35], (mesh.num_elements, 1))),
        pf.PlasticMaterialBinding(
            plastic, mesh.num_elements,
            np.empty((mesh.num_elements, 0), dtype=np.float64)),
    )
    return pf.DeformationEnergyOperator(
        mesh, binding,
        formulation=pf.CubicLinear(),
        options=pf.DeformationOptions(project_hessian_psd=False),
    )


def confined_uniaxial_case(vertices, stretch=STRETCH):
    num_dofs = vertices.size
    bottom = np.flatnonzero(np.isclose(vertices[:, 1], 0.0))
    top = np.flatnonzero(np.isclose(vertices[:, 1], 1.0))
    mapping = {}
    for v in bottom:
        mapping[3 * int(v) + 1] = 0.0
    for v in top:
        mapping[3 * int(v) + 1] = stretch - 1.0
    for v in range(len(vertices)):
        mapping[3 * v] = 0.0
        mapping[3 * v + 2] = 0.0
    fixed_dofs = np.array(sorted(mapping), dtype=np.int64)
    fixed_values = np.array([mapping[int(d)] for d in fixed_dofs], dtype=np.float64)
    free_dofs = np.setdiff1d(
        np.arange(num_dofs, dtype=np.int64), fixed_dofs)
    selector = np.zeros(num_dofs)
    selector[3 * top + 1] = 1.0
    initial = np.zeros(num_dofs)
    initial[1::3] = (stretch - 1.0) * vertices[:, 1]
    return fixed_dofs, fixed_values, free_dofs, selector, initial


def initial_params(num_elements):
    mu, lam = 2.0e5 / (2 * (1 + 0.35)), 2.0e5 * 0.35 / ((1 + 0.35) * (1 - 2 * 0.35))
    values = np.concatenate([mu * (1 + 1 / KNOTS ** 2), [lam]])
    return np.broadcast_to(values.reshape(1, -1), (num_elements, values.size)).copy()


def solve_direct(operator, elastic_values, case):
    fixed_dofs, fixed_values, _, _, initial = case
    state = pf.MaterialState(
        np.asarray(elastic_values, dtype=np.float64).reshape(-1), np.empty(0))
    energy = pf.DeformationPotentialEnergy(operator, state)
    problem = ps.OptimizationProblem(objective=energy)
    problem.fix_variables(
        fixed_dofs.tolist(), fixed_values, num_dofs=energy.num_dofs)
    x0 = initial.copy()
    x0[fixed_dofs] = fixed_values
    optimizer = ps.NewtonOptimizer(
        max_iterations=200,
        termination=ps.AbsoluteTermination(abs_tolerance=1.0e-10))
    return optimizer.solve(problem, x0).x, energy


@pytest.fixture(scope="module")
def setup():
    vertices, elements = make_grid()
    operator = make_operator(vertices, elements)
    case = confined_uniaxial_case(vertices)
    layer = StaticEquilibriumLayer(
        energy_operator=operator,
        fixed_dofs=case[0],
        fixed_values=case[1],
        reaction_selectors=case[3],
        sparse_backend=ps.EigenLDLT(),
        initial_displacement=case[4],
        residual_tol=1.0e-6,
    )
    params = initial_params(operator.num_elements)
    return operator, case, layer, params


def test_forward_reaction_matches_direct_solve(setup):
    operator, case, layer, params = setup
    tensor = torch.as_tensor(params, dtype=torch.float64)
    reaction = float(layer(
        tensor, torch.empty((operator.num_elements, 0), dtype=torch.float64)))

    x, energy = solve_direct(operator, params, case)
    expected = float(case[3] @ energy.gradient(x))
    assert reaction == pytest.approx(expected, rel=1e-12)


def test_forward_displacement_matches_direct_solve(setup):
    operator, case, _, params = setup
    fixed_dofs, fixed_values, _, _, initial = case
    displacement_layer = StaticEquilibriumLayer(
        energy_operator=operator,
        fixed_dofs=fixed_dofs,
        fixed_values=fixed_values,
        sparse_backend=ps.EigenLDLT(),
        initial_displacement=initial,
        residual_tol=1.0e-6,
    )
    tensor = torch.as_tensor(params, dtype=torch.float64)
    u_star = displacement_layer(
        tensor, torch.empty((operator.num_elements, 0), dtype=torch.float64))

    x, _ = solve_direct(operator, params, case)
    np.testing.assert_allclose(u_star.numpy(), x, atol=1e-10)


def test_backward_reaction_matches_finite_differences(setup):
    operator, _, layer, params = setup
    num_elements = operator.num_elements
    theta = torch.tensor(
        np.log(params[0]), dtype=torch.float64, requires_grad=True)

    def reaction(theta_value):
        values = torch.exp(theta_value).expand(num_elements, -1)
        return layer(values, torch.empty((num_elements, 0), dtype=torch.float64))

    reaction(theta).backward()
    autograd_grad = theta.grad.numpy().copy()

    h = 1.0e-5
    fd = np.zeros_like(autograd_grad)
    for i in range(autograd_grad.size):
        theta_plus = theta.detach().numpy().copy()
        theta_minus = theta_plus.copy()
        theta_plus[i] += h
        theta_minus[i] -= h
        y_plus = float(reaction(torch.as_tensor(theta_plus)))
        y_minus = float(reaction(torch.as_tensor(theta_minus)))
        fd[i] = (y_plus - y_minus) / (2 * h)

    np.testing.assert_allclose(autograd_grad, fd, rtol=1e-5, atol=1e-3)


def test_backward_reaction_matches_explicit_formula(setup):
    operator, case, layer, params = setup
    num_elements = operator.num_elements
    fixed_dofs, _, free_dofs, selector, _ = case
    theta = torch.tensor(
        np.log(params[0]), dtype=torch.float64, requires_grad=True)
    values = torch.exp(theta).expand(num_elements, -1)
    reaction = layer(
        values, torch.empty((num_elements, 0), dtype=torch.float64))
    reaction.backward()
    autograd_grad = theta.grad.numpy().copy()

    x, energy = solve_direct(operator, params, case)
    channels = operator.num_elastic_params
    hessian = energy.hessian(x).to_dense()
    direct = energy.elastic_material_vjp(x, selector)
    direct = direct.reshape(-1, channels).sum(axis=0)
    objective_u = hessian.T @ selector
    adjoint_free = np.linalg.solve(
        hessian[np.ix_(free_dofs, free_dofs)].T, objective_u[free_dofs])
    adjoint = np.zeros(operator.num_dofs)
    adjoint[free_dofs] = adjoint_free
    implicit = energy.elastic_material_vjp(x, adjoint).reshape(
        -1, channels).sum(axis=0)
    explicit_grad = (direct - implicit) * params[0]

    np.testing.assert_allclose(autograd_grad, explicit_grad, rtol=1e-8, atol=1e-4)


def test_global_broadcast_gradient_sums_elements(setup):
    operator, _, _, params = setup
    num_elements = operator.num_elements
    theta = torch.tensor(
        np.log(params[0]), dtype=torch.float64, requires_grad=True)
    elementwise = torch.exp(theta).expand(num_elements, -1)
    loss = (elementwise * torch.arange(
        num_elements, dtype=torch.float64).reshape(-1, 1)).sum()
    loss.backward()
    summed = theta.grad.numpy().copy()

    expected = (params[0] * np.arange(num_elements).reshape(-1, 1)).sum(axis=0)
    np.testing.assert_allclose(summed, expected, rtol=1e-12)


def test_plastic_zero_channels_returns_empty_gradient(setup):
    operator, _, layer, params = setup
    num_elements = operator.num_elements
    values = torch.as_tensor(params, dtype=torch.float64).requires_grad_()
    plastic = torch.empty(
        (num_elements, 0), dtype=torch.float64, requires_grad=True)
    layer(values, plastic).backward()
    assert values.grad is not None
    assert plastic.grad is not None
    assert plastic.grad.shape == (num_elements, 0)


def test_plastic_channels_backward_matches_finite_differences():
    """Plastic-only Jacobian: dofs=6 channels, finite-difference check."""
    vertices, elements = make_grid(1)
    operator = make_plastic_operator(vertices, elements, plastic_dofs=6)
    case = confined_uniaxial_case(vertices)
    layer = StaticEquilibriumLayer(
        energy_operator=operator,
        fixed_dofs=case[0],
        fixed_values=case[1],
        reaction_selectors=case[3],
        sparse_backend=ps.EigenLDLT(),
        initial_displacement=case[4],
        residual_tol=1.0e-6,
    )

    elastic = torch.empty((1, 0), dtype=torch.float64)
    # Symmetric plastic-strain tensor rest state (xx, yy, zz, xy, yz, zx).
    p0 = np.array([[1.0, 0.0, 0.0, 1.0, 0.0, 1.0]])
    plastic = torch.tensor(p0, dtype=torch.float64).requires_grad_()
    reaction = layer(elastic, plastic)
    reaction.backward()
    autograd_grad = plastic.grad.numpy().copy()

    h = 1.0e-6
    fd = np.zeros(6)
    for i in range(6):
        plus, minus = p0.copy(), p0.copy()
        plus[0, i] += h
        minus[0, i] -= h
        y_plus = float(layer(
            elastic, torch.tensor(plus, dtype=torch.float64)).detach())
        y_minus = float(layer(
            elastic, torch.tensor(minus, dtype=torch.float64)).detach())
        fd[i] = (y_plus - y_minus) / (2 * h)

    np.testing.assert_allclose(autograd_grad, fd.reshape(1, -1), rtol=1e-4, atol=1e-2)


def test_elastic_and_plastic_backward_match_finite_differences():
    """Simultaneous elastic+plastic Jacobian through the shared adjoint."""
    vertices, elements = make_grid(1)
    volume = pgo.mesh.volume.VolumeMesh(
        pgo.mesh.CubicMeshData(vertices, elements),
        pgo.mesh.volume.ENuMaterial(E=2.0e5, nu=0.35),
    )
    mesh = pf.SimulationMesh(volume)
    plastic = pf.VolumetricPlasticityDefinition(dofs=3)
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
    operator = pf.DeformationEnergyOperator(
        mesh, binding,
        formulation=pf.CubicLinear(),
        options=pf.DeformationOptions(project_hessian_psd=False),
    )
    case = confined_uniaxial_case(vertices)
    layer = StaticEquilibriumLayer(
        energy_operator=operator,
        fixed_dofs=case[0],
        fixed_values=case[1],
        reaction_selectors=case[3],
        sparse_backend=ps.EigenLDLT(),
        initial_displacement=case[4],
        residual_tol=1.0e-6,
    )

    mu, lam = (2.0e5 / (2 * (1 + 0.35)),
               2.0e5 * 0.35 / ((1 + 0.35) * (1 - 2 * 0.35)))
    e0 = np.concatenate([mu * (1 + 1 / KNOTS ** 2), [lam]])
    p0 = np.array([[1.0, 1.0, 1.0]])  # dofs=3 rest state (diagonal components)
    elastic = torch.tensor(e0.reshape(1, -1), dtype=torch.float64).requires_grad_()
    plastic = torch.tensor(p0, dtype=torch.float64).requires_grad_()
    reaction = layer(elastic, plastic)
    reaction.backward()
    grad_e = elastic.grad.numpy().copy()
    grad_p = plastic.grad.numpy().copy()

    h = 1.0e-6
    fd_e = np.zeros_like(grad_e)
    fd_p = np.zeros_like(grad_p)
    for i in range(grad_e.size):
        plus, minus = e0.copy(), e0.copy()
        plus[i] += h
        minus[i] -= h
        y_plus = float(layer(
            torch.tensor(plus.reshape(1, -1), dtype=torch.float64),
            torch.tensor(p0, dtype=torch.float64)).detach())
        y_minus = float(layer(
            torch.tensor(minus.reshape(1, -1), dtype=torch.float64),
            torch.tensor(p0, dtype=torch.float64)).detach())
        fd_e[0, i] = (y_plus - y_minus) / (2 * h)
    for i in range(grad_p.size):
        plus, minus = p0.copy(), p0.copy()
        plus[0, i] += h
        minus[0, i] -= h
        y_plus = float(layer(
            torch.tensor(e0.reshape(1, -1), dtype=torch.float64),
            torch.tensor(plus, dtype=torch.float64)).detach())
        y_minus = float(layer(
            torch.tensor(e0.reshape(1, -1), dtype=torch.float64),
            torch.tensor(minus, dtype=torch.float64)).detach())
        fd_p[0, i] = (y_plus - y_minus) / (2 * h)

    np.testing.assert_allclose(grad_e, fd_e, rtol=1e-4, atol=1e-2)
    np.testing.assert_allclose(grad_p, fd_p, rtol=1e-4, atol=1e-2)


def test_layer_rejects_bad_inputs(setup):
    operator, _, layer, params = setup
    values = torch.as_tensor(params, dtype=torch.float64)
    plastic = torch.empty((operator.num_elements, 0), dtype=torch.float64)

    with pytest.raises(ValueError, match="2-D"):
        layer(values[0], plastic)
    with pytest.raises(ValueError, match="2-D"):
        layer(values, torch.empty(0, dtype=torch.float64))
    with pytest.raises(ValueError, match="shape must be"):
        layer(values[:, :-1], plastic)
    with pytest.raises(TypeError, match="float64"):
        layer(values.float(), plastic)
    with pytest.raises(TypeError, match="torch.Tensor"):
        layer(params, plastic)


def test_layer_rejects_singular_backend_mismatch(setup):
    operator, case, _, params = setup
    with pytest.raises(TypeError, match="sparse_backend"):
        StaticEquilibriumLayer(
            energy_operator=operator,
            fixed_dofs=case[0],
            fixed_values=case[1],
            sparse_backend="eigen",
        )
