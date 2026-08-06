"""Self-contained regression tests for equilibrium-based Systematic Poking calibration.

These tests cover the load-case construction, the implicit-function-theorem
reaction Jacobian, and a Gauss-Newton fit without depending on any experiment
script (the original ``main.py`` was removed; the notebooks are the reference
implementation).
"""

from dataclasses import dataclass

import numpy as np
import pytest

import pypgo as pgo
import pypgo.fem as pf
import pypgo.solver as ps


TARGET_E, TARGET_NU = 2.0e5, 0.35
INIT_E, INIT_NU = 1.2e5, 0.25
FORCE_SCALE = TARGET_E
STATIC_TOL = 1.0e-6
SMOOTHNESS = 1.0e-3
GRID_SIZE = 2
KNOTS = np.exp(np.linspace(np.log(0.6), -np.log(0.6), 5))
SHEARS = (-0.8, -0.4, 0.4, 0.8)

OPTIMIZER = ps.NewtonOptimizer(
    max_iterations=200,
    termination=ps.AbsoluteTermination(abs_tolerance=1.0e-10),
)


@dataclass
class LoadCase:
    label: str
    protocol: str
    coordinate: float
    fixed_dofs: np.ndarray
    fixed_values: np.ndarray
    free_dofs: np.ndarray
    reaction_selector: np.ndarray
    initial_displacement: np.ndarray
    target: float = 0.0


def make_cubic_grid(n: int = GRID_SIZE):
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


def _make_operator(elastic, fixed_values):
    volume = pgo.mesh.volume.VolumeMesh(
        pgo.mesh.CubicMeshData(*make_cubic_grid()),
        pgo.mesh.volume.ENuMaterial(E=TARGET_E, nu=TARGET_NU),
    )
    mesh = pf.SimulationMesh(volume)
    plastic = pf.VolumetricPlasticityDefinition(dofs=0)
    binding = pf.MaterialBinding(
        pf.ElasticMaterialBinding(elastic, mesh.num_elements, fixed_values),
        pf.PlasticMaterialBinding(
            plastic, mesh.num_elements,
            np.empty((mesh.num_elements, 0), dtype=np.float64)),
    )
    return pf.DeformationEnergyOperator(
        mesh, binding,
        formulation=pf.CubicLinear(),
        options=pf.DeformationOptions(project_hessian_psd=False),
    )


def initial_parameters():
    mu = INIT_E / (2 * (1 + INIT_NU))
    lam = INIT_E * INIT_NU / ((1 + INIT_NU) * (1 - 2 * INIT_NU))
    return np.concatenate([mu * (1 + 1 / KNOTS ** 2), [lam]])


def make_energy():
    operator = _make_operator(
        pf.SystematicPokingDefinition(KNOTS, 2, KNOTS, 2),
        np.empty((operator_num_elements(), 0)),
    )
    values = np.broadcast_to(
        initial_parameters(), (operator.num_elements, initial_parameters().size)
    ).copy()
    return pf.DeformationPotentialEnergy(
        operator, pf.MaterialState(values.reshape(-1), np.empty(0)))


def operator_num_elements():
    return GRID_SIZE ** 3


def make_target_energy():
    operator = _make_operator(
        pf.NeoHookeanDefinition(),
        np.tile([TARGET_E, TARGET_NU], (operator_num_elements(), 1)),
    )
    return pf.DeformationPotentialEnergy(
        operator, pf.MaterialState(np.empty(0), np.empty(0)))


def _constraint_arrays(mapping, num_dofs):
    fixed_dofs = np.array(sorted(mapping), dtype=np.int64)
    fixed_values = np.array(
        [mapping[int(d)] for d in fixed_dofs], dtype=np.float64)
    free_dofs = np.setdiff1d(
        np.arange(num_dofs, dtype=np.int64), fixed_dofs)
    return fixed_dofs, fixed_values, free_dofs


def _free_uniaxial_rigid_pins(vertices, bottom):
    positions = vertices[bottom]
    centroid = positions.mean(axis=0)
    d2 = ((positions[:, 0] - centroid[0]) ** 2 +
          (positions[:, 2] - centroid[2]) ** 2)
    anchor = int(bottom[np.argmin(d2)])
    same_z = bottom[np.isclose(vertices[bottom, 2], vertices[anchor, 2])]
    candidates = same_z[same_z != anchor]
    if candidates.size == 0:
        candidates = bottom[bottom != anchor]
    rotation_pin = int(candidates[np.argmax(
        np.abs(vertices[candidates, 0] - vertices[anchor, 0]))])
    return anchor, rotation_pin


def _uniaxial_case(vertices, stretch, *, confined):
    num_dofs = vertices.size
    bottom = np.flatnonzero(np.isclose(vertices[:, 1], 0.0))
    top = np.flatnonzero(np.isclose(vertices[:, 1], 1.0))
    mapping = {}
    for v in bottom:
        mapping[3 * int(v) + 1] = 0.0
    for v in top:
        mapping[3 * int(v) + 1] = stretch - 1.0
    if confined:
        for v in range(len(vertices)):
            mapping[3 * v] = 0.0
            mapping[3 * v + 2] = 0.0
    else:
        anchor, rotation_pin = _free_uniaxial_rigid_pins(vertices, bottom)
        mapping[3 * anchor] = 0.0
        mapping[3 * anchor + 2] = 0.0
        mapping[3 * rotation_pin + 2] = 0.0
    fixed_dofs, fixed_values, free_dofs = _constraint_arrays(
        mapping, num_dofs)
    selector = np.zeros(num_dofs)
    selector[3 * top + 1] = 1.0
    initial = np.zeros(num_dofs)
    initial[1::3] = (stretch - 1.0) * vertices[:, 1]
    return LoadCase(
        label="uniaxial", protocol=(
            "confined_uniaxial" if confined else "free_uniaxial"),
        coordinate=stretch,
        fixed_dofs=fixed_dofs, fixed_values=fixed_values,
        free_dofs=free_dofs, reaction_selector=selector,
        initial_displacement=initial)


def _shear_case(vertices, shear):
    num_dofs = vertices.size
    bottom = np.flatnonzero(np.isclose(vertices[:, 1], 0.0))
    top = np.flatnonzero(np.isclose(vertices[:, 1], 1.0))
    mapping = {}
    for v in bottom:
        for c in range(3):
            mapping[3 * int(v) + c] = 0.0
    for v in top:
        mapping[3 * int(v)] = shear
        mapping[3 * int(v) + 1] = 0.0
    fixed_dofs, fixed_values, free_dofs = _constraint_arrays(
        mapping, num_dofs)
    selector = np.zeros(num_dofs)
    selector[3 * top] = 1.0
    initial = np.zeros(num_dofs)
    initial[0::3] = shear * vertices[:, 1]
    return LoadCase(
        label="shear", protocol="simple_shear", coordinate=shear,
        fixed_dofs=fixed_dofs, fixed_values=fixed_values,
        free_dofs=free_dofs, reaction_selector=selector,
        initial_displacement=initial)


def load_cases(vertices):
    stretches = [k for i, k in enumerate(KNOTS) if i != len(KNOTS) // 2]
    cases = []
    for s in stretches:
        cases.append(_uniaxial_case(vertices, s, confined=False))
        cases.append(_uniaxial_case(vertices, s, confined=True))
    for g in SHEARS:
        cases.append(_shear_case(vertices, g))
    return cases


def solve_case(energy, case):
    problem = ps.OptimizationProblem(objective=energy)
    problem.fix_variables(
        case.fixed_dofs.tolist(), case.fixed_values,
        num_dofs=energy.num_dofs)
    x0 = case.initial_displacement.copy()
    x0[case.fixed_dofs] = case.fixed_values
    x = OPTIMIZER.solve(problem, x0).x.copy()
    if case.free_dofs.size:
        residual = np.max(np.abs(energy.gradient(x)[case.free_dofs]))
        if residual > STATIC_TOL:
            raise RuntimeError(
                f"static solve failed for {case.label}: residual={residual:.3e}")
    return x


def reaction_jacobian(energy, case, x):
    """Implicit-function-theorem reaction Jacobian (notebook formula)."""
    channels = energy.num_elastic_params
    hessian = energy.hessian(x).to_dense()
    direct = energy.elastic_material_vjp(x, case.reaction_selector)
    direct = direct.reshape(-1, channels).sum(axis=0)
    if case.free_dofs.size:
        objective_u = hessian.T @ case.reaction_selector
        adjoint_free = np.linalg.solve(
            hessian[np.ix_(case.free_dofs, case.free_dofs)].T,
            objective_u[case.free_dofs])
        adjoint = np.zeros(energy.num_dofs)
        adjoint[case.free_dofs] = adjoint_free
        direct -= energy.elastic_material_vjp(
            x, adjoint).reshape(-1, channels).sum(axis=0)
    return direct


def residual_and_jacobian(energy, cases, theta):
    parameters = FORCE_SCALE * np.exp(theta)
    values = np.broadcast_to(
        parameters, (energy.num_elements, parameters.size)).copy()
    state = energy.material_state.with_elastic_values(values.reshape(-1))
    fitted_energy = pf.DeformationPotentialEnergy(
        energy.energy_operator, state)
    residuals, jacobians = [], []
    for case in cases:
        x = solve_case(fitted_energy, case)
        reaction = float(case.reaction_selector @ fitted_energy.gradient(x))
        residuals.append((reaction - case.target) / FORCE_SCALE)
        jacobians.append(
            reaction_jacobian(fitted_energy, case, x) * parameters / FORCE_SCALE)
    residual = np.asarray(residuals)
    jacobian = np.asarray(jacobians)
    if SMOOTHNESS > 0.0:
        stretch_count = len(theta) - 1
        operator = np.zeros((stretch_count - 2, stretch_count))
        rows = np.arange(stretch_count - 2)
        operator[rows, rows] = 1.0
        operator[rows, rows + 1] = -2.0
        operator[rows, rows + 2] = 1.0
        scale = np.sqrt(SMOOTHNESS)
        reg_jacobian = np.zeros((operator.shape[0], len(theta)))
        reg_jacobian[:, :-1] = scale * operator
        residual = np.concatenate([residual, scale * operator @ theta[:-1]])
        jacobian = np.vstack([jacobian, reg_jacobian])
    return residual, jacobian


def fit_parameters(energy, cases, theta0, max_iterations=20):
    theta = theta0.copy()
    damping = 1.0e-5
    history = []
    for _ in range(max_iterations):
        residual, jacobian = residual_and_jacobian(energy, cases, theta)
        objective = 0.5 * float(residual @ residual)
        gradient = jacobian.T @ residual
        history.append(objective)
        if np.linalg.norm(gradient, ord=np.inf) < 1.0e-9:
            break
        normal = jacobian.T @ jacobian
        diagonal = np.maximum(np.diag(normal), 1.0e-12)
        for _ in range(12):
            step = np.linalg.solve(
                normal + damping * np.diag(diagonal), -gradient)
            if np.max(np.abs(step)) > 0.75:
                step *= 0.75 / np.max(np.abs(step))
            candidate = theta + step
            candidate_residual, _ = residual_and_jacobian(
                energy, cases, candidate)
            candidate_objective = 0.5 * float(
                candidate_residual @ candidate_residual)
            if candidate_objective < objective:
                theta = candidate
                damping = max(damping / 3.0, 1.0e-12)
                break
            damping *= 10.0
        else:
            break
    return theta, history


@pytest.fixture(scope="module")
def problem():
    vertices, _ = make_cubic_grid()
    energy = make_energy()
    cases = load_cases(vertices)
    target_energy = make_target_energy()
    for case in cases:
        x = solve_case(target_energy, case)
        case.target = float(
            case.reaction_selector @ target_energy.gradient(x))
    return energy, cases


def test_load_protocols_leave_true_equilibrium_dofs(problem):
    energy, cases = problem
    assert {c.protocol for c in cases} == {
        "free_uniaxial", "confined_uniaxial", "simple_shear"}
    assert len(cases) == 12
    assert all(
        0 < c.free_dofs.size < energy.num_dofs for c in cases)


def test_free_uniaxial_uses_only_normal_constraints_and_rigid_pins(problem):
    vertices, _ = make_cubic_grid()
    case = _uniaxial_case(vertices, 0.75, confined=False)
    bottom = np.flatnonzero(np.isclose(vertices[:, 1], 0.0))
    top = np.flatnonzero(np.isclose(vertices[:, 1], 1.0))
    assert case.fixed_dofs.size == 21
    assert case.free_dofs.size == 60
    assert set(3 * bottom + 1) <= set(case.fixed_dofs)
    assert set(3 * top + 1) <= set(case.fixed_dofs)
    tangent_dofs = case.fixed_dofs[case.fixed_dofs % 3 != 1]
    assert tangent_dofs.size == 3


def test_confined_uniaxial_fixes_every_lateral_dof(problem):
    vertices, _ = make_cubic_grid()
    case = _uniaxial_case(vertices, 0.75, confined=True)
    assert case.fixed_dofs.size == 72
    assert case.free_dofs.size == 9
    assert set(3 * np.arange(len(vertices))) <= set(case.fixed_dofs)
    assert set(3 * np.arange(len(vertices)) + 2) <= set(case.fixed_dofs)


def test_reaction_jacobian_matches_finite_differences(problem):
    energy, cases = problem
    parameters = initial_parameters()
    theta = np.log(parameters / FORCE_SCALE)
    _, analytic = residual_and_jacobian(energy, cases, theta)
    analytic = analytic[:len(cases)]
    finite_difference = np.empty_like(analytic)
    step = 2.0e-6
    for i in range(theta.size):
        direction = np.zeros_like(theta)
        direction[i] = step
        residual_plus, _ = residual_and_jacobian(energy, cases, theta + direction)
        residual_minus, _ = residual_and_jacobian(energy, cases, theta - direction)
        finite_difference[:, i] = (
            residual_plus[:len(cases)] - residual_minus[:len(cases)]
        ) / (2.0 * step)
    np.testing.assert_allclose(
        analytic, finite_difference, rtol=5.0e-7, atol=2.0e-9)


def test_fit_reduces_train_rmse(problem):
    energy, cases = problem
    theta0 = np.log(initial_parameters() / FORCE_SCALE)
    initial_residual, _ = residual_and_jacobian(energy, cases, theta0)
    initial_rmse = np.sqrt(np.mean(initial_residual[:len(cases)] ** 2))
    theta, history = fit_parameters(energy, cases, theta0)
    final_residual, _ = residual_and_jacobian(energy, cases, theta)
    final_rmse = np.sqrt(np.mean(final_residual[:len(cases)] ** 2))
    assert np.all(FORCE_SCALE * np.exp(theta) > 0.0)
    assert history[-1] < history[0]
    assert final_rmse < 0.01 * initial_rmse
