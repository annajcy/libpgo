"""Fit Systematic Poking parameters through static reaction-force curves."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
from importlib.util import find_spec
import json
from pathlib import Path

import numpy as np
import pypgo as pgo
import pypgo.fem as pf
import pypgo.solver as ps


CASE_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = CASE_DIR / "output"
YOUNGS_MODULUS = 2.0e5
POISSON_RATIO = 0.35
INITIAL_YOUNGS_MODULUS = 1.2e5
INITIAL_POISSON_RATIO = 0.25
FORCE_SCALE = YOUNGS_MODULUS
STATIC_RESIDUAL_TOLERANCE = 1.0e-6


@dataclass(frozen=True)
class LoadCase:
    label: str
    protocol: str
    coordinate: float
    fixed_dofs: np.ndarray
    fixed_values: np.ndarray
    free_dofs: np.ndarray
    reaction_selector: np.ndarray
    initial_displacement: np.ndarray


@dataclass
class ReactionState:
    reaction: float
    displacement: np.ndarray
    iterations: int
    free_residual_max: float


@dataclass
class CalibrationProblem:
    systematic_energy: object
    target_energy: object
    stretch_knots: np.ndarray
    volume_knots: np.ndarray
    train_cases: tuple[LoadCase, ...]
    validation_cases: tuple[LoadCase, ...]
    holdout_cases: tuple[LoadCase, ...]
    train_targets: np.ndarray
    validation_targets: np.ndarray
    holdout_targets: np.ndarray
    inner_optimizer: object
    grid_size: int
    warm_starts: dict[str, np.ndarray] = field(default_factory=dict)

    def data_for(
        self, split: str
    ) -> tuple[tuple[LoadCase, ...], np.ndarray]:
        if split == "train":
            return self.train_cases, self.train_targets
        if split == "validation":
            return self.validation_cases, self.validation_targets
        if split == "holdout":
            return self.holdout_cases, self.holdout_targets
        raise ValueError(f"unknown split {split!r}")

    def reset_warm_starts(self) -> None:
        self.warm_starts.clear()


@dataclass
class CalibrationResult:
    initial_parameters: np.ndarray
    fitted_parameters: np.ndarray
    history: list[dict[str, float]]
    summary: dict[str, object]


def _lame_parameters(
    youngs_modulus: float, poisson_ratio: float
) -> tuple[float, float]:
    mu = youngs_modulus / (2.0 * (1.0 + poisson_ratio))
    lame_lambda = (
        youngs_modulus
        * poisson_ratio
        / ((1.0 + poisson_ratio) * (1.0 - 2.0 * poisson_ratio))
    )
    return mu, lame_lambda


def make_cubic_grid(n: int) -> tuple[np.ndarray, np.ndarray]:
    if n < 2:
        raise ValueError("grid size must be at least two")
    vertex_ids = np.arange((n + 1) ** 3).reshape(
        (n + 1, n + 1, n + 1))
    vertices = np.array(
        [
            [i / n, j / n, k / n]
            for i in range(n + 1)
            for j in range(n + 1)
            for k in range(n + 1)
        ],
        dtype=np.float64,
    )
    elements = np.array(
        [
            [
                vertex_ids[i, j, k],
                vertex_ids[i + 1, j, k],
                vertex_ids[i + 1, j + 1, k],
                vertex_ids[i, j + 1, k],
                vertex_ids[i, j, k + 1],
                vertex_ids[i + 1, j, k + 1],
                vertex_ids[i + 1, j + 1, k + 1],
                vertex_ids[i, j + 1, k + 1],
            ]
            for i in range(n)
            for j in range(n)
            for k in range(n)
        ],
        dtype=np.int64,
    )
    return vertices, elements


def _make_import(grid_size: int) -> pf.SimulationImportResult:
    vertices, elements = make_cubic_grid(grid_size)
    cubic = pgo.mesh.CubicMeshData(vertices, elements)
    volume = pgo.mesh.volume.VolumeMesh(
        cubic,
        pgo.mesh.volume.ENuMaterial(
            E=YOUNGS_MODULUS,
            nu=POISSON_RATIO,
        ),
    )
    return pf.SimulationImportResult(volume)


def _make_energy(imported, elastic, elastic_values):
    mesh = imported.mesh
    plastic = pf.VolumetricPlasticityDefinition(dofs=0)

    def identity_field(field_type, names, layout_type):
        count = len(names)
        return field_type(
            names,
            layout_type(mesh.num_elements, count),
            pf.IdentityMaterialChannelMapping(count),
        )

    elastic_fixed = identity_field(
        pf.FixedParameterField,
        elastic.fixed_channel_names,
        pf.ElementwiseParameterLayout,
    )
    plastic_fixed = identity_field(
        pf.FixedParameterField,
        plastic.fixed_channel_names,
        pf.ElementwiseParameterLayout,
    )
    elastic_optimizable = identity_field(
        pf.OptimizableParameterField,
        elastic.optimizable_channel_names,
        pf.ConstantParameterLayout,
    )
    plastic_optimizable = identity_field(
        pf.OptimizableParameterField,
        plastic.optimizable_channel_names,
        pf.ConstantParameterLayout,
    )
    parameterization = pf.MaterialParameterization(
        pf.ElasticParameterization(
            elastic, elastic_fixed, elastic_optimizable),
        pf.PlasticParameterization(
            plastic, plastic_fixed, plastic_optimizable),
    )

    if elastic.fixed_channel_names:
        fixed_values = np.asarray(
            pf.project_imported_material_inputs(
                imported.material_catalog, elastic_fixed),
            dtype=np.float64,
        ).reshape(-1)
    else:
        fixed_values = np.empty(0, dtype=np.float64)

    parameter_data = pf.MaterialParameterData(
        elastic=pf.MaterialParameterDataBlock(
            fixed_values=fixed_values,
            initial_optimizable_values=np.asarray(
                elastic_values, dtype=np.float64).reshape(-1),
        ),
        plastic=pf.MaterialParameterDataBlock(
            fixed_values=np.empty(0, dtype=np.float64),
            initial_optimizable_values=np.empty(0, dtype=np.float64),
        ),
    )
    assignment = pf.MaterialAssignment(
        mesh=mesh,
        parameterization=parameterization,
        parameter_data=parameter_data,
        material_frames=pf.GlobalAxesMaterialFrameField(mesh.num_elements),
    )
    return pf.DeformationEnergy(
        assignment,
        formulation=pf.CubicLinear(),
        options=pf.DeformationOptions(
            project_hessian_psd=False,
            enable_material_max_step=False,
        ),
    )


def initial_material_parameters(stretch_knots):
    initial_mu, initial_lambda = _lame_parameters(
        INITIAL_YOUNGS_MODULUS, INITIAL_POISSON_RATIO)
    return np.concatenate([
        initial_mu * (1.0 + 1.0 / stretch_knots**2),
        [initial_lambda],
    ])


def reference_material_parameters(stretch_knots):
    target_mu, target_lambda = _lame_parameters(
        YOUNGS_MODULUS, POISSON_RATIO)
    return np.concatenate([
        target_mu * (1.0 + 1.0 / stretch_knots**2),
        [target_lambda],
    ])


def _constraint_arrays(
    mapping: dict[int, float],
    num_dofs: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    fixed_dofs = np.array(sorted(mapping), dtype=np.int64)
    fixed_values = np.array(
        [mapping[int(dof)] for dof in fixed_dofs],
        dtype=np.float64,
    )
    mask = np.ones(num_dofs, dtype=bool)
    mask[fixed_dofs] = False
    free_dofs = np.flatnonzero(mask).astype(np.int64)
    return fixed_dofs, fixed_values, free_dofs


def _uniaxial_case(
    vertices: np.ndarray,
    stretch: float,
    *,
    confined: bool,
    split: str,
) -> LoadCase:
    num_dofs = vertices.size
    bottom = np.flatnonzero(np.isclose(vertices[:, 1], 0.0))
    top = np.flatnonzero(np.isclose(vertices[:, 1], 1.0))
    displacement = np.zeros_like(vertices)
    displacement[:, 1] = (stretch - 1.0) * vertices[:, 1]
    constraints: dict[int, float] = {}

    for vertex in bottom:
        for component in range(3):
            constraints[3 * int(vertex) + component] = 0.0
    for vertex in top:
        constraints[3 * int(vertex) + 1] = stretch - 1.0
    if confined:
        for vertex in range(len(vertices)):
            constraints[3 * vertex] = 0.0
            constraints[3 * vertex + 2] = 0.0

    fixed_dofs, fixed_values, free_dofs = _constraint_arrays(
        constraints, num_dofs)
    selector = np.zeros(num_dofs, dtype=np.float64)
    selector[3 * top + 1] = 1.0
    protocol = "confined_uniaxial" if confined else "free_uniaxial"
    return LoadCase(
        label=f"{split}_{protocol}_{stretch:.9f}",
        protocol=protocol,
        coordinate=stretch,
        fixed_dofs=fixed_dofs,
        fixed_values=fixed_values,
        free_dofs=free_dofs,
        reaction_selector=selector,
        initial_displacement=displacement.ravel(),
    )


def _shear_case(
    vertices: np.ndarray,
    shear: float,
    *,
    split: str,
) -> LoadCase:
    num_dofs = vertices.size
    bottom = np.flatnonzero(np.isclose(vertices[:, 1], 0.0))
    top = np.flatnonzero(np.isclose(vertices[:, 1], 1.0))
    displacement = np.zeros_like(vertices)
    displacement[:, 0] = shear * vertices[:, 1]
    constraints: dict[int, float] = {}

    for vertex in bottom:
        for component in range(3):
            constraints[3 * int(vertex) + component] = 0.0
    for vertex in top:
        constraints[3 * int(vertex)] = shear
        constraints[3 * int(vertex) + 1] = 0.0

    fixed_dofs, fixed_values, free_dofs = _constraint_arrays(
        constraints, num_dofs)
    selector = np.zeros(num_dofs, dtype=np.float64)
    selector[3 * top] = 1.0
    return LoadCase(
        label=f"{split}_simple_shear_{shear:+.9f}",
        protocol="simple_shear",
        coordinate=shear,
        fixed_dofs=fixed_dofs,
        fixed_values=fixed_values,
        free_dofs=free_dofs,
        reaction_selector=selector,
        initial_displacement=displacement.ravel(),
    )


def _load_cases(
    vertices: np.ndarray,
    stretch_knots: np.ndarray,
    *,
    split: str,
) -> tuple[LoadCase, ...]:
    rest_index = len(stretch_knots) // 2
    if split == "train":
        stretches = [
            value for index, value in enumerate(stretch_knots)
            if index != rest_index
        ]
        shears = (-0.8, -0.4, 0.4, 0.8)
    elif split == "validation":
        stretches = np.sqrt(
            stretch_knots[:-1] * stretch_knots[1:])
        shears = (-0.6, -0.2, 0.2, 0.6)
    elif split == "holdout":
        log_knots = np.log(stretch_knots)
        stretches = np.concatenate([
            np.exp(0.75 * log_knots[:-1] + 0.25 * log_knots[1:]),
            np.exp(0.25 * log_knots[:-1] + 0.75 * log_knots[1:]),
        ])
        shears = (-0.7, -0.5, -0.3, -0.1, 0.1, 0.3, 0.5, 0.7)
    else:
        raise ValueError(f"unknown split {split!r}")

    cases = []
    for stretch in stretches:
        cases.append(_uniaxial_case(
            vertices, float(stretch), confined=False, split=split))
        cases.append(_uniaxial_case(
            vertices, float(stretch), confined=True, split=split))
    for shear in shears:
        cases.append(_shear_case(vertices, shear, split=split))
    return tuple(cases)


def _solve_case(
    energy,
    optimizer,
    case: LoadCase,
    warm_start: np.ndarray | None,
) -> tuple[ReactionState, np.ndarray]:
    problem = ps.OptimizationProblem(objective=energy)
    problem.fix_variables(
        case.fixed_dofs.tolist(),
        case.fixed_values,
        num_dofs=energy.num_dofs,
    )
    initial = (
        case.initial_displacement.copy()
        if warm_start is None
        else np.asarray(warm_start, dtype=np.float64).copy()
    )
    initial[case.fixed_dofs] = case.fixed_values
    result = optimizer.solve(problem, initial)
    displacement = result.x.copy()
    polish_iterations = 0
    for polish_iterations in range(9):
        gradient = energy.gradient(displacement)
        free_gradient = gradient[case.free_dofs]
        if (
            not case.free_dofs.size
            or np.max(np.abs(free_gradient))
            <= STATIC_RESIDUAL_TOLERANCE
        ):
            break
        stiffness = energy.hessian(displacement).to_dense()
        step = np.linalg.solve(
            stiffness[np.ix_(case.free_dofs, case.free_dofs)],
            -free_gradient,
        )
        objective = energy.value(displacement)
        directional_derivative = float(free_gradient @ step)
        current_residual = float(np.max(np.abs(free_gradient)))
        alpha = 1.0
        accepted = False
        for _ in range(18):
            candidate = displacement.copy()
            candidate[case.free_dofs] += alpha * step
            candidate_residual = float(np.max(np.abs(
                energy.gradient(candidate)[case.free_dofs])))
            armijo = energy.value(candidate) <= (
                objective + 1.0e-4 * alpha * directional_derivative)
            if armijo or candidate_residual < current_residual:
                displacement = candidate
                accepted = True
                break
            alpha *= 0.5
        if not accepted:
            raise RuntimeError(
                f"equilibrium polish failed for {case.label}")

    gradient = energy.gradient(displacement)
    free_residual_max = (
        float(np.max(np.abs(gradient[case.free_dofs])))
        if case.free_dofs.size else 0.0
    )
    if free_residual_max > STATIC_RESIDUAL_TOLERANCE:
        raise RuntimeError(
            f"static solve failed for {case.label}: "
            f"status={result.status.name}, residual={free_residual_max:.3e}"
        )
    reaction = float(case.reaction_selector @ gradient)
    return ReactionState(
        reaction=reaction,
        displacement=displacement.copy(),
        iterations=result.iterations + polish_iterations,
        free_residual_max=free_residual_max,
    ), displacement.copy()


def _reaction_parameter_jacobian(
    energy,
    case: LoadCase,
    displacement: np.ndarray,
) -> np.ndarray:
    stiffness = energy.hessian(displacement).to_dense()
    mixed = energy.d2E_dude(displacement).to_dense()
    if case.free_dofs.size:
        free_sensitivity = -np.linalg.solve(
            stiffness[np.ix_(case.free_dofs, case.free_dofs)],
            mixed[case.free_dofs, :],
        )
        total_mixed = (
            mixed
            + stiffness[:, case.free_dofs] @ free_sensitivity
        )
    else:
        total_mixed = mixed
    return case.reaction_selector @ total_mixed


def _target_reactions(
    energy,
    optimizer,
    cases: tuple[LoadCase, ...],
) -> np.ndarray:
    reactions = []
    for case in cases:
        state, _ = _solve_case(
            energy, optimizer, case, warm_start=None)
        reactions.append(state.reaction)
    return np.asarray(reactions, dtype=np.float64)


def build_problem(
    knot_count: int = 17,
    *,
    grid_size: int = 2,
) -> CalibrationProblem:
    if knot_count < 3 or knot_count % 2 == 0:
        raise ValueError("knot_count must be odd and at least three")
    stretch_knots = np.exp(
        np.linspace(np.log(0.5), np.log(2.0), knot_count))
    volume_knots = np.exp(np.linspace(-1.0, 1.0, knot_count))
    imported = _make_import(grid_size)

    target_energy = _make_energy(
        imported,
        pf.NeoHookeanDefinition(),
        np.empty(0, dtype=np.float64),
    )
    vertices = target_energy.vertex_rest_positions
    systematic_energy = _make_energy(
        imported,
        pf.SystematicPokingDefinition(
            stretch_knots,
            knot_count // 2,
            volume_knots,
            knot_count // 2,
        ),
        initial_material_parameters(stretch_knots),
    )
    optimizer = ps.NewtonOptimizer(
        max_iterations=80,
        gradient_tolerance=1.0e-10,
        damping=ps.FixedDamping(),
        line_search=ps.Backtrack(),
    )
    train_cases = _load_cases(
        vertices, stretch_knots, split="train")
    validation_cases = _load_cases(
        vertices, stretch_knots, split="validation")
    holdout_cases = _load_cases(
        vertices, stretch_knots, split="holdout")
    return CalibrationProblem(
        systematic_energy=systematic_energy,
        target_energy=target_energy,
        stretch_knots=stretch_knots,
        volume_knots=volume_knots,
        train_cases=train_cases,
        validation_cases=validation_cases,
        holdout_cases=holdout_cases,
        train_targets=_target_reactions(
            target_energy, optimizer, train_cases),
        validation_targets=_target_reactions(
            target_energy, optimizer, validation_cases),
        holdout_targets=_target_reactions(
            target_energy, optimizer, holdout_cases),
        inner_optimizer=optimizer,
        grid_size=grid_size,
    )


def _smoothness_operator(parameter_count: int) -> np.ndarray:
    if parameter_count < 3:
        return np.empty((0, parameter_count), dtype=np.float64)
    operator = np.zeros(
        (parameter_count - 2, parameter_count), dtype=np.float64)
    rows = np.arange(parameter_count - 2)
    operator[rows, rows] = 1.0
    operator[rows, rows + 1] = -2.0
    operator[rows, rows + 2] = 1.0
    return operator


def reaction_residual_and_jacobian(
    problem: CalibrationProblem,
    theta: np.ndarray,
    split: str = "train",
    *,
    smoothness_weight: float = 0.0,
    include_regularization: bool = True,
) -> tuple[np.ndarray, np.ndarray, list[ReactionState]]:
    if smoothness_weight < 0.0:
        raise ValueError("smoothness_weight must be nonnegative")
    theta = np.asarray(theta, dtype=np.float64)
    parameters = FORCE_SCALE * np.exp(theta)
    problem.systematic_energy.optimizable_parameters.set_elastic_values(
        parameters[None, :])
    cases, targets = problem.data_for(split)
    residuals = []
    jacobians = []
    states = []

    for case, target in zip(cases, targets):
        state, warm_start = _solve_case(
            problem.systematic_energy,
            problem.inner_optimizer,
            case,
            problem.warm_starts.get(case.label),
        )
        problem.warm_starts[case.label] = warm_start
        derivative = _reaction_parameter_jacobian(
            problem.systematic_energy,
            case,
            state.displacement,
        )
        residuals.append((state.reaction - target) / FORCE_SCALE)
        jacobians.append(derivative * parameters / FORCE_SCALE)
        states.append(state)

    residual = np.asarray(residuals, dtype=np.float64)
    jacobian = np.asarray(jacobians, dtype=np.float64)
    if (
        include_regularization
        and split == "train"
        and smoothness_weight > 0.0
    ):
        operator = _smoothness_operator(len(problem.stretch_knots))
        scale = np.sqrt(smoothness_weight)
        regularization_residual = scale * operator @ theta[:-1]
        regularization_jacobian = np.zeros(
            (operator.shape[0], theta.size), dtype=np.float64)
        regularization_jacobian[:, :-1] = scale * operator
        residual = np.concatenate([residual, regularization_residual])
        jacobian = np.vstack([jacobian, regularization_jacobian])
    return residual, jacobian, states


def _data_metrics(residual: np.ndarray) -> dict[str, float]:
    residual = np.asarray(residual, dtype=np.float64)
    return {
        "rmse": float(np.sqrt(np.mean(residual**2))),
        "mean_absolute_error": float(np.mean(np.abs(residual))),
        "max_absolute_error": float(np.max(np.abs(residual))),
    }


def evaluate_parameters(
    problem: CalibrationProblem,
    parameters: np.ndarray,
    split: str,
) -> dict[str, float]:
    parameters = np.asarray(parameters, dtype=np.float64)
    if np.any(parameters <= 0.0) or not np.all(np.isfinite(parameters)):
        raise ValueError("material parameters must be finite and positive")
    problem.reset_warm_starts()
    residual, _, states = reaction_residual_and_jacobian(
        problem,
        np.log(parameters / FORCE_SCALE),
        split,
        include_regularization=False,
    )
    metrics = _data_metrics(residual)
    metrics["max_free_residual"] = float(
        max(state.free_residual_max for state in states))
    metrics["mean_static_iterations"] = float(
        np.mean([state.iterations for state in states]))
    return metrics


def fit_parameters(
    problem: CalibrationProblem,
    *,
    max_iterations: int = 40,
    gradient_tolerance: float = 1.0e-9,
    smoothness_weight: float = 3.0e-3,
) -> CalibrationResult:
    if max_iterations <= 0:
        raise ValueError("max_iterations must be positive")
    if smoothness_weight < 0.0:
        raise ValueError("smoothness_weight must be nonnegative")
    initial_parameters = initial_material_parameters(
        problem.stretch_knots)
    theta = np.log(initial_parameters / FORCE_SCALE)
    damping = 1.0e-5
    history = []
    termination_reason = "maximum_iterations"

    for iteration in range(max_iterations):
        problem.reset_warm_starts()
        residual, jacobian, states = reaction_residual_and_jacobian(
            problem,
            theta,
            "train",
            smoothness_weight=smoothness_weight,
        )
        objective = 0.5 * float(residual @ residual)
        gradient = jacobian.T @ residual
        gradient_inf = float(np.linalg.norm(gradient, ord=np.inf))
        data_count = len(problem.train_cases)
        data_metrics = _data_metrics(residual[:data_count])
        entry = {
            "iteration": float(iteration),
            "objective": objective,
            "train_rmse": data_metrics["rmse"],
            "gradient_inf": gradient_inf,
            "damping": damping,
            "step_norm": 0.0,
            "line_search_alpha": 0.0,
            "max_free_residual": float(max(
                state.free_residual_max for state in states)),
        }
        history.append(entry)
        if gradient_inf < gradient_tolerance:
            termination_reason = "gradient_tolerance"
            break

        normal = jacobian.T @ jacobian
        diagonal = np.maximum(np.diag(normal), 1.0e-12)
        accepted = False
        for _ in range(12):
            step = np.linalg.solve(
                normal + damping * np.diag(diagonal),
                -gradient,
            )
            max_component = float(np.max(np.abs(step)))
            if max_component > 0.75:
                step *= 0.75 / max_component

            alpha = 1.0
            for _ in range(14):
                candidate = theta + alpha * step
                problem.reset_warm_starts()
                candidate_residual, _, _ = (
                    reaction_residual_and_jacobian(
                        problem,
                        candidate,
                        "train",
                        smoothness_weight=smoothness_weight,
                    )
                )
                candidate_objective = (
                    0.5
                    * float(candidate_residual @ candidate_residual)
                )
                if candidate_objective < objective:
                    theta = candidate
                    entry["step_norm"] = float(
                        np.linalg.norm(alpha * step))
                    entry["line_search_alpha"] = alpha
                    damping = max(damping / 3.0, 1.0e-12)
                    accepted = True
                    break
                alpha *= 0.5
            if accepted:
                break
            damping *= 10.0
        if not accepted:
            termination_reason = "no_decreasing_step"
            break
        if entry["step_norm"] < 1.0e-9:
            termination_reason = "step_tolerance"
            break

    fitted_parameters = FORCE_SCALE * np.exp(theta)
    initial_train = evaluate_parameters(
        problem, initial_parameters, "train")
    initial_validation = evaluate_parameters(
        problem, initial_parameters, "validation")
    initial_holdout = evaluate_parameters(
        problem, initial_parameters, "holdout")
    fitted_train = evaluate_parameters(
        problem, fitted_parameters, "train")
    fitted_validation = evaluate_parameters(
        problem, fitted_parameters, "validation")
    fitted_holdout = evaluate_parameters(
        problem, fitted_parameters, "holdout")

    problem.reset_warm_starts()
    data_residual, data_jacobian, _ = (
        reaction_residual_and_jacobian(
            problem,
            theta,
            "train",
            include_regularization=False,
        )
    )
    singular_values = np.linalg.svd(
        data_jacobian, compute_uv=False)
    rank = int(np.linalg.matrix_rank(data_jacobian))
    condition = (
        float(singular_values[0] / singular_values[-1])
        if singular_values[-1] > 0.0 else float("inf")
    )
    operator = _smoothness_operator(len(problem.stretch_knots))
    smoothness = float(np.linalg.norm(operator @ theta[:-1])**2)
    summary = {
        "target_material": {
            "youngs_modulus": YOUNGS_MODULUS,
            "poisson_ratio": POISSON_RATIO,
        },
        "initial_material": {
            "youngs_modulus": INITIAL_YOUNGS_MODULUS,
            "poisson_ratio": INITIAL_POISSON_RATIO,
        },
        "knot_count": int(len(problem.stretch_knots)),
        "grid_size": problem.grid_size,
        "num_elements": problem.grid_size**3,
        "num_displacement_dofs": problem.systematic_energy.num_dofs,
        "num_parameters": int(len(fitted_parameters)),
        "num_train_cases": len(problem.train_cases),
        "num_validation_cases": len(problem.validation_cases),
        "num_holdout_cases": len(problem.holdout_cases),
        "smoothness_weight": smoothness_weight,
        "iterations": len(history),
        "termination_reason": termination_reason,
        "final_objective": history[-1]["objective"],
        "data_jacobian_rank": rank,
        "data_jacobian_condition": condition,
        "data_jacobian_singular_values": singular_values.tolist(),
        "log_curvature_smoothness": smoothness,
        "initial_train": initial_train,
        "initial_validation": initial_validation,
        "initial_holdout": initial_holdout,
        "fitted_train": fitted_train,
        "fitted_validation": fitted_validation,
        "fitted_holdout": fitted_holdout,
        "minimum_fitted_parameter": float(
            np.min(fitted_parameters)),
        "final_data_residual_norm": float(
            np.linalg.norm(data_residual)),
    }
    return CalibrationResult(
        initial_parameters=initial_parameters,
        fitted_parameters=fitted_parameters,
        history=history,
        summary=summary,
    )


def _predictions(
    problem: CalibrationProblem,
    parameters: np.ndarray,
    split: str,
) -> tuple[list[str], np.ndarray, np.ndarray]:
    problem.reset_warm_starts()
    theta = np.log(np.asarray(parameters) / FORCE_SCALE)
    residual, _, _ = reaction_residual_and_jacobian(
        problem, theta, split, include_regularization=False)
    cases, targets = problem.data_for(split)
    predicted = targets + FORCE_SCALE * residual
    return [case.label for case in cases], targets, predicted


def save_results(
    problem: CalibrationProblem,
    result: CalibrationResult,
    output_dir: Path,
    *,
    make_plots: bool,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(
        json.dumps(result.summary, indent=2) + "\n")

    history_fields = (
        "iteration", "objective", "train_rmse", "gradient_inf",
        "damping", "step_norm", "line_search_alpha",
        "max_free_residual",
    )
    with (output_dir / "optimization_history.csv").open(
        "w", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=history_fields)
        writer.writeheader()
        writer.writerows(result.history)

    reference = reference_material_parameters(problem.stretch_knots)
    with (output_dir / "fitted_parameters.csv").open(
        "w", newline=""
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "name", "coordinate", "initial", "fitted",
            "neo_hookean_reference",
        ])
        for index, stretch in enumerate(problem.stretch_knots):
            writer.writerow([
                f"f_dd_{index}", stretch,
                result.initial_parameters[index],
                result.fitted_parameters[index],
                reference[index],
            ])
        writer.writerow([
            "lambda", "", result.initial_parameters[-1],
            result.fitted_parameters[-1], reference[-1],
        ])

    train_labels, train_target, train_predicted = _predictions(
        problem, result.fitted_parameters, "train")
    validation_labels, validation_target, validation_predicted = (
        _predictions(
            problem, result.fitted_parameters, "validation")
    )
    holdout_labels, holdout_target, holdout_predicted = _predictions(
        problem, result.fitted_parameters, "holdout")
    np.savez(
        output_dir / "reaction_predictions.npz",
        train_labels=np.asarray(train_labels),
        train_target=train_target,
        train_predicted=train_predicted,
        validation_labels=np.asarray(validation_labels),
        validation_target=validation_target,
        validation_predicted=validation_predicted,
        holdout_labels=np.asarray(holdout_labels),
        holdout_target=holdout_target,
        holdout_predicted=holdout_predicted,
    )

    if not make_plots:
        return
    if find_spec("matplotlib") is None:
        print("Matplotlib is not installed; skipping plots.")
        return
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, 3, figsize=(14, 4))
    axes[0].plot(
        problem.stretch_knots,
        reference[:-1] / FORCE_SCALE,
        "k--",
        label="NH curvature sample",
    )
    axes[0].plot(
        problem.stretch_knots,
        result.initial_parameters[:-1] / FORCE_SCALE,
        "o-",
        label="initial",
    )
    axes[0].plot(
        problem.stretch_knots,
        result.fitted_parameters[:-1] / FORCE_SCALE,
        "o-",
        label="fitted",
    )
    axes[0].set(
        xlabel="stretch knot",
        ylabel=r"$f''/E$",
        title="Stretch curvature parameters",
    )
    axes[0].legend()

    axes[1].semilogy(
        [row["iteration"] for row in result.history],
        [row["objective"] for row in result.history],
        "o-",
    )
    axes[1].set(
        xlabel="iteration",
        ylabel="objective",
        title="Equilibrium Gauss-Newton",
    )

    holdout_cases, _ = problem.data_for("holdout")
    for protocol in sorted({
        case.protocol for case in holdout_cases
    }):
        indices = [
            index for index, case in enumerate(holdout_cases)
            if case.protocol == protocol
        ]
        coordinates = np.asarray([
            holdout_cases[index].coordinate for index in indices])
        order = np.argsort(coordinates)
        selected = np.asarray(indices)[order]
        axes[2].plot(
            coordinates[order],
            holdout_target[selected] / FORCE_SCALE,
            "--",
            label=f"{protocol} target",
        )
        axes[2].plot(
            coordinates[order],
            holdout_predicted[selected] / FORCE_SCALE,
            "o",
            label=f"{protocol} fitted",
        )
    axes[2].set(
        xlabel="stretch or shear",
        ylabel="reaction / E",
        title="Holdout reaction curves",
    )
    axes[2].legend(fontsize=7)
    figure.tight_layout()
    figure.savefig(
        output_dir / "fit_diagnostics.png", dpi=180)
    plt.close(figure)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--knot-count", type=int, default=17)
    parser.add_argument("--grid-size", type=int, default=2)
    parser.add_argument("--max-iterations", type=int, default=40)
    parser.add_argument(
        "--smoothness-weight", type=float, default=3.0e-3)
    parser.add_argument(
        "--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args(argv)

    problem = build_problem(
        args.knot_count, grid_size=args.grid_size)
    result = fit_parameters(
        problem,
        max_iterations=args.max_iterations,
        smoothness_weight=args.smoothness_weight,
    )
    save_results(
        problem,
        result,
        args.output_dir,
        make_plots=not args.no_plots,
    )

    for row in result.history:
        iteration = int(row["iteration"])
        if iteration < 5 or iteration % 5 == 0:
            print(
                f"iter {iteration:2d}  "
                f"objective={row['objective']:.8e}  "
                f"train_rmse={row['train_rmse']:.8e}  "
                f"|g|inf={row['gradient_inf']:.3e}  "
                f"equilibrium={row['max_free_residual']:.3e}"
            )
    summary = result.summary
    print(
        "train RMSE: "
        f"{summary['initial_train']['rmse']:.6e} -> "
        f"{summary['fitted_train']['rmse']:.6e}"
    )
    print(
        "validation RMSE: "
        f"{summary['initial_validation']['rmse']:.6e} -> "
        f"{summary['fitted_validation']['rmse']:.6e}"
    )
    print(
        "holdout RMSE: "
        f"{summary['initial_holdout']['rmse']:.6e} -> "
        f"{summary['fitted_holdout']['rmse']:.6e}"
    )
    print(
        "data Jacobian rank/condition: "
        f"{summary['data_jacobian_rank']}/"
        f"{summary['data_jacobian_condition']:.3e}"
    )
    print(
        "termination:",
        summary["termination_reason"],
        f"after {summary['iterations']} iterations",
    )
    print("saved results ->", args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
