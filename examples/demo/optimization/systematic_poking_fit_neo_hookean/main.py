"""Fit Systematic Poking parameters to synthetic Neo-Hookean stress data."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from importlib.util import find_spec
import json
from pathlib import Path

import numpy as np
import pypgo as pgo
import pypgo.fem as pf


CASE_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = CASE_DIR / "output"
YOUNGS_MODULUS = 2.0e5
POISSON_RATIO = 0.35
INITIAL_YOUNGS_MODULUS = 1.2e5
INITIAL_POISSON_RATIO = 0.25


@dataclass(frozen=True)
class LoadCase:
    label: str
    F: np.ndarray


@dataclass
class CalibrationProblem:
    systematic_energy: object
    target_energy: object
    stretch_knots: np.ndarray
    volume_knots: np.ndarray
    affine_basis: np.ndarray
    train_cases: tuple[LoadCase, ...]
    holdout_cases: tuple[LoadCase, ...]
    train_targets: np.ndarray
    holdout_targets: np.ndarray

    def target_for(self, split: str) -> tuple[tuple[LoadCase, ...], np.ndarray]:
        if split == "train":
            return self.train_cases, self.train_targets
        if split == "holdout":
            return self.holdout_cases, self.holdout_targets
        raise ValueError(f"unknown split {split!r}")


@dataclass
class CalibrationResult:
    initial_parameters: np.ndarray
    fitted_parameters: np.ndarray
    oracle_parameters: np.ndarray
    history: list[dict[str, float]]
    summary: dict[str, object]


def _lame_parameters(youngs_modulus: float, poisson_ratio: float):
    mu = youngs_modulus / (2.0 * (1.0 + poisson_ratio))
    lame_lambda = (
        youngs_modulus
        * poisson_ratio
        / ((1.0 + poisson_ratio) * (1.0 - 2.0 * poisson_ratio))
    )
    return mu, lame_lambda


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
            E=YOUNGS_MODULUS,
            nu=POISSON_RATIO,
        ),
    )
    return volume


def _make_energy(volume, elastic, elastic_values):
    mesh = pf.SimulationMesh(volume)
    plastic = pf.VolumetricPlasticityDefinition(dofs=0)

    fixed_channel_order = {
        "neo_hookean": ("E", "nu"),
    }.get(elastic.name, ())
    if len(fixed_channel_order) != elastic.num_fixed_channels:
        raise ValueError(f"unsupported fixed channels for {elastic.name}")
    materials = volume.to_veg_file().materials
    assignments = volume.element_material_indices
    fixed_values = np.asarray([
        [getattr(materials[int(assignments[element])], name)
         for name in fixed_channel_order]
        for element in range(mesh.num_elements)
    ], dtype=np.float64).reshape(mesh.num_elements, -1)

    elastic_values = np.asarray(elastic_values, dtype=np.float64).reshape(-1)
    channels = elastic.num_optimizable_channels
    if elastic_values.size == channels:
        elastic_values = np.broadcast_to(
            elastic_values.reshape(1, channels),
            (mesh.num_elements, channels),
        ).copy()
    elif elastic_values.size != mesh.num_elements * channels:
        raise ValueError("elastic_values must be constant or elementwise")

    material_binding = pf.MaterialBinding(
        pf.ElasticMaterialBinding(elastic, mesh.num_elements, fixed_values),
        pf.PlasticMaterialBinding(
            plastic, mesh.num_elements,
            np.empty((mesh.num_elements, 0), dtype=np.float64)),
    )
    material_state = pf.MaterialState(elastic_values, np.empty(0))
    operator = pf.DeformationEnergyOperator(
        mesh, material_binding,
        formulation=pf.CubicLinear(),
        options=pf.DeformationOptions(
            project_hessian_psd=False,
            enable_material_max_step=False,
        ),
    )
    return pf.DeformationPotentialEnergy(
        operator, material_state)


def _axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=np.float64)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ])
    return (
        np.eye(3)
        + np.sin(angle) * cross
        + (1.0 - np.cos(angle)) * (cross @ cross)
    )


def _random_cases(count: int, seed: int, prefix: str):
    rng = np.random.default_rng(seed)
    cases = []
    while len(cases) < count:
        log_stretches = rng.uniform(-0.42, 0.42, size=3)
        if abs(np.sum(log_stretches)) > 0.75:
            continue
        left = _axis_angle(rng.normal(size=3), rng.uniform(-0.9, 0.9))
        right = _axis_angle(rng.normal(size=3), rng.uniform(-0.9, 0.9))
        F = left @ np.diag(np.exp(log_stretches)) @ right.T
        cases.append(LoadCase(f"{prefix}_random_{len(cases):02d}", F))
    return cases


def _load_cases(training: bool):
    if training:
        uniaxial = (-0.55, -0.40, -0.25, -0.10, 0.10, 0.25, 0.40, 0.55)
        biaxial = (-0.35, -0.25, -0.15, 0.15, 0.25, 0.35)
        volumetric = (-0.22, -0.15, -0.08, 0.08, 0.15, 0.22)
        isochoric = (-0.45, -0.30, -0.15, 0.15, 0.30, 0.45)
        shear = (-0.80, -0.50, -0.25, 0.25, 0.50, 0.80)
        random_count, random_seed, prefix = 12, 20260729, "train"
    else:
        uniaxial = (-0.475, -0.325, -0.175, 0.0, 0.175, 0.325, 0.475)
        biaxial = (-0.30, -0.20, -0.10, 0.10, 0.20, 0.30)
        volumetric = (-0.19, -0.11, -0.04, 0.04, 0.11, 0.19)
        isochoric = (-0.375, -0.225, -0.075, 0.075, 0.225, 0.375)
        shear = (-0.65, -0.375, -0.125, 0.125, 0.375, 0.65)
        random_count, random_seed, prefix = 18, 20260730, "holdout"

    cases = []
    for value in uniaxial:
        cases.append(LoadCase(
            f"{prefix}_uniaxial_{value:+.3f}",
            np.diag([np.exp(value), 1.0, 1.0]),
        ))
    for value in biaxial:
        stretch = np.exp(value)
        cases.append(LoadCase(
            f"{prefix}_biaxial_{value:+.3f}",
            np.diag([stretch, stretch, 1.0]),
        ))
    for value in volumetric:
        cases.append(LoadCase(
            f"{prefix}_volumetric_{value:+.3f}",
            np.exp(value) * np.eye(3),
        ))
    for value in isochoric:
        cases.append(LoadCase(
            f"{prefix}_isochoric_{value:+.3f}",
            np.diag([np.exp(value), np.exp(-value), 1.0]),
        ))
    for value in shear:
        F = np.eye(3)
        F[0, 1] = value
        cases.append(LoadCase(f"{prefix}_shear_{value:+.3f}", F))
    cases.extend(_random_cases(random_count, random_seed, prefix))
    return tuple(cases)


def _affine_basis(rest_positions):
    basis = np.empty((rest_positions.size, 9), dtype=np.float64)
    column = 0
    for row in range(3):
        for col in range(3):
            dF = np.zeros((3, 3), dtype=np.float64)
            dF[row, col] = 1.0
            basis[:, column] = (rest_positions @ dF.T).ravel()
            column += 1
    return basis


def _stress(energy, affine_basis, F):
    rest_positions = energy.vertex_rest_positions
    displacement = (
        rest_positions @ (F - np.eye(3)).T
    ).ravel()
    gradient = energy.gradient(displacement)
    return (affine_basis.T @ gradient).reshape((3, 3))


def _stress_and_parameter_jacobian(energy, affine_basis, F):
    rest_positions = energy.vertex_rest_positions
    displacement = (
        rest_positions @ (F - np.eye(3)).T
    ).ravel()
    stress = (affine_basis.T @ energy.gradient(displacement)).reshape((3, 3))
    channels = energy.material_binding.elastic.num_optimizable_channels
    parameter_jacobian = np.empty((9, channels), dtype=np.float64)
    for output_index in range(9):
        elementwise_vjp = energy.elastic_material_vjp(
            displacement, affine_basis[:, output_index])
        parameter_jacobian[output_index] = (
            elementwise_vjp.reshape(-1, channels).sum(axis=0)
        )
    return stress, parameter_jacobian


def _with_homogeneous_elastic_values(energy, values):
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    elementwise = np.broadcast_to(
        values,
        (energy.material_binding.num_elements, values.size),
    ).copy()
    return energy.material_state.with_elastic_values(elementwise)


def _target_stresses(energy, affine_basis, cases):
    return np.asarray([
        _stress(energy, affine_basis, case.F)
        for case in cases
    ])


def build_problem(knot_count: int = 17):
    if knot_count < 3 or knot_count % 2 == 0:
        raise ValueError("knot_count must be odd and at least three")
    stretch_knots = np.exp(
        np.linspace(np.log(0.5), np.log(2.0), knot_count))
    volume_knots = np.exp(np.linspace(-1.0, 1.0, knot_count))
    imported = _unit_cube_import()

    target_energy = _make_energy(
        imported,
        pf.NeoHookeanDefinition(),
        np.empty(0, dtype=np.float64),
    )
    initial_parameters = initial_material_parameters(stretch_knots)
    systematic_energy = _make_energy(
        imported,
        pf.SystematicPokingDefinition(
            stretch_knots,
            knot_count // 2,
            volume_knots,
            knot_count // 2,
        ),
        initial_parameters,
    )
    affine_basis = _affine_basis(
        systematic_energy.vertex_rest_positions)
    train_cases = _load_cases(training=True) + tuple(
        LoadCase(
            f"train_knot_uniaxial_{index:02d}",
            np.diag([stretch, 1.0, 1.0]),
        )
        for index, stretch in enumerate(stretch_knots)
        if index != knot_count // 2
    )
    holdout_cases = _load_cases(training=False)
    return CalibrationProblem(
        systematic_energy=systematic_energy,
        target_energy=target_energy,
        stretch_knots=stretch_knots,
        volume_knots=volume_knots,
        affine_basis=affine_basis,
        train_cases=train_cases,
        holdout_cases=holdout_cases,
        train_targets=_target_stresses(
            target_energy, affine_basis, train_cases),
        holdout_targets=_target_stresses(
            target_energy, affine_basis, holdout_cases),
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


def stress_residual_and_jacobian(
    problem: CalibrationProblem,
    theta: np.ndarray,
    split: str = "train",
):
    parameters = YOUNGS_MODULUS * np.exp(
        np.asarray(theta, dtype=np.float64))
    problem.systematic_energy = pf.DeformationPotentialEnergy(
        problem.systematic_energy.energy_operator,
        _with_homogeneous_elastic_values(
            problem.systematic_energy, parameters))
    cases, targets = problem.target_for(split)
    residuals = []
    jacobians = []
    for case, target in zip(cases, targets):
        stress, parameter_jacobian = _stress_and_parameter_jacobian(
            problem.systematic_energy,
            problem.affine_basis,
            case.F,
        )
        residuals.append(
            ((stress - target) / YOUNGS_MODULUS).ravel())
        jacobians.append(
            parameter_jacobian
            * (parameters / YOUNGS_MODULUS)[None, :])
    return np.concatenate(residuals), np.vstack(jacobians)


def physical_linear_system(
    problem: CalibrationProblem,
    split: str = "train",
):
    parameter_count = len(problem.stretch_knots) + 1
    unit_parameters = np.ones(parameter_count, dtype=np.float64)
    problem.systematic_energy = pf.DeformationPotentialEnergy(
        problem.systematic_energy.energy_operator,
        _with_homogeneous_elastic_values(
            problem.systematic_energy, unit_parameters))
    cases, targets = problem.target_for(split)
    design_blocks = []
    target_blocks = []
    for case, target in zip(cases, targets):
        _, parameter_jacobian = _stress_and_parameter_jacobian(
            problem.systematic_energy,
            problem.affine_basis,
            case.F,
        )
        design_blocks.append(parameter_jacobian)
        target_blocks.append((target / YOUNGS_MODULUS).ravel())
    return np.vstack(design_blocks), np.concatenate(target_blocks)


def _split_metrics(residual):
    reshaped = np.asarray(residual).reshape((-1, 9))
    case_norms = np.linalg.norm(reshaped, axis=1)
    return {
        "rmse": float(np.sqrt(np.mean(residual**2))),
        "mean_case_error": float(np.mean(case_norms)),
        "max_case_error": float(np.max(case_norms)),
    }


def _linear_metrics(design, target, normalized_parameters):
    return _split_metrics(
        design @ np.asarray(normalized_parameters) - target)


def evaluate_parameters(problem, parameters, split):
    parameters = np.asarray(parameters, dtype=np.float64)
    if np.any(parameters <= 0.0) or not np.all(np.isfinite(parameters)):
        raise ValueError("material parameters must be finite and positive")
    theta = np.log(parameters / YOUNGS_MODULUS)
    residual, _ = stress_residual_and_jacobian(
        problem, theta, split)
    return _split_metrics(residual)


def fit_parameters(
    problem: CalibrationProblem,
    *,
    max_iterations: int = 60,
    gradient_tolerance: float = 1.0e-10,
):
    if max_iterations <= 0:
        raise ValueError("max_iterations must be positive")
    initial_parameters = initial_material_parameters(
        problem.stretch_knots)
    theta = np.log(initial_parameters / YOUNGS_MODULUS)
    damping = 1.0e-6
    history = []
    termination_reason = "maximum_iterations"

    for iteration in range(max_iterations):
        residual, jacobian = stress_residual_and_jacobian(
            problem, theta, "train")
        objective = 0.5 * float(residual @ residual)
        gradient = jacobian.T @ residual
        gradient_inf = float(np.linalg.norm(gradient, ord=np.inf))
        train_metrics = _split_metrics(residual)
        entry = {
            "iteration": float(iteration),
            "objective": objective,
            "train_rmse": train_metrics["rmse"],
            "gradient_inf": gradient_inf,
            "damping": damping,
            "step_norm": 0.0,
            "line_search_alpha": 0.0,
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
            if max_component > 1.0:
                step /= max_component

            alpha = 1.0
            for _ in range(16):
                candidate = theta + alpha * step
                candidate_residual, _ = stress_residual_and_jacobian(
                    problem, candidate, "train")
                candidate_objective = (
                    0.5 * float(candidate_residual @ candidate_residual)
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
        if entry["step_norm"] < 1.0e-10:
            termination_reason = "step_tolerance"
            break

    fitted_parameters = YOUNGS_MODULUS * np.exp(theta)
    design, target = physical_linear_system(problem, "train")
    oracle_normalized, _, oracle_rank, singular_values = np.linalg.lstsq(
        design, target, rcond=None)
    oracle_parameters = YOUNGS_MODULUS * oracle_normalized

    initial_train = evaluate_parameters(
        problem, initial_parameters, "train")
    initial_holdout = evaluate_parameters(
        problem, initial_parameters, "holdout")
    fitted_train = evaluate_parameters(
        problem, fitted_parameters, "train")
    fitted_holdout = evaluate_parameters(
        problem, fitted_parameters, "holdout")
    oracle_train = _linear_metrics(
        design, target, oracle_normalized)
    holdout_design, holdout_target = physical_linear_system(
        problem, "holdout")
    oracle_holdout = _linear_metrics(
        holdout_design, holdout_target, oracle_normalized)
    condition = float(singular_values[0] / singular_values[-1])
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
        "stretch_knot_range": [
            float(problem.stretch_knots[0]),
            float(problem.stretch_knots[-1]),
        ],
        "volume_knot_range": [
            float(problem.volume_knots[0]),
            float(problem.volume_knots[-1]),
        ],
        "num_parameters": int(len(fitted_parameters)),
        "num_train_cases": len(problem.train_cases),
        "num_holdout_cases": len(problem.holdout_cases),
        "iterations": len(history),
        "termination_reason": termination_reason,
        "final_objective": history[-1]["objective"],
        "design_rank": int(oracle_rank),
        "design_condition": condition,
        "oracle_all_positive": bool(np.all(oracle_parameters > 0.0)),
        "initial_train": initial_train,
        "initial_holdout": initial_holdout,
        "fitted_train": fitted_train,
        "fitted_holdout": fitted_holdout,
        "oracle_train": oracle_train,
        "oracle_holdout": oracle_holdout,
        "fitted_oracle_relative_error": float(
            np.linalg.norm(fitted_parameters - oracle_parameters)
            / np.linalg.norm(oracle_parameters)),
        "minimum_fitted_parameter": float(np.min(fitted_parameters)),
        "minimum_oracle_parameter": float(np.min(oracle_parameters)),
    }
    return CalibrationResult(
        initial_parameters=initial_parameters,
        fitted_parameters=fitted_parameters,
        oracle_parameters=oracle_parameters,
        history=history,
        summary=summary,
    )


def _predictions(problem, parameters, split):
    problem.systematic_energy = pf.DeformationPotentialEnergy(
        problem.systematic_energy.energy_operator,
        _with_homogeneous_elastic_values(
            problem.systematic_energy, parameters))
    cases, targets = problem.target_for(split)
    predicted = np.asarray([
        _stress_and_parameter_jacobian(
            problem.systematic_energy,
            problem.affine_basis,
            case.F,
        )[0]
        for case in cases
    ])
    return [case.label for case in cases], targets, predicted


def save_results(
    problem: CalibrationProblem,
    result: CalibrationResult,
    output_dir: Path,
    *,
    make_plots: bool,
):
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(
        json.dumps(result.summary, indent=2) + "\n")

    history_fields = (
        "iteration", "objective", "train_rmse", "gradient_inf",
        "damping", "step_norm", "line_search_alpha",
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
            "linear_oracle", "reference_sample",
        ])
        for index, stretch in enumerate(problem.stretch_knots):
            writer.writerow([
                f"f_dd_{index}", stretch,
                result.initial_parameters[index],
                result.fitted_parameters[index],
                result.oracle_parameters[index],
                reference[index],
            ])
        writer.writerow([
            "lambda", "", result.initial_parameters[-1],
            result.fitted_parameters[-1],
            result.oracle_parameters[-1],
            reference[-1],
        ])

    train_labels, train_target, train_predicted = _predictions(
        problem, result.fitted_parameters, "train")
    holdout_labels, holdout_target, holdout_predicted = _predictions(
        problem, result.fitted_parameters, "holdout")
    np.savez(
        output_dir / "stress_predictions.npz",
        train_labels=np.asarray(train_labels),
        train_target=train_target,
        train_predicted=train_predicted,
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
        reference[:-1] / YOUNGS_MODULUS,
        "k--", label="NH curvature sample")
    axes[0].plot(
        problem.stretch_knots,
        result.initial_parameters[:-1] / YOUNGS_MODULUS,
        "o-", label="initial")
    axes[0].plot(
        problem.stretch_knots,
        result.fitted_parameters[:-1] / YOUNGS_MODULUS,
        "o-", label="fitted")
    axes[0].set(
        xlabel="stretch knot",
        ylabel=r"$f''/E$",
        title="Stretch curvature parameters",
    )
    axes[0].legend()

    history = result.history
    axes[1].semilogy(
        [row["iteration"] for row in history],
        [row["objective"] for row in history],
        "o-",
    )
    axes[1].set(
        xlabel="iteration",
        ylabel="objective",
        title="Gauss-Newton convergence",
    )

    axes[2].scatter(
        holdout_target.ravel() / YOUNGS_MODULUS,
        holdout_predicted.ravel() / YOUNGS_MODULUS,
        s=8,
        alpha=0.6,
    )
    limits = axes[2].get_xlim()
    lower = min(limits[0], axes[2].get_ylim()[0])
    upper = max(limits[1], axes[2].get_ylim()[1])
    axes[2].plot([lower, upper], [lower, upper], "k--")
    axes[2].set(
        xlabel="target P/E",
        ylabel="fitted P/E",
        title="Holdout stress components",
    )
    figure.tight_layout()
    figure.savefig(output_dir / "fit_diagnostics.png", dpi=180)
    plt.close(figure)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--knot-count", type=int, default=17)
    parser.add_argument("--max-iterations", type=int, default=60)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args(argv)

    problem = build_problem(args.knot_count)
    result = fit_parameters(
        problem, max_iterations=args.max_iterations)
    save_results(
        problem,
        result,
        args.output_dir,
        make_plots=not args.no_plots,
    )

    summary = result.summary
    for row in result.history:
        iteration = int(row["iteration"])
        if iteration < 5 or iteration % 5 == 0:
            print(
                f"iter {iteration:2d}  "
                f"objective={row['objective']:.8e}  "
                f"train_rmse={row['train_rmse']:.8e}  "
                f"|g|inf={row['gradient_inf']:.3e}"
            )
    print(
        "train RMSE: "
        f"{summary['initial_train']['rmse']:.6e} -> "
        f"{summary['fitted_train']['rmse']:.6e}"
    )
    print(
        "holdout RMSE: "
        f"{summary['initial_holdout']['rmse']:.6e} -> "
        f"{summary['fitted_holdout']['rmse']:.6e}"
    )
    print(
        f"design rank/condition: {summary['design_rank']}/"
        f"{summary['design_condition']:.3e}"
    )
    print(
        "fit vs linear oracle: "
        f"{summary['fitted_oracle_relative_error']:.3e}"
    )
    print("saved results ->", args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
