"""Regression tests for the Systematic Poking material-fitting demo."""

import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[2]
DEMO = (
    ROOT
    / "examples"
    / "demo"
    / "optimization"
    / "systematic_poking_fit_neo_hookean"
)


def _load_demo():
    name = "systematic_poking_fit_neo_hookean_demo"
    spec = importlib.util.spec_from_file_location(name, DEMO / "main.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def demo():
    return _load_demo()


@pytest.fixture(scope="module")
def problem(demo):
    return demo.build_problem(17)


@pytest.fixture(scope="module")
def fit(demo, problem):
    return demo.fit_parameters(problem, max_iterations=20)


def test_training_design_excites_every_material_parameter(demo, problem):
    design, _ = demo.physical_linear_system(problem, "train")
    singular_values = np.linalg.svd(design, compute_uv=False)

    assert np.linalg.matrix_rank(design) == 18
    assert singular_values[0] / singular_values[-1] < 1.0e3
    knot_cases = [
        case for case in problem.train_cases
        if case.label.startswith("train_knot_uniaxial_")
    ]
    assert len(knot_cases) == 16
    assert min(case.F[0, 0] for case in knot_cases) == pytest.approx(0.5)
    assert max(case.F[0, 0] for case in knot_cases) == pytest.approx(2.0)


def test_physical_parameter_model_is_linear(demo, problem):
    parameters = demo.initial_material_parameters(problem.stretch_knots)
    theta = np.log(parameters / demo.YOUNGS_MODULUS)
    residual, _ = demo.stress_residual_and_jacobian(
        problem, theta, "train")
    design, target = demo.physical_linear_system(problem, "train")

    np.testing.assert_allclose(
        residual,
        design @ (parameters / demo.YOUNGS_MODULUS) - target,
        rtol=2.0e-12,
        atol=2.0e-12,
    )


def test_log_parameter_stress_jacobian_matches_finite_difference(
    demo, problem
):
    parameters = demo.initial_material_parameters(problem.stretch_knots)
    theta = np.log(parameters / demo.YOUNGS_MODULUS)
    _, analytic = demo.stress_residual_and_jacobian(
        problem, theta, "train")
    finite_difference = np.empty_like(analytic)
    step = 1.0e-6

    for parameter_index in range(theta.size):
        direction = np.zeros_like(theta)
        direction[parameter_index] = step
        residual_plus, _ = demo.stress_residual_and_jacobian(
            problem, theta + direction, "train")
        residual_minus, _ = demo.stress_residual_and_jacobian(
            problem, theta - direction, "train")
        finite_difference[:, parameter_index] = (
            residual_plus - residual_minus) / (2.0 * step)

    np.testing.assert_allclose(
        analytic,
        finite_difference,
        rtol=2.0e-7,
        atol=2.0e-9,
    )


def test_fit_reaches_positive_linear_optimum_and_generalizes(
    demo, fit
):
    summary = fit.summary

    assert np.all(fit.fitted_parameters > 0.0)
    assert summary["termination_reason"] == "gradient_tolerance"
    assert summary["oracle_all_positive"]
    assert summary["fitted_oracle_relative_error"] < 1.0e-8
    assert (
        summary["fitted_train"]["rmse"]
        < 0.01 * summary["initial_train"]["rmse"]
    )
    assert (
        summary["fitted_holdout"]["rmse"]
        < 0.01 * summary["initial_holdout"]["rmse"]
    )
    assert summary["fitted_train"]["rmse"] < 5.0e-4
    assert summary["fitted_holdout"]["rmse"] < 5.0e-4


def test_demo_writes_reproducible_fit_artifacts(
    demo, problem, fit, tmp_path
):
    demo.save_results(problem, fit, tmp_path, make_plots=False)

    assert (tmp_path / "summary.json").is_file()
    assert (tmp_path / "optimization_history.csv").is_file()
    assert (tmp_path / "fitted_parameters.csv").is_file()
    predictions = np.load(tmp_path / "stress_predictions.npz")
    assert predictions["train_target"].shape == (60, 3, 3)
    assert predictions["train_predicted"].shape == (60, 3, 3)
    assert predictions["holdout_target"].shape == (49, 3, 3)
    assert predictions["holdout_predicted"].shape == (49, 3, 3)
