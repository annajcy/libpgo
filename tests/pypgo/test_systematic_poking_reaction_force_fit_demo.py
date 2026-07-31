"""Regression tests for equilibrium-based Systematic Poking calibration."""

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
    / "systematic_poking_fit_reaction_force"
)


def _load_demo():
    name = "systematic_poking_fit_reaction_force_demo"
    spec = importlib.util.spec_from_file_location(
        name, DEMO / "main.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def demo():
    return _load_demo()


@pytest.fixture(scope="module")
def small_problem(demo):
    return demo.build_problem(5, grid_size=2)


@pytest.fixture(scope="module")
def fitted_problem(demo):
    problem = demo.build_problem(9, grid_size=2)
    result = demo.fit_parameters(
        problem,
        max_iterations=15,
        smoothness_weight=1.0e-3,
    )
    return problem, result


def test_load_protocols_leave_true_equilibrium_dofs(
    demo, small_problem
):
    assert {
        case.protocol for case in small_problem.train_cases
    } == {
        "free_uniaxial",
        "confined_uniaxial",
        "simple_shear",
    }
    assert all(
        0 < case.free_dofs.size
        < small_problem.systematic_energy.num_dofs
        for case in small_problem.train_cases
    )
    assert len(small_problem.train_cases) == 12
    assert len(small_problem.validation_cases) == 12
    assert len(small_problem.holdout_cases) == 24


def test_free_uniaxial_uses_only_normal_constraints_and_rigid_pins(
    demo,
):
    vertices, _ = demo.make_cubic_grid(2)
    stretch = 0.75
    case = demo._uniaxial_case(
        vertices,
        stretch,
        confined=False,
        split="test",
    )
    bottom = np.flatnonzero(np.isclose(vertices[:, 1], 0.0))
    top = np.flatnonzero(np.isclose(vertices[:, 1], 1.0))

    assert case.fixed_dofs.size == 21
    assert case.free_dofs.size == 60
    assert set(3 * bottom + 1) <= set(case.fixed_dofs)
    assert set(3 * top + 1) <= set(case.fixed_dofs)
    tangent_dofs = case.fixed_dofs[case.fixed_dofs % 3 != 1]
    assert tangent_dofs.size == 3

    anchor, _ = demo._free_uniaxial_rigid_pins(vertices, bottom)
    lateral_stretch = 1.2
    displacement = np.zeros_like(vertices)
    displacement[:, 0] = (
        (lateral_stretch - 1.0)
        * (vertices[:, 0] - vertices[anchor, 0])
    )
    displacement[:, 1] = (stretch - 1.0) * vertices[:, 1]
    displacement[:, 2] = (
        (lateral_stretch - 1.0)
        * (vertices[:, 2] - vertices[anchor, 2])
    )
    np.testing.assert_allclose(
        displacement.ravel()[case.fixed_dofs],
        case.fixed_values,
    )


def test_confined_uniaxial_fixes_every_lateral_dof(demo):
    vertices, _ = demo.make_cubic_grid(2)
    case = demo._uniaxial_case(
        vertices,
        0.75,
        confined=True,
        split="test",
    )

    assert case.fixed_dofs.size == 72
    assert case.free_dofs.size == 9
    assert set(3 * np.arange(len(vertices))) <= set(case.fixed_dofs)
    assert set(3 * np.arange(len(vertices)) + 2) <= set(case.fixed_dofs)


def test_equilibrium_reaction_jacobian_matches_full_resolve_fd(
    demo, small_problem
):
    parameters = demo.initial_material_parameters(
        small_problem.stretch_knots)
    theta = np.log(parameters / demo.FORCE_SCALE)
    weight = 3.0e-3
    small_problem.reset_warm_starts()
    _, analytic, states = demo.reaction_residual_and_jacobian(
        small_problem,
        theta,
        "train",
        smoothness_weight=weight,
    )
    finite_difference = np.empty_like(analytic)
    step = 2.0e-6

    for parameter_index in range(theta.size):
        direction = np.zeros_like(theta)
        direction[parameter_index] = step
        small_problem.reset_warm_starts()
        residual_plus, _, _ = demo.reaction_residual_and_jacobian(
            small_problem,
            theta + direction,
            "train",
            smoothness_weight=weight,
        )
        small_problem.reset_warm_starts()
        residual_minus, _, _ = demo.reaction_residual_and_jacobian(
            small_problem,
            theta - direction,
            "train",
            smoothness_weight=weight,
        )
        finite_difference[:, parameter_index] = (
            residual_plus - residual_minus) / (2.0 * step)

    assert max(
        state.free_residual_max for state in states
    ) <= demo.STATIC_RESIDUAL_TOLERANCE
    np.testing.assert_allclose(
        analytic,
        finite_difference,
        rtol=5.0e-7,
        atol=2.0e-9,
    )


def test_fit_is_full_rank_positive_and_generalizes(
    demo, fitted_problem
):
    _, result = fitted_problem
    summary = result.summary

    assert summary["termination_reason"] == "gradient_tolerance"
    assert summary["data_jacobian_rank"] == 10
    assert summary["data_jacobian_condition"] < 2.0e2
    assert np.all(result.fitted_parameters > 0.0)
    for split in ("train", "validation", "holdout"):
        assert (
            summary[f"fitted_{split}"]["rmse"]
            < 0.01 * summary[f"initial_{split}"]["rmse"]
        )
        assert (
            summary[f"fitted_{split}"]["max_free_residual"]
            <= demo.STATIC_RESIDUAL_TOLERANCE
        )


def test_demo_writes_reaction_fit_artifacts(
    demo, fitted_problem, tmp_path
):
    problem, result = fitted_problem
    demo.save_results(
        problem, result, tmp_path, make_plots=False)

    assert (tmp_path / "summary.json").is_file()
    assert (tmp_path / "optimization_history.csv").is_file()
    assert (tmp_path / "fitted_parameters.csv").is_file()
    predictions = np.load(
        tmp_path / "reaction_predictions.npz")
    assert predictions["train_target"].shape == (20,)
    assert predictions["train_predicted"].shape == (20,)
    assert predictions["validation_target"].shape == (20,)
    assert predictions["validation_predicted"].shape == (20,)
    assert predictions["holdout_target"].shape == (40,)
    assert predictions["holdout_predicted"].shape == (40,)
