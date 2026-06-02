import numpy as np
import pytest

import pypgo as pgo
import pypgo.solver as solver


def make_quadratic():
    A = np.eye(3, dtype=np.float64)
    b = np.array([-1.0, 2.0, -4.0], dtype=np.float64)
    return pgo.energy.QuadraticEnergy(A, b=b)


def test_newton_solves_quadratic__warm_start():
    energy = make_quadratic()
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    result = solver.solve_newton(energy, x0=x0, damping=False)

    assert isinstance(result, solver.SolverResult)
    assert result.status == solver.SolveStatus.CONVERGED
    assert result.converged
    assert result.iterations >= 0
    assert np.allclose(result.x, [1.0, -2.0, 4.0])
    assert result.final_objective is not None


def test_newton_solves_quadratic__implicit_fixed_values_from_x0():
    energy = make_quadratic()
    x0 = np.array([7.0, 10.0, -3.0], dtype=np.float64)

    result = solver.solve_newton(
        energy,
        x0=x0,
        fixed_dofs=[2, 0],
        fixed_values=None,
        damping=False,
    )

    assert result.converged
    assert np.allclose(result.x, [7.0, -2.0, -3.0])


def test_newton_solves_quadratic__explicit_fixed_values():
    energy = make_quadratic()
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    result = solver.solve_newton(
        energy,
        x0=x0,
        fixed_dofs=[2],
        fixed_values=np.array([9.0], dtype=np.float64),
        damping=False,
    )

    assert result.converged
    assert np.allclose(result.x, [1.0, -2.0, 9.0])


def test_x0_not_mutated():
    energy = make_quadratic()
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)
    original = x0.copy()

    solver.solve_newton(energy, x0=x0, damping=False)

    assert np.array_equal(x0, original)


def test_result_x_is_independent_numpy_array():
    energy = make_quadratic()
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    result = solver.solve_newton(energy, x0=x0, damping=False)
    result.x[0] = 123.0

    second = solver.solve_newton(energy, x0=x0, damping=False)
    assert np.allclose(second.x, [1.0, -2.0, 4.0])


def test_line_search_keywords():
    energy = make_quadratic()
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    for keyword in ("golden", "brents", "backtrack", "simple"):
        result = solver.solve_newton(energy, x0=x0, line_search=keyword, damping=False)
        assert result.converged


def test_newton_options_can_be_passed_directly():
    energy = make_quadratic()
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)
    options = solver.NewtonOptions(
        max_iter=20,
        tol=1e-8,
        damping=False,
        line_search="backtrack",
        verbose=0,
    )

    result = solver.solve_newton(energy, x0=x0, options=options)

    assert result.converged
    assert np.allclose(result.x, [1.0, -2.0, 4.0])


def test_explicit_solver_keywords_override_newton_options():
    energy = make_quadratic()
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)
    options = solver.NewtonOptions(max_iter=20, damping=False)

    result = solver.solve_newton(energy, x0=x0, options=options, max_iter=0)

    assert result.status == solver.SolveStatus.MAX_ITERATIONS


def test_invalid_newton_options_type_raises():
    energy = make_quadratic()
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    with pytest.raises(TypeError, match="NewtonOptions"):
        solver.solve_newton(energy, x0=x0, options={"max_iter": 20})


def test_status_roundtrip_for_all_values():
    expected = {
        "CONVERGED": 0,
        "MAX_ITERATIONS": 1,
        "LINE_SEARCH_FAILED": 2,
        "STEP_TOO_SMALL": 3,
        "NON_FINITE": 4,
        "LINEAR_SOLVE_FAILED": 5,
        "EXTERNAL_SOLVER_FAILURE": 100,
        "UNSUPPORTED_BACKEND": 101,
    }
    assert {name: int(getattr(solver.SolveStatus, name)) for name in expected} == expected


def test_final_gradient_stats_are_optional():
    energy = make_quadratic()
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    result = solver.solve_newton(energy, x0=x0, max_iter=0, damping=False)

    assert result.status == solver.SolveStatus.MAX_ITERATIONS
    assert result.final_gradient_norm is None
    assert result.final_gradient_max_norm is None


def test_invalid_fixed_values_length_raises():
    energy = make_quadratic()
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    with pytest.raises(ValueError):
        solver.solve_newton(
            energy,
            x0=x0,
            fixed_dofs=[0, 1],
            fixed_values=np.array([1.0], dtype=np.float64),
        )


def test_invalid_line_search_raises():
    energy = make_quadratic()
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    with pytest.raises(ValueError):
        solver.solve_newton(energy, x0=x0, line_search="wolfe")


def test_verbose_does_not_crash():
    energy = make_quadratic()
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    result = solver.solve_newton(energy, x0=x0, verbose=1, damping=False)

    assert result.status in {
        solver.SolveStatus.CONVERGED,
        solver.SolveStatus.MAX_ITERATIONS,
        solver.SolveStatus.LINE_SEARCH_FAILED,
        solver.SolveStatus.STEP_TOO_SMALL,
    }


def test_solver_public_surface_hides_legacy_names():
    hidden = {
        "NewtonSolver",
        "SolverParam",
        "EnergyOptimizer",
        "OptimizationProblem",
        "FixedVariables",
        "BoxBounds",
        "NonlinearConstraints",
        "OptimizationBackend",
        "NewtonLineSearchPolicy",
        "NewtonSparseSolverBackend",
        "NewtonSparseSolverKind",
        "NewtonSparseSolverOptions",
        "minimize",
    }
    assert hidden.isdisjoint(set(dir(solver)))
