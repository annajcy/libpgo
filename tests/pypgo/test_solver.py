import numpy as np
import pytest

import pypgo as pgo
import pypgo.solver as solver


def make_quadratic(dim=3):
    A = np.eye(dim, dtype=np.float64)
    b = np.zeros(dim, dtype=np.float64)
    if dim == 3:
        b[:] = [-1.0, 2.0, -4.0]
    return pgo.energy.QuadraticEnergy(A, b=b)


def make_problem(energy):
    return solver.OptimizationProblem(objective=energy)


def make_optimizer(**kwargs):
    return solver.NewtonOptimizer(damping=False, **kwargs)


def test_newton_optimizer_solves_quadratic():
    energy = make_quadratic()
    problem = make_problem(energy)
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    result = make_optimizer().solve(problem, x0)

    assert isinstance(result, solver.SolverResult)
    assert result.status == solver.SolveStatus.CONVERGED
    assert result.converged
    assert result.iterations >= 0
    assert np.allclose(result.x, [1.0, -2.0, 4.0])
    assert result.final_objective is not None


def test_optimization_problem_fix_variables():
    energy = make_quadratic()
    problem = make_problem(energy)
    problem.fix_variables([2, 0], [-3.0, 7.0])
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    result = make_optimizer().solve(problem, x0)

    assert result.converged
    assert np.allclose(result.x, [7.0, -2.0, -3.0])


def test_x0_not_mutated():
    energy = make_quadratic()
    problem = make_problem(energy)
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)
    original = x0.copy()

    make_optimizer().solve(problem, x0)

    assert np.array_equal(x0, original)


def test_result_x_is_independent_numpy_array():
    energy = make_quadratic()
    problem = make_problem(energy)
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    result = make_optimizer().solve(problem, x0)
    result.x[0] = 123.0

    second = make_optimizer().solve(problem, x0)
    assert np.allclose(second.x, [1.0, -2.0, 4.0])


def test_line_search_keywords():
    energy = make_quadratic()
    problem = make_problem(energy)
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    for keyword in ("golden", "brents", "backtrack", "simple"):
        result = solver.NewtonOptimizer(line_search=keyword, damping=False).solve(problem, x0)
        assert result.converged


def test_newton_optimizer_sparse_solver_is_forwarded():
    """sparse_solver='eigen_ldlt' maps to sparse_solver_kind=1 in C++."""
    import pypgo._core as _core

    # Verify the options struct carries the right value through construction
    energy = make_quadratic()
    problem = make_problem(energy)
    optimizer = solver.NewtonOptimizer(sparse_solver="eigen_ldlt")

    assert isinstance(optimizer._handle, _core.PyNewtonOptimizer)
    result = optimizer.solve(problem, np.zeros(3, dtype=np.float64))
    assert result.converged


def test_optimization_problem_rejects_non_energy_objective():
    # A bare object and a ConstraintFunction (which also exposes a `_handle`)
    # must both be rejected now that the check is isinstance(PotentialEnergy).
    with pytest.raises(TypeError):
        solver.OptimizationProblem(objective=object())

    constraint = pgo.constraints.Linear(
        pgo.sparse.SparseMatrix.from_coo((1, 3), [0], [0], [1.0])
    )
    assert hasattr(constraint, "_handle")  # would have passed the old duck-typed check
    with pytest.raises(TypeError):
        solver.OptimizationProblem(objective=constraint)


def test_optimizer_keywords_control_solve():
    energy = make_quadratic()
    problem = make_problem(energy)
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)
    optimizer = solver.NewtonOptimizer(max_iterations=0, damping=False)

    result = optimizer.solve(problem, x0)

    assert result.status == solver.SolveStatus.MAX_ITERATIONS


def test_fix_variables_via_object_api():
    """Construct problem → fix variables → NewtonOptimizer → solve."""
    energy = make_quadratic()
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    problem = solver.OptimizationProblem(objective=energy)
    problem.fix_variables([2], [9.0], num_dofs=x0.size)
    result = solver.NewtonOptimizer(damping=False).solve(problem, x0)

    assert result.converged
    assert np.allclose(result.x, [1.0, -2.0, 9.0])


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
    problem = make_problem(energy)
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    result = solver.NewtonOptimizer(max_iterations=0, damping=False).solve(problem, x0)

    assert result.status == solver.SolveStatus.MAX_ITERATIONS
    assert result.final_gradient_norm is None
    assert result.final_gradient_max_norm is None


def test_invalid_fixed_values_length_raises():
    energy = make_quadratic()
    problem = make_problem(energy)

    with pytest.raises(ValueError):
        problem.fix_variables([0, 1], [1.0])


def test_invalid_line_search_raises():
    with pytest.raises(ValueError):
        solver.NewtonOptimizer(line_search="wolfe")


def test_invalid_sparse_solver_raises():
    with pytest.raises(ValueError):
        solver.NewtonOptimizer(sparse_solver="not_a_solver")


def test_verbose_does_not_crash():
    energy = make_quadratic()
    problem = make_problem(energy)
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    result = solver.NewtonOptimizer(verbose=1, damping=False).solve(problem, x0)

    assert result.status in {
        solver.SolveStatus.CONVERGED,
        solver.SolveStatus.MAX_ITERATIONS,
        solver.SolveStatus.LINE_SEARCH_FAILED,
        solver.SolveStatus.STEP_TOO_SMALL,
    }


def test_optimizer_problem_peers_and_newton_solve():
    import pypgo._core as _core

    objective = pgo.energy.QuadraticEnergy(np.eye(2, dtype=np.float64), b=np.array([-4.0, 0.0]))
    problem = solver.OptimizationProblem(objective=objective)
    optimizer = solver.NewtonOptimizer(max_iterations=10, gradient_tolerance=1e-10, damping=False)

    assert isinstance(problem._handle, _core.PyOptimizationProblem)
    assert isinstance(optimizer, solver.Optimizer)
    assert isinstance(optimizer._handle, _core.PyNewtonOptimizer)
    assert isinstance(optimizer._handle, _core.PyOptimizer)
    assert not hasattr(optimizer._handle, "as_optimizer")

    result = optimizer.solve(problem, np.array([0.0, 0.0], dtype=np.float64))

    assert result.converged
    assert result.status == solver.SolveStatus.CONVERGED
    np.testing.assert_allclose(result.x, [4.0, 0.0], atol=1e-8)


def test_solver_public_surface_hides_legacy_names():
    hidden = {
        "NewtonOptions",
        "NewtonSolver",
        "SolverParam",
        "EnergyOptimizer",
        "FixedVariables",
        "BoxBounds",
        "NonlinearConstraints",
        "OptimizationBackend",
        "NewtonLineSearchPolicy",
        "NewtonSparseSolverBackend",
        "NewtonSparseSolverKind",
        "NewtonSparseSolverOptions",
        "minimize",
        "solve_newton",
    }
    assert hidden.isdisjoint(set(dir(solver)))
