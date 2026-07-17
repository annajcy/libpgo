import numpy as np
import pytest
import gc

import pypgo as pgo
import pypgo.solver as solver
from pypgo.parallel import ArenaThreadingExecutor
from pypgo.solver.result import NewtonConvergenceReason


def make_quadratic(dim=3):
    A = np.eye(dim, dtype=np.float64)
    b = np.zeros(dim, dtype=np.float64)
    if dim == 3:
        b[:] = [-1.0, 2.0, -4.0]
    return pgo.energy.QuadraticEnergy(A, b=b)


def make_problem(energy):
    return solver.OptimizationProblem(objective=energy)


def make_optimizer(**kwargs):
    return solver.NewtonOptimizer(damping=solver.NoDamping(), **kwargs)


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
    assert (
        result.diagnostics.newton_convergence_reason
        == NewtonConvergenceReason.ABSOLUTE_GRADIENT
    )
    assert result.diagnostics.newton_convergence_reason_name == "AbsoluteGradient"
    assert result.diagnostics.newton_convergence_threshold == pytest.approx(1e-6)


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

    for line_search in (
        solver.Golden(),
        solver.Brents(),
        solver.Backtrack(),
        solver.Simple(),
    ):
        result = solver.NewtonOptimizer(
            line_search=line_search, damping=solver.NoDamping()
        ).solve(problem, x0)
        assert result.converged


def test_newton_optimizer_sparse_solver_is_forwarded():
    """sparse_solver=EigenLDLT() selects the Eigen sparse LDLT backend."""
    import pypgo._core as _core

    energy = make_quadratic()
    problem = make_problem(energy)
    optimizer = solver.NewtonOptimizer(sparse_solver=solver.EigenLDLT())

    assert isinstance(optimizer._handle, _core.PyNewtonOptimizer)
    result = optimizer.solve(problem, np.zeros(3, dtype=np.float64))
    assert result.converged


def test_newton_threading_policy_routes_phases_and_owns_executors():
    energy = make_quadratic()
    problem = make_problem(energy)
    evaluation = ArenaThreadingExecutor(2, mkl_local_thread_budget=1)
    linear_solver = ArenaThreadingExecutor(3, mkl_local_thread_budget=3)
    threading = solver.NewtonThreadingPolicy(
        evaluation=evaluation,
        linear_solver=linear_solver,
    )
    optimizer = make_optimizer(threading=threading)

    del threading, evaluation, linear_solver
    gc.collect()

    result = optimizer.solve(problem, np.array([10.0, -3.0, 5.0], dtype=np.float64))
    again = optimizer.solve(problem, np.array([10.0, -3.0, 5.0], dtype=np.float64))

    assert result.converged
    assert again.converged
    assert result.diagnostics.threading_evaluation_phase_calls == 5
    assert result.diagnostics.threading_linear_solver_phase_calls == 3
    assert result.diagnostics.threading_evaluation_phase_seconds >= 0.0
    assert result.diagnostics.threading_linear_solver_phase_seconds >= 0.0
    assert result.diagnostics.newton_solver_setup_seconds is not None
    assert result.diagnostics.initial_symbolic_analyze_seconds is not None
    assert result.diagnostics.newton_solve_seconds is not None
    assert result.diagnostics.final_objective_seconds is not None
    assert result.diagnostics.linear_solver_cleanup_seconds is not None
    assert result.diagnostics.optimizer_total_seconds is not None


def test_newton_threading_policy_validates_executors_and_optimizer_argument():
    executor = ArenaThreadingExecutor(1)
    with pytest.raises(TypeError, match="evaluation"):
        solver.NewtonThreadingPolicy(evaluation=object(), linear_solver=executor)
    with pytest.raises(TypeError, match="linear_solver"):
        solver.NewtonThreadingPolicy(evaluation=executor, linear_solver=object())
    with pytest.raises(TypeError, match="threading"):
        make_optimizer(threading=object())
    policy = solver.NewtonThreadingPolicy(evaluation=executor, linear_solver=executor)
    with pytest.raises(AttributeError):
        policy.evaluation = executor


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
    optimizer = solver.NewtonOptimizer(max_iterations=0, damping=solver.NoDamping())

    result = optimizer.solve(problem, x0)

    assert result.status == solver.SolveStatus.MAX_ITERATIONS


def test_fix_variables_via_object_api():
    """Construct problem → fix variables → NewtonOptimizer → solve."""
    energy = make_quadratic()
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    problem = solver.OptimizationProblem(objective=energy)
    problem.fix_variables([2], [9.0], num_dofs=x0.size)
    result = solver.NewtonOptimizer(damping=solver.NoDamping()).solve(problem, x0)

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
    assert {
        name: int(getattr(solver.SolveStatus, name)) for name in expected
    } == expected


def test_final_gradient_stats_are_optional():
    energy = make_quadratic()
    problem = make_problem(energy)
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    result = solver.NewtonOptimizer(max_iterations=0, damping=solver.NoDamping()).solve(
        problem, x0
    )

    assert result.status == solver.SolveStatus.MAX_ITERATIONS
    assert result.final_gradient_norm is None
    assert result.final_gradient_max_norm is None
    assert result.diagnostics.newton_convergence_reason == NewtonConvergenceReason.NONE
    assert result.diagnostics.newton_convergence_reason_name == "None"
    assert result.diagnostics.newton_convergence_threshold is None


def test_invalid_fixed_values_length_raises():
    energy = make_quadratic()
    problem = make_problem(energy)

    with pytest.raises(ValueError):
        problem.fix_variables([0, 1], [1.0])


def test_invalid_line_search_raises():
    # A bare string is no longer accepted — line_search must be a LineSearch object.
    with pytest.raises(TypeError):
        solver.NewtonOptimizer(line_search="wolfe")


def test_invalid_line_search_params_raise():
    with pytest.raises(ValueError):
        solver.Backtrack(shrink=2.0)
    with pytest.raises(ValueError):
        solver.Simple(max_iterations=0)


def test_invalid_sparse_solver_raises():
    # A bare string is no longer accepted — sparse_solver must be a SparseSolver object.
    with pytest.raises(TypeError):
        solver.NewtonOptimizer(sparse_solver="not_a_solver")


def test_verbose_does_not_crash():
    energy = make_quadratic()
    problem = make_problem(energy)
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)

    result = solver.NewtonOptimizer(verbose=1, damping=solver.NoDamping()).solve(
        problem, x0
    )

    assert result.status in {
        solver.SolveStatus.CONVERGED,
        solver.SolveStatus.MAX_ITERATIONS,
        solver.SolveStatus.LINE_SEARCH_FAILED,
        solver.SolveStatus.STEP_TOO_SMALL,
    }


def test_optimizer_problem_peers_and_newton_solve():
    import pypgo._core as _core

    objective = pgo.energy.QuadraticEnergy(
        np.eye(2, dtype=np.float64), b=np.array([-4.0, 0.0])
    )
    problem = solver.OptimizationProblem(objective=objective)
    optimizer = solver.NewtonOptimizer(
        max_iterations=10, gradient_tolerance=1e-10, damping=solver.NoDamping()
    )

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
