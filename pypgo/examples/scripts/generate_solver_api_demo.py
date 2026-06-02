#!/usr/bin/env python3
"""Generate pypgo/examples/solver_api_demo.ipynb.

Run from the repository root:

    conda run -n libpgo python pypgo/examples/scripts/generate_solver_api_demo.py
"""

from __future__ import annotations

from notebook_builder import code, md, repo_root, write_notebook


CELLS = [
    md(
        """
        # pypgo.solver API Demo

        This tutorial demonstrates the Python Newton solver API:
        `pypgo.solver.solve_newton`, `SolveStatus`, `SolverResult`,
        `SolveDiagnostics`, and `NewtonOptions`.

        The Python API is intentionally small: users pass a `pypgo.energy`
        potential energy and an initial state `x0`; the solver returns a new
        solution array and diagnostics without mutating `x0`.
        """
    ),
    md(
        """
        ## Outline

        1. Imports and a small quadratic problem
        2. Basic Newton solve
        3. Result fields and diagnostics
        4. Input ownership: `x0` is not mutated
        5. Fixed DOFs with implicit and explicit values
        6. Line-search modes
        7. `NewtonOptions` as a reusable parameter object
        8. Solving a weighted `EnergySet`
        9. Constraint functions and soft penalties
        10. Common validation errors
        11. Public surface check
        """
    ),
    code(
        """
        import numpy as np
        import pypgo as pgo
        import pypgo.solver as solver
        """
    ),
    md(
        """
        ## 1. A small quadratic problem

        We solve:

        $$\\min_x \\; \\frac{1}{2} x^T A x + b^T x$$

        with `A = I` and `b = [-1, 2, -4]`.  The exact minimizer is
        $x^* = -b = [1, -2, 4]$.
        """
    ),
    code(
        """
        A = np.eye(3, dtype=np.float64)
        b = np.array([-1.0, 2.0, -4.0], dtype=np.float64)
        energy = pgo.energy.QuadraticEnergy(A, b=b)

        x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)
        exact = -b

        print("energy:", energy)
        print("x0:", x0)
        print("exact minimizer:", exact)
        """
    ),
    md(
        """
        ## 2. Basic Newton solve

        `solve_newton` returns a `SolverResult` object.  The default line search
        is `"backtrack"` and the default tolerance is `1e-6`.
        """
    ),
    code(
        """
        result = solver.solve_newton(
            energy,
            x0=x0,
            max_iter=50,
            tol=1e-8,
            damping=False,
            line_search="backtrack",
        )

        print("status:", result.status)
        print("converged:", result.converged)
        print("iterations:", result.iterations)
        print("x:", result.x)
        print("close to exact:", np.allclose(result.x, exact))
        """
    ),
    md(
        """
        ## 3. Result fields and diagnostics

        `SolverResult` separates solver-level status from optimization-level
        outputs such as the final objective and final solution vector.
        """
    ),
    code(
        """
        print("raw_status_code:", result.raw_status_code)
        print("final_objective:", result.final_objective)
        print("final_gradient_norm:", result.final_gradient_norm)
        print("final_gradient_max_norm:", result.final_gradient_max_norm)

        diag = result.diagnostics
        print("min_feasible_alpha:", diag.min_feasible_alpha)
        print("min_line_search_alpha:", diag.min_line_search_alpha)
        print("min_effective_alpha:", diag.min_effective_alpha)
        print("material clamps:", diag.material_clamp_count)
        print("contact clamps:", diag.contact_clamp_count)
        """
    ),
    md(
        """
        ## 4. Input ownership

        The solver treats `x0` as a read-only initial state.  It copies `x0`
        internally and returns an independently owned `result.x`.
        """
    ),
    code(
        """
        before = x0.copy()
        result = solver.solve_newton(energy, x0=x0, damping=False)

        print("x0 unchanged:", np.array_equal(x0, before))
        print("result.x shares memory with x0:", np.shares_memory(result.x, x0))

        result.x[0] = 123.0
        again = solver.solve_newton(energy, x0=x0, damping=False)
        print("mutating one result does not affect a fresh solve:", again.x)
        """
    ),
    md(
        """
        ## 5. Fixed DOFs

        `fixed_dofs` pins selected variables.  If `fixed_values=None`, each fixed
        value is taken from `x0[fixed_dofs]`.  DOFs may be unsorted; the service
        canonicalizes them before calling the native Newton service.
        """
    ),
    code(
        """
        x0_fixed = np.array([7.0, 10.0, -3.0], dtype=np.float64)

        implicit = solver.solve_newton(
            energy,
            x0=x0_fixed,
            fixed_dofs=[2, 0],
            fixed_values=None,
            damping=False,
        )

        print("implicit fixed values:", implicit.x)
        print("x[0] fixed to x0[0]:", implicit.x[0])
        print("x[2] fixed to x0[2]:", implicit.x[2])
        """
    ),
    code(
        """
        explicit = solver.solve_newton(
            energy,
            x0=x0,
            fixed_dofs=[2],
            fixed_values=np.array([9.0], dtype=np.float64),
            damping=False,
        )

        print("explicit fixed value:", explicit.x)
        """
    ),
    md(
        """
        ## 6. Line-search modes

        Python exposes four stable keywords:
        `"golden"`, `"brents"`, `"backtrack"`, and `"simple"`.
        """
    ),
    code(
        """
        for line_search in ("golden", "brents", "backtrack", "simple"):
            r = solver.solve_newton(
                energy,
                x0=x0,
                line_search=line_search,
                damping=False,
            )
            print(f"{line_search:10s}", r.status.name, r.iterations, r.x)
        """
    ),
    md(
        """
        ## 7. `NewtonOptions`

        `NewtonOptions` is a small frozen dataclass.  It is useful for keeping
        solver parameters near an experiment, and it can be passed directly to
        `solve_newton`.
        """
    ),
    code(
        """
        opts = solver.NewtonOptions(
            max_iter=20,
            tol=1e-8,
            damping=False,
            line_search="backtrack",
            verbose=0,
        )

        result = solver.solve_newton(
            energy,
            x0=x0,
            options=opts,
        )
        print(result)
        """
    ),
    md(
        """
        ## 8. Solving a weighted EnergySet

        `solve_newton` accepts any object with a `pypgo.energy.PotentialEnergy`
        handle, including `EnergySet`.  This example combines two quadratic
        terms with different weights.
        """
    ),
    code(
        """
        attraction = pgo.energy.QuadraticEnergy(
            np.eye(3, dtype=np.float64),
            b=np.array([-2.0, 0.0, 0.0], dtype=np.float64),
        )
        regularizer = pgo.energy.QuadraticEnergy(0.1 * np.eye(3, dtype=np.float64))

        total = pgo.energy.EnergySet([
            (attraction, 1.0),
            (regularizer, 1.0),
        ])

        r = solver.solve_newton(total, x0=total.zero_state(), damping=False)
        print("EnergySet solution:", r.x)
        print("final objective:", r.final_objective)
        """
    ),
    md(
        """
        ## 9. Constraint functions and soft penalties

        `pypgo.constraints` represents hard constraint functions such as
        $C(x) = A x + d$.  `Bounded` attaches solver-facing bounds:
        $\\ell \\le C(x) \\le u$.

        The Newton API in this demo still solves unconstrained or fixed-DOF
        problems only.  For Newton, use a soft penalty energy:

        - `ConstraintPenalty(c)` penalizes zero residual: $\\|C(x)\\|^2$.
        - `ConstraintViolationPenalty(Bounded(c, ...))` penalizes only bound
          violations.

        In formulas, a vector constraint function is:

        $$
        C(x) = A x + d.
        $$

        A hard bounded constraint descriptor represents:

        $$
        \\ell \\le C(x) \\le u.
        $$
        """
    ),
    code(
        """
        C = np.array([[1.0, 0.0, 0.0]], dtype=np.float64)
        linear_constraint = pgo.constraints.Linear(C, offset=np.array([-2.0], dtype=np.float64))

        x_probe = np.array([1.5, 0.0, 0.0], dtype=np.float64)
        print("C(x):", linear_constraint.value(x_probe))
        print("Jacobian:\\n", linear_constraint.jacobian(x_probe).to_dense())
        print("Hessian nnz:", linear_constraint.hessian(x_probe, np.ones(1)).nnz)

        hard_equality_descriptor = pgo.constraints.Bounded(linear_constraint, lower=0.0, upper=0.0)
        print("hard equality descriptor:", hard_equality_descriptor.lower, hard_equality_descriptor.upper)
        """
    ),
    md(
        """
        For the first example,

        $$
        C(x) = [1\\;0\\;0]x - 2 = x_0 - 2.
        $$

        `Bounded(linear_constraint, lower=0, upper=0)` describes the hard
        equality:

        $$
        x_0 - 2 = 0 \\quad \\Longleftrightarrow \\quad x_0 = 2.
        $$

        Since `solve_newton` does not consume hard constraints yet, the
        zero-residual penalty solves:

        $$
        \\min_x \\; \\frac{1}{2}\\|x\\|^2
        + \\frac{100}{2}(x_0 - 2)^2.
        $$
        """
    ),
    code(
        """
        zero_residual_penalty = pgo.energy.ConstraintPenalty(linear_constraint, weight=100.0)
        zero_residual_total = pgo.energy.EnergySet([
            (pgo.energy.QuadraticEnergy(np.eye(3, dtype=np.float64)), 1.0),
            (zero_residual_penalty, 1.0),
        ])

        zero_residual_result = solver.solve_newton(
            zero_residual_total,
            x0=np.zeros(3, dtype=np.float64),
            damping=False,
        )

        print("zero-residual penalty solution:", zero_residual_result.x)
        print("constraint residual:", linear_constraint.value(zero_residual_result.x))
        """
    ),
    md(
        """
        The second example uses bound-violation penalty.  For each constraint
        component, define:

        $$
        v_i(x) =
        \\begin{cases}
        C_i(x) - \\ell_i, & C_i(x) < \\ell_i,\\\\
        C_i(x) - u_i, & C_i(x) > u_i,\\\\
        0, & \\ell_i \\le C_i(x) \\le u_i.
        \\end{cases}
        $$

        Then:

        $$
        E_{\\text{viol}}(x) = \\frac{w}{2}\\sum_i v_i(x)^2.
        $$

        In the code below, $C(x)=x$, $w=100$, and the bounds encode
        $x_0 = 2$, $x_1 \\ge 0$, and $-1 \\le x_2 \\le 1$.  At
        $x = [1.5, -0.5, 0.25]$, only the first two components violate their
        bounds.
        """
    ),
    code(
        """
        # Bound-violation penalties consume Bounded descriptors.
        bound_constraint = pgo.constraints.Linear(
            np.eye(3, dtype=np.float64),
            offset=np.zeros(3, dtype=np.float64),
        )
        bounds = pgo.constraints.Bounded(
            bound_constraint,
            lower=np.array([2.0, 0.0, -1.0], dtype=np.float64),
            upper=np.array([2.0, np.inf, 1.0], dtype=np.float64),
        )
        violation_penalty = pgo.energy.ConstraintViolationPenalty(bounds, weight=100.0)

        x_probe = np.array([1.5, -0.5, 0.25], dtype=np.float64)
        print("probe value:", violation_penalty.value(x_probe))
        print("probe gradient:", violation_penalty.gradient(x_probe))
        print("probe Hessian:\\n", violation_penalty.hessian(x_probe).to_dense())
        """
    ),
    md(
        """
        To optimize with the same soft bounds, combine the violation penalty
        with any other potential energy.  Here the unconstrained attraction
        target is outside the bounds:

        $$
        E_{\\text{target}}(x)
        = \\frac{1}{2}\\|x - t\\|^2,
        \\quad t = [0, -1, 2.5].
        $$

        The total energy is:

        $$
        \\min_x \\;
        E_{\\text{target}}(x) + E_{\\text{viol}}(x).
        $$

        Because this is still a soft penalty, the solution is pulled close to
        the bounds but is not projected exactly onto the hard feasible set.
        """
    ),
    code(
        """
        target = np.array([0.0, -1.0, 2.5], dtype=np.float64)
        target_energy = pgo.energy.QuadraticEnergy(
            np.eye(3, dtype=np.float64),
            b=-target,
        )
        soft_bounded_total = pgo.energy.EnergySet([
            (target_energy, 1.0),
            (violation_penalty, 1.0),
        ])

        soft_bounded_result = solver.solve_newton(
            soft_bounded_total,
            x0=target.copy(),
            damping=False,
        )

        print("target:", target)
        print("soft-bounded solution:", soft_bounded_result.x)
        print("constraint value:", bound_constraint.value(soft_bounded_result.x))
        print("bound violation penalty:", violation_penalty.value(soft_bounded_result.x))
        print("total objective:", soft_bounded_result.final_objective)
        """
    ),
    md(
        """
        ## 10. Common validation errors

        Invalid line-search names and inconsistent fixed values raise
        `ValueError`.
        """
    ),
    code(
        """
        try:
            solver.solve_newton(energy, x0=x0, line_search="wolfe")
        except ValueError as exc:
            print("invalid line_search:", exc)

        try:
            solver.solve_newton(
                energy,
                x0=x0,
                fixed_dofs=[0, 1],
                fixed_values=np.array([1.0], dtype=np.float64),
            )
        except ValueError as exc:
            print("fixed_values mismatch:", exc)
        """
    ),
    md(
        """
        ## 11. Public surface

        The first Python release intentionally hides legacy C++ names such as
        `NewtonSolver`, `SolverParam`, and `EnergyOptimizer`.
        """
    ),
    code(
        """
        public_names = [name for name in dir(solver) if not name.startswith("_")]
        print(public_names)

        hidden = {"NewtonSolver", "SolverParam", "EnergyOptimizer", "minimize"}
        print("hidden legacy names present:", sorted(hidden.intersection(public_names)))
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "pypgo" / "examples" / "solver_api_demo.ipynb", CELLS)


if __name__ == "__main__":
    main()
