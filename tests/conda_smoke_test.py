"""Out-of-box smoke test for an installed pypgo wheel.

Designed to run in a *fresh* environment, with
nothing on the path but the published package and its runtime dependencies. It
verifies that:

  1. the native extension loads (all runtime libs — BLAS, geogram, gmp, ... —
     resolve at import time);
  2. ``build_info()`` reports its solver-backend capabilities;
  3. a real Newton solve runs end-to-end with the default and Eigen backends;
  4. the MKL Pardiso backend is *functional* when the package advertises it,
     and is correctly *absent* otherwise.

The script self-adapts to platform capabilities via ``build_info()``. Depends
only on numpy.

Run manually inside an environment with pypgo installed:

    python tests/conda_smoke_test.py
"""

import numpy as np

import pypgo as pgo
import pypgo.solver as ps
import pypgo._core as _core


def _make_problem():
    """Quadratic E(x) = 1/2 x^T A x + b^T x with A = I.

    Gradient is x + b, so the unique minimizer is x* = -b. Newton converges in a
    single step, exercising exactly one linear solve through the chosen backend.
    """
    b = np.array([-1.0, 2.0, -4.0], dtype=np.float64)
    energy = pgo.energy.QuadraticEnergy(np.eye(3, dtype=np.float64), b=b)
    problem = ps.OptimizationProblem(objective=energy)
    return problem, -b


def _solve_with(backend):
    problem, expected = _make_problem()
    x0 = np.array([10.0, -3.0, 5.0], dtype=np.float64)
    result = ps.NewtonOptimizer(damping=ps.NoDamping(), sparse_solver=backend).solve(problem, x0)
    return result, expected


def _assert_solved(backend, label):
    result, expected = _solve_with(backend)
    assert result.converged, f"{label} did not converge (status={result.status})"
    assert np.allclose(result.x, expected), f"{label} wrong solution: {result.x} != {expected}"


def main():
    info = _core.build_info()
    print("build_info:", info)

    backends = set(info.get("solver_backends", []))

    # 1) Out-of-box: the default (Auto) and Eigen LDLT backends must solve on
    #    every platform/flavor. This is the core "the package actually works"
    #    check — a broken native link surfaces as an import or solve failure.
    _assert_solved(ps.Auto(), "Auto backend")
    _assert_solved(ps.EigenLDLT(), "EigenLDLT backend")
    print("OK: default + EigenLDLT solve converged to the expected minimizer.")

    # 2) MKL Pardiso, self-adapting on the advertised capability.
    if "mkl_pardiso" in backends:
        assert info.get("mkl") is True, "mkl_pardiso advertised but build_info()['mkl'] is False"
        _assert_solved(ps.MKLPardiso(), "MKLPardiso backend")
        print("OK: MKL Pardiso Newton backend is functional.")
    else:
        # Non-MKL build: asking for MKL Pardiso must not silently succeed. The
        # backend factory raises in builds without MKL; accept either a clean
        # exception or a non-converged result, but never a successful MKL solve.
        assert info.get("mkl") is False, "build_info()['mkl'] is True but mkl_pardiso is unavailable"
        rejected = False
        try:
            result, _ = _solve_with(ps.MKLPardiso())
            rejected = not result.converged
        except Exception as exc:  # noqa: BLE001 - any failure means "correctly unavailable"
            rejected = True
            print(f"OK: MKL Pardiso correctly rejected: {type(exc).__name__}: {exc}")
        assert rejected, "non-MKL build unexpectedly solved with the MKL Pardiso backend"
        print("OK: MKL Pardiso backend is correctly unavailable in this non-MKL build.")

    print("pypgo conda smoke test PASSED.")


if __name__ == "__main__":
    main()
