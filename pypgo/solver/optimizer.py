"""Concrete optimizer implementations.

This is the side of the solver package expected to grow: each optimizer keeps
its own option mapping and validation here, so adding a new one (LBFGS,
trust-region, ...) is a local change that never touches the result/problem
data layer.
"""

from __future__ import annotations

import pypgo._core as _core
from pypgo.solver.base import Optimizer

_SPARSE_SOLVERS = {
    "auto": 0,
    "eigen_ldlt": 1,
    "pardiso": 2,
    "orig_pardiso": 3,
}

_LINE_SEARCH_METHODS = {"golden", "brents", "backtrack", "simple"}


class NewtonOptimizer(Optimizer):
    """Newton optimizer with backend-specific options.

    Parameters are validated at construction time and used to build the
    C++ peer once.  The peer is immutable after construction; to change
    options, create a new ``NewtonOptimizer``.
    """

    def __init__(self, *, max_iterations=50, gradient_tolerance=1e-6,
                 damping=True, line_search="backtrack", verbose=0,
                 sparse_solver="auto"):
        if sparse_solver not in _SPARSE_SOLVERS:
            raise ValueError(f"sparse_solver must be one of {list(_SPARSE_SOLVERS)}, got {sparse_solver!r}")
        if line_search not in _LINE_SEARCH_METHODS:
            raise ValueError(f"line_search must be one of {sorted(_LINE_SEARCH_METHODS)}, got {line_search!r}")

        options = _core.PyNewtonOptimizerOptions()
        options.max_iterations = int(max_iterations)
        options.gradient_tolerance = float(gradient_tolerance)
        options.damping = bool(damping)
        options.line_search = str(line_search)
        options.verbose = int(verbose)
        options.sparse_solver_kind = _SPARSE_SOLVERS[sparse_solver]
        super().__init__(_core.PyNewtonOptimizer(options))
