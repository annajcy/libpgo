"""Concrete optimizer implementations.

This is the side of the solver package expected to grow: each optimizer keeps
its own option mapping and validation here, so adding a new one (LBFGS,
trust-region, ...) is a local change that never touches the result/problem
data layer.  When this file gets heavy, promote it to an ``optimizer/``
subpackage (one module per optimizer).
"""

from __future__ import annotations

import pypgo._core as _core
from pypgo.solver.base import Optimizer
from pypgo.solver.damping import Damping, NoDamping
from pypgo.solver.line_search import Backtrack, LineSearch
from pypgo.solver.sparse_solver import Auto, SparseSolver
from pypgo.solver.termination import FixedTermination, Termination


class NewtonOptimizer(Optimizer):
    """Newton optimizer with backend-specific options.

    Parameters are validated at construction time and used to build the
    C++ peer once.  The peer is immutable after construction; to change
    options, create a new ``NewtonOptimizer``.

    ``line_search`` takes a :class:`pypgo.solver.LineSearch` instance — e.g.
    ``ps.Backtrack(armijo_c=1e-4)``, ``ps.Simple(max_iterations=50)``,
    ``ps.Golden()`` or ``ps.Brents()`` — which binds the concrete C++ policy
    and its parameters.  Defaults to ``Backtrack()``.

    ``damping`` takes a :class:`pypgo.solver.Damping` instance — e.g.
    ``ps.FixedDamping(damping_scale=2.0)`` or ``ps.NoDamping()``.
    Defaults to ``NoDamping()``.

    ``termination`` takes a :class:`pypgo.solver.Termination` instance —
    e.g. ``ps.DefaultTermination()``.  Defaults to ``DefaultTermination()``.

    ``sparse_solver`` takes a :class:`pypgo.solver.SparseSolver` instance —
    ``ps.Auto()``, ``ps.EigenLDLT()``, ``ps.MKLPardiso()`` or
    ``ps.OrigPardiso()`` — selecting the Newton step's linear solver.
    Defaults to ``Auto()``.
    """

    def __init__(self, *, max_iterations=50, gradient_tolerance=1e-6,
                 line_search=None, damping=None, termination=None,
                 verbose=0, sparse_solver=None):
        if line_search is None:
            line_search = Backtrack()
        if not isinstance(line_search, LineSearch):
            raise TypeError(
                "line_search must be a pypgo.solver.LineSearch policy "
                "(e.g. ps.Backtrack(), ps.Golden(), ps.Brents(), ps.Simple()), "
                f"got {type(line_search).__name__}")
        if damping is None:
            damping = NoDamping()
        if not isinstance(damping, Damping):
            raise TypeError(
                "damping must be a pypgo.solver.Damping policy "
                "(e.g. ps.FixedDamping(), ps.NoDamping()), "
                f"got {type(damping).__name__}")
        if termination is None:
            termination = FixedTermination()
        if not isinstance(termination, Termination):
            raise TypeError(
                "termination must be a pypgo.solver.Termination policy "
                "(e.g. ps.DefaultTermination()), "
                f"got {type(termination).__name__}")
        if sparse_solver is None:
            sparse_solver = Auto()
        if not isinstance(sparse_solver, SparseSolver):
            raise TypeError(
                "sparse_solver must be a pypgo.solver.SparseSolver "
                "(e.g. ps.Auto(), ps.EigenLDLT(), ps.MKLPardiso(), ps.OrigPardiso()), "
                f"got {type(sparse_solver).__name__}")

        options = _core.PyNewtonOptimizerOptions()
        options.max_iterations = int(max_iterations)
        options.gradient_tolerance = float(gradient_tolerance)
        options.line_search = line_search
        options.damping = damping
        options.termination = termination
        options.sparse_solver = sparse_solver
        options.verbose = int(verbose)
        super().__init__(_core.PyNewtonOptimizer(options))
