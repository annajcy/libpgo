"""Sparse linear-solver backends for the Newton step.

Each factory constructs a concrete selector object (a
``pypgo._core.PySparseSolver`` subclass) naming the backend used to solve the
Newton linear system, and returns it as a handle.  Pass the handle to
:class:`pypgo.solver.NewtonOptimizer` via ``sparse_solver=...``::

    ps.NewtonOptimizer(sparse_solver=ps.Auto())
    ps.NewtonOptimizer(sparse_solver=ps.EigenLDLT())
    ps.NewtonOptimizer(sparse_solver=ps.MKLPardiso())

The objects are opaque handles.  Backends carry no tunable parameters today;
the object form mirrors ``line_search`` and leaves room for per-backend options.
"""

from __future__ import annotations

import pypgo._core as _core

# Public base type — use for ``isinstance`` checks and type hints.
SparseSolver = _core.PySparseSolver


def Auto() -> SparseSolver:
    """Let the backend pick the best available sparse solver."""
    return _core.PyAutoSparseSolver()


def EigenLDLT() -> SparseSolver:
    """Eigen's SimplicialLDLT (always available)."""
    return _core.PyEigenLDLTSparseSolver()


def MKLPardiso() -> SparseSolver:
    """Intel MKL PARDISO."""
    return _core.PyMKLPardisoSparseSolver()


def OrigPardiso() -> SparseSolver:
    """The original (non-MKL) PARDISO backend."""
    return _core.PyOrigPardisoSparseSolver()
