"""Newton line-search policies.

Each factory constructs a concrete C++ line-search policy object (a
``pypgo._core.PyLineSearchPolicy`` subclass) carrying that method's own
parameters, and returns it as a handle.  Pass the handle to
:class:`pypgo.solver.NewtonOptimizer` via ``line_search=...``::

    ps.NewtonOptimizer(line_search=ps.Backtrack(armijo_c=1e-4, shrink=0.5))
    ps.NewtonOptimizer(line_search=ps.Simple(max_iterations=50))
    ps.NewtonOptimizer(line_search=ps.Golden())

The returned objects are opaque, immutable handles; build a new one to change
parameters.
"""

from __future__ import annotations

import pypgo._core as _core

# Public base type — use for ``isinstance`` checks and type hints.
LineSearch = _core.PyLineSearchPolicy


def Golden() -> LineSearch:
    """Golden-section search. Brackets a minimum along the Newton direction."""
    return _core.PyGoldenLineSearch()


def Brents() -> LineSearch:
    """Brent's method (parabolic interpolation + golden-section fallback)."""
    return _core.PyBrentsLineSearch()


def Backtrack(armijo_c: float = 1e-4, shrink: float = 0.5,
              initial_alpha: float = 1.0) -> LineSearch:
    """Backtracking line search with the Armijo sufficient-decrease condition.

    Shrinks the trial step by ``shrink`` from ``initial_alpha`` until
    ``f(x + a p) <= f(x) + armijo_c * a * g^T p``.
    """
    if not 0.0 < armijo_c < 1.0:
        raise ValueError(f"armijo_c must be in (0, 1), got {armijo_c}")
    if not 0.0 < shrink < 1.0:
        raise ValueError(f"shrink must be in (0, 1), got {shrink}")
    if not initial_alpha > 0.0:
        raise ValueError(f"initial_alpha must be > 0, got {initial_alpha}")
    return _core.PyBacktrackLineSearch(float(armijo_c), float(shrink), float(initial_alpha))


def Simple(max_iterations: int = 100, shrink: float = 0.5) -> LineSearch:
    """Simple line search: shrink the step by ``shrink`` until the energy drops."""
    if max_iterations < 1:
        raise ValueError(f"max_iterations must be >= 1, got {max_iterations}")
    if not 0.0 < shrink < 1.0:
        raise ValueError(f"shrink must be in (0, 1), got {shrink}")
    return _core.PySimpleLineSearch(int(max_iterations), float(shrink))
