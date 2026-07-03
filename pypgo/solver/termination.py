"""Newton termination policies.

Each factory constructs a concrete C++ termination policy object (a
``pypgo._core.PyTerminationPolicy`` subclass) and returns it as a handle.
Pass the handle to :class:`pypgo.solver.NewtonOptimizer` via
``termination=...``::

    ps.NewtonOptimizer(termination=ps.FixedTermination())

The returned objects are opaque, immutable handles.
"""

from __future__ import annotations

import pypgo._core as _core

# Public base type — use for ``isinstance`` checks and type hints.
Termination = _core.PyTerminationPolicy


def FixedTermination() -> Termination:
    """``FixedNewtonTerminationPolicy`` — fixed (non-adaptive) termination.

    Converges when the gradient max-norm drops below ``gradient_tolerance``
    (absolute) or ``lambda0 * 1e-5`` (relative to initial gradient).
    Never stops before the Newton loop exhausts unless one of those two
    conditions is met.
    """
    return _core.PyFixedTermination()
