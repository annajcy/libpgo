"""Newton diagonal damping policies.

Each factory constructs a concrete C++ damping policy object (a
``pypgo._core.PyDampingPolicy`` subclass) carrying its own parameters,
and returns it as a handle.  Pass the handle to
:class:`pypgo.solver.NewtonOptimizer` via ``damping=...``::

    ps.NewtonOptimizer(damping=ps.FixedDamping(damping_scale=2.0))
    ps.NewtonOptimizer(damping=ps.NoDamping())

The returned objects are opaque, immutable handles; build a new one to change
parameters.
"""

from __future__ import annotations

import pypgo._core as _core

# Public base type — use for ``isinstance`` checks and type hints.
Damping = _core.PyDampingPolicy


def NoDamping() -> Damping:
    """No diagonal damping.  The Newton system is solved as-is."""
    return _core.PyNoDamping()


def FixedDamping(damping_scale: float = 1.0) -> Damping:
    """Fixed diagonal damping.

    Adds ``dampingScale * lambdaScale * lambda0`` to the diagonal of the
    Newton system, where ``lambdaScale`` decays per-iteration and
    ``lambda0`` is the initial gradient max-norm.
    """
    if damping_scale <= 0.0:
        raise ValueError(f"damping_scale must be positive, got {damping_scale}")
    return _core.PyFixedDamping(float(damping_scale))
