"""Newton termination policies.

Each factory constructs a concrete C++ termination policy object (a
``pypgo._core.PyTerminationPolicy`` subclass) and returns it as a handle.
Pass the handle to :class:`pypgo.solver.NewtonOptimizer` via
``termination=...``::

    ps.NewtonOptimizer(termination=ps.AbsoluteTermination(abs_tolerance=1e-6))
    ps.NewtonOptimizer(termination=ps.RelativeTermination(relative_tolerance=1e-5))
    ps.NewtonOptimizer(termination=ps.HybridTermination(
        abs_tolerance=1e-6, relative_tolerance=1e-5))

The returned objects are opaque, immutable handles.
"""

from __future__ import annotations

import pypgo._core as _core

# Public base type — use for ``isinstance`` checks and type hints.
Termination = _core.PyTerminationPolicy


def HybridTermination(
    abs_tolerance: float, relative_tolerance: float,
) -> Termination:
    """``HybridNewtonTerminationPolicy`` — absolute-or-relative termination.

    Converges when the gradient max-norm drops below ``abs_tolerance``
    (absolute) or ``lambda0 * relative_tolerance`` (relative to the initial
    gradient). Because of the relative shortcut, a solve with a large
    initial gradient can stop earlier than ``abs_tolerance``; use
    :func:`AbsoluteTermination` to guarantee an absolute residual.
    """
    if abs_tolerance <= 0.0:
        raise ValueError(
            f"abs_tolerance must be positive, got {abs_tolerance}")
    if relative_tolerance <= 0.0:
        raise ValueError(
            f"relative_tolerance must be positive, got {relative_tolerance}")
    return _core.PyHybridTermination(
        abs_tolerance, float(relative_tolerance))


def AbsoluteTermination(abs_tolerance: float) -> Termination:
    """``AbsoluteNewtonTerminationPolicy`` — absolute-only termination.

    Converges only when the gradient max-norm drops below ``abs_tolerance``.
    Unlike :func:`HybridTermination`, there is no relative shortcut, so a
    solve cannot stop early just because the initial gradient was large.
    """
    if abs_tolerance <= 0.0:
        raise ValueError(
            f"abs_tolerance must be positive, got {abs_tolerance}")
    return _core.PyAbsoluteTermination(abs_tolerance)


def RelativeTermination(relative_tolerance: float) -> Termination:
    """``RelativeNewtonTerminationPolicy`` — relative-only termination.

    Converges only when the gradient max-norm drops below
    ``lambda0 * relative_tolerance``, where ``lambda0`` is the initial
    gradient max-norm. The absolute tolerance is ignored.
    """
    if relative_tolerance <= 0.0:
        raise ValueError(
            f"relative_tolerance must be positive, got {relative_tolerance}")
    return _core.PyRelativeTermination(float(relative_tolerance))
