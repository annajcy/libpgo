"""Dynamic time integrators for ``DynamicSimulation``.

This is the side of the sim package expected to grow: each integrator is a
concrete ``DynamicStepper`` subclass and lives here.  When this file gets heavy,
promote it to a ``stepper/`` subpackage (one module per integrator).
"""

from __future__ import annotations

import pypgo._core as _core


class DynamicStepper:
    """Base facade for dynamic time integrators used by ``DynamicSimulation``."""

    def __init__(self, core_obj):
        if not isinstance(core_obj, _core.PyDynamicStepper):
            raise TypeError(f"core_obj must be PyDynamicStepper, got {type(core_obj).__name__}")
        self._handle = core_obj


class BackwardEulerDynamicStepper(DynamicStepper):
    """Implicit/backward Euler dynamic stepper."""

    def __init__(self):
        super().__init__(_core.PyBackwardEulerDynamicStepper())

    def __repr__(self) -> str:
        return "BackwardEulerDynamicStepper()"


class TRBDF2DynamicStepper(DynamicStepper):
    """Two-stage TRBDF2 dynamic stepper."""

    def __init__(self, gamma: float = 0.5):
        super().__init__(_core.PyTRBDF2DynamicStepper(float(gamma)))

    @property
    def gamma(self) -> float:
        return float(self._handle.gamma)

    def __repr__(self) -> str:
        return f"TRBDF2DynamicStepper(gamma={self.gamma!r})"
