"""pypgo.sim — state types and dynamic time-stepping.

``state`` provides ``DynamicState``/``DynamicFrame``. ``stepper`` is the
growing axis — new time integrators are added there. ``simulation`` holds the
``DynamicSimulation`` engine.

``SimulationMesh`` and shell material helpers have moved to ``pypgo.fem.mesh``.
"""

from pypgo.sim.state import DynamicFrame, DynamicState
from pypgo.sim.stepper import (
    BackwardEulerDynamicStepper,
    DynamicStepper,
    TRBDF2DynamicStepper,
)
from pypgo.sim.simulation import DynamicSimulation

__all__ = [
    "BackwardEulerDynamicStepper",
    "DynamicFrame",
    "DynamicSimulation",
    "DynamicState",
    "DynamicStepper",
    "TRBDF2DynamicStepper",
]
