"""Per-frame kinematic state and step-result types for dynamic simulation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from pypgo import solver as _solver


@dataclass(frozen=True)
class DynamicState:
    """Per-frame kinematic state carried across dynamic time steps."""

    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    timestep_id: int = 0
    time: float = 0.0


@dataclass(frozen=True)
class DynamicFrame:
    """Result of a single ``DynamicSimulation.step``.

    ``solver_result`` is the final stage's solve; ``stage_results`` holds one
    entry for implicit Euler and two for TRBDF2 (``gamma`` < 1). Per-stage
    solution vectors are not captured, so stage results carry an empty ``x``.
    """

    frame_index: int
    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    solver_result: _solver.SolverResult
    stage_results: Sequence[_solver.SolverResult]
    accepted: bool
