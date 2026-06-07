"""Stateful contact mixin — shared by contact energies that maintain per-step state."""

from __future__ import annotations

import numpy as np


class StatefulContactMixin:
    def begin_step(self, *, time: float, timestep: float, previous_x=None) -> None:
        previous = None if previous_x is None else np.asarray(previous_x, dtype=np.float64)
        self._handle.begin_step(float(time), float(timestep), previous)

    @property
    def is_step_dependent(self) -> bool:
        return bool(self._handle.is_step_dependent)
