"""Energy composition: weighted sums of energy terms."""

from __future__ import annotations

import pypgo._core as _core
from pypgo.energy.base import PotentialEnergy


class EnergySet(PotentialEnergy):
    """Weighted sum of energy terms: total = sum_i weight_i * energy_i.

    Shares the full :class:`PotentialEnergy` evaluation interface
    (``value``/``gradient``/``hessian``/``max_step``/``dofs``/``state_kind``/
    ``zero_state``); only the term-management surface is added here.

    Parameters
    ----------
    terms : list of (energy, weight) tuples
        Each energy must be a :class:`PotentialEnergy` (this includes domain
        energies such as ``pypgo.fem`` and ``pypgo.contact`` energies, and
        nested ``EnergySet`` instances).  weight is a float scalar.
    """

    def __init__(self, terms):
        cpp_terms = []
        for i, (energy, weight) in enumerate(terms):
            if not isinstance(energy, PotentialEnergy):
                raise TypeError(f"term {i}: energy must be a pypgo.energy.PotentialEnergy")
            cpp_terms.append((energy._handle, float(weight)))
        handle = _core._create_energy_set(cpp_terms)
        super().__init__(handle)

    @property
    def num_terms(self) -> int:
        return self._handle.num_terms

    def set_weight(self, i: int, w: float) -> None:
        self._handle.set_weight(i, float(w))

    def __repr__(self) -> str:
        return self._handle.__repr__()
