"""pypgo.energy — General-purpose energy types for pypgo.

Each Python facade stores its concrete C++ PyXXXX peer in ``_handle``.
Energy peers inherit ``_core.PyPotentialEnergy``, and C++ internals use
``potentialEnergyHandle()`` when a core ``PotentialEnergy`` pointer is required.
Evaluation dispatches through the virtual method so Python users never see
hessianInPlace / hessianAlloc / isHessianTopologyFixed.

``PotentialEnergy`` is the stable contract; concrete energies are grouped by
concept (algebraic / penalty / attachment / composition) and are the side
expected to grow.  This package re-exports a flat public surface.
"""

from pypgo.energy.base import PotentialEnergy
from pypgo.energy.algebraic import LinearEnergy, QuadraticEnergy
from pypgo.energy.penalty import ConstraintPenalty, ConstraintViolationPenalty
from pypgo.energy.attachment import VertexAttachment
from pypgo.energy.sets import EnergySet

__all__ = [
    "ConstraintPenalty",
    "ConstraintViolationPenalty",
    "EnergySet",
    "LinearEnergy",
    "PotentialEnergy",
    "QuadraticEnergy",
    "VertexAttachment",
]
