"""pypgo.contact — Contact energy construction facades.

``ContactSurface`` and ``ContactVertexEmbedding`` are the stable data layer;
parameter dataclasses live in ``params``, the stateful mixin in ``base``, and
concrete energy implementations in ``energies`` — the side expected to grow.
This package re-exports a flat public surface.
"""

from pypgo.contact.surface import ContactSurface, ContactVertexEmbedding
from pypgo.contact.params import (
    FloorParameters,
    FrictionParameters,
    IPCParameters,
    ObstacleSpec,
    SampledPenaltyParameters,
)
from pypgo.contact.energies import (
    FloorEnergy,
    IPCEnergy,
    SampledPenaltyEnergy,
)

__all__ = [
    "ContactSurface",
    "ContactVertexEmbedding",
    "FloorEnergy",
    "FloorParameters",
    "FrictionParameters",
    "IPCEnergy",
    "IPCParameters",
    "ObstacleSpec",
    "SampledPenaltyEnergy",
    "SampledPenaltyParameters",
]
