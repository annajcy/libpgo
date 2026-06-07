"""pypgo.constraints - hard constraint function helpers.

``ConstraintFunction`` is the stable contract; concrete constraint types live
in ``functions.py`` and are the side expected to grow.  ``Bounded`` wraps a
constraint function with element-wise lower/upper bounds.  This package
re-exports a flat public surface.
"""

from pypgo.constraints.base import ConstraintFunction
from pypgo.constraints.functions import Linear, ConstraintFunctionSet
from pypgo.constraints.bounded import Bounded

__all__ = [
    "Bounded",
    "ConstraintFunction",
    "ConstraintFunctionSet",
    "Linear",
]
