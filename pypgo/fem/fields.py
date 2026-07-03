"""Parameter field descriptors and field views."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import pypgo._core as _core


# ---------------------------------------------------------------------------
# Parameter field descriptors
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ElementwiseField:
    """Elementwise parameter field descriptor.

    ``values=None`` asks C++ to create model-appropriate default values from the
    mesh/material payload. Otherwise values may be flat or shaped as
    ``(num_elements, num_channels)``.
    """

    values: object = None


@dataclass(frozen=True)
class ConstantField:
    """Constant (mesh-wide shared) parameter field descriptor.

    A single set of ``num_channels`` parameters is shared by every element.
    ``values=None`` asks C++ to seed the shared values from the mesh/material
    payload. Otherwise values may be flat ``(num_channels,)`` or ``(1, num_channels)``.
    """

    values: object = None


# ---------------------------------------------------------------------------
# ParameterField — view of a state-owned C++ field
# ---------------------------------------------------------------------------


class ParameterField:
    """View of a state-owned C++ parameter field."""

    def __init__(self, core) -> None:
        if not isinstance(core, _core.PyParameterField):
            raise TypeError(
                f"core must be a PyParameterField, got {type(core).__name__}"
            )
        self._handle = core

    @property
    def domain(self) -> str:
        return self._handle.domain

    @property
    def model(self) -> str:
        return self._handle.model

    @property
    def num_elements(self) -> int:
        return self._handle.num_elements

    @property
    def num_value_rows(self) -> int:
        return self._handle.num_value_rows

    @property
    def num_channels(self) -> int:
        return self._handle.num_channels

    @property
    def values(self) -> np.ndarray:
        return np.asarray(self._handle.values(), dtype=np.float64).copy()
