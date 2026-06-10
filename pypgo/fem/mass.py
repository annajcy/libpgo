"""Mass-property fields for formulation mass / body-force assembly.

A mass field answers "how much mass per integration region" — separate from
the constitutive material. Volume fields carry kg/m^3, shell fields kg/m^2.
"""

from __future__ import annotations

import numpy as np

import pypgo._core as _core


class VolumeMassField:
    """Base for volumetric (kg/m^3) mass fields. Holds a C++ handle."""

    def __init__(self, handle) -> None:
        self._handle = handle

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"


class VolumeDensity(VolumeMassField):
    """Volumetric density: scalar (constant) or 1-D per-element array."""

    def __init__(self, density) -> None:
        arr = np.asarray(density, dtype=np.float64)
        if arr.ndim == 0:
            super().__init__(_core.make_constant_volume_density(float(arr)))
        elif arr.ndim == 1:
            super().__init__(_core.make_elementwise_volume_density(arr.tolist()))
        else:
            raise ValueError(f"density must be a scalar or 1-D array, got shape {arr.shape}")


def volume_density_from_veg(volume) -> VolumeDensity:
    """Per-element densities from a VolumeMesh's .veg material regions."""
    densities = np.zeros(volume.num_elements, dtype=np.float64)
    for _name, material, elements in volume.to_veg_file().to_volume_regions():
        densities[np.asarray(elements, dtype=np.int64)] = float(material.density)
    return VolumeDensity(densities)
