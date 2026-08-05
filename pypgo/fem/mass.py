"""Density fields for formulation mass / body-force assembly.

VolumeDensity carries kg/m^3 and ShellArealDensity carries kg/m^2.  They are
physical-property wrappers; formulation objects own integration and assembly.
"""

from __future__ import annotations

import numpy as np

import pypgo._core as _core


def _require_positive_finite(name: str, value: np.ndarray) -> None:
    if (
        value.size == 0
        or not np.all(np.isfinite(value))
        or not np.all(value > 0.0)
    ):
        raise ValueError(f"{name} must contain finite values > 0")


class VolumeDensity:
    """Finite positive density: scalar or one value per volume element."""

    def __init__(self, value) -> None:
        arr = np.asarray(value, dtype=np.float64)
        if arr.ndim == 0:
            _require_positive_finite("density", arr)
            self._handle = _core.make_constant_volume_density(float(arr))
        elif arr.ndim == 1:
            _require_positive_finite("density", arr)
            self._handle = _core.make_elementwise_volume_density(arr.tolist())
        else:
            raise ValueError(f"density must be a scalar or 1-D array, got shape {arr.shape}")

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"


def volume_density(volume) -> VolumeDensity:
    """Per-element densities from a VolumeMesh's .veg material regions."""
    densities = np.zeros(volume.num_elements, dtype=np.float64)
    veg = volume.to_veg_file()
    for region in veg.regions:
        material = veg.materials[region.material_index]
        elements = veg.sets[region.set_index].elements
        densities[np.asarray(elements, dtype=np.int64)] = float(material.density)
    return VolumeDensity(densities)


class ShellArealDensity:
    """Finite positive areal density: scalar or one value per shell element."""

    def __init__(self, value) -> None:
        arr = np.asarray(value, dtype=np.float64)
        if arr.ndim == 0:
            _require_positive_finite("areal density", arr)
            self._handle = _core.make_constant_shell_areal_density(float(arr))
        elif arr.ndim == 1:
            _require_positive_finite("areal density", arr)
            self._handle = _core.make_shell_areal_density_elementwise(arr.tolist())
        else:
            raise ValueError(
                f"areal density must be a scalar or 1-D array, got shape {arr.shape}"
            )

    @classmethod
    def from_density_thickness(cls, *, density: float, thickness) -> "ShellArealDensity":
        density_arr = np.asarray(density, dtype=np.float64)
        if density_arr.ndim != 0:
            raise ValueError(
                f"density must be a scalar, got shape {density_arr.shape}"
            )
        _require_positive_finite("density", density_arr)
        arr = np.asarray(thickness, dtype=np.float64)
        result = cls.__new__(cls)
        if arr.ndim == 0:
            _require_positive_finite("thickness", arr)
            result._handle = _core.make_shell_areal_density_from_density_thickness(
                float(density_arr), float(arr)
            )
        elif arr.ndim == 1:
            _require_positive_finite("thickness", arr)
            result._handle = _core.make_shell_areal_density_from_density_thickness(
                float(density_arr), arr.tolist()
            )
        else:
            raise ValueError(
                f"thickness must be a scalar or 1-D array, got shape {arr.shape}"
            )
        return result

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"
