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

    @classmethod
    def from_elastic_parameter(
        cls, *, scale: float, parameter
    ) -> "ShellArealDensity":
        from pypgo.fem.fields import OptimizableParameterRef

        if not isinstance(parameter, OptimizableParameterRef):
            raise TypeError(
                f"parameter must be a OptimizableParameterRef, got {type(parameter).__name__}"
            )
        scale_arr = np.asarray(scale, dtype=np.float64)
        if scale_arr.ndim != 0:
            raise ValueError(f"scale must be a scalar, got shape {scale_arr.shape}")
        _require_positive_finite("scale", scale_arr)
        result = cls.__new__(cls)
        result._parameter = parameter
        result._handle = _core.make_shell_areal_density_from_elastic_parameter(
            float(scale_arr), parameter._handle
        )
        return result

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"


class SelfWeightGravity:
    """External load from a parameter-coupled shell areal density."""

    def __init__(
        self, *, formulation, mesh, areal_density, optimizable_parameters, acceleration
    ) -> None:
        from pypgo.fem.formulations import ShellFormulation
        from pypgo.fem.fields import OptimizableParameters

        if not isinstance(formulation, ShellFormulation):
            raise TypeError(
                f"formulation must be a ShellFormulation, got {type(formulation).__name__}"
            )
        if not isinstance(areal_density, ShellArealDensity):
            raise TypeError(
                "areal_density must be a ShellArealDensity, "
                f"got {type(areal_density).__name__}"
            )
        if not isinstance(optimizable_parameters, OptimizableParameters):
            raise TypeError(
                "optimizable_parameters must be OptimizableParameters, "
                f"got {type(optimizable_parameters).__name__}"
            )
        self._formulation = formulation
        self._mesh = mesh
        self._areal_density = areal_density
        self._optimizable_parameters = optimizable_parameters
        self._acceleration = np.asarray(acceleration, dtype=np.float64).reshape(3)

    @property
    def optimizable_parameters(self):
        return self._optimizable_parameters

    def force(self) -> np.ndarray:
        return self._formulation.body_force(
            self._mesh,
            self._acceleration,
            self._areal_density,
            optimizable_parameters=self._optimizable_parameters,
        )

    def parameter_jacobian(self):
        return self._formulation.body_force_parameter_jacobian(
            self._mesh,
            self._acceleration,
            self._areal_density,
            optimizable_parameters=self._optimizable_parameters,
        )
