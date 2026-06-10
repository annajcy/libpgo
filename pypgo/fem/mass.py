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


class ShellMassField:
    """Base for shell (kg/m^2) mass fields. Holds a C++ handle."""

    def __init__(self, handle) -> None:
        self._handle = handle

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"


class ShellArealDensity(ShellMassField):
    """Constant areal density rho*h in kg/m^2."""

    def __init__(self, areal_density: float) -> None:
        super().__init__(_core.make_constant_shell_areal_density(float(areal_density)))


class ShellDensityThickness(ShellMassField):
    """rho * h with fixed thickness (scalar or per-element array)."""

    def __init__(self, *, density: float, thickness) -> None:
        arr = np.asarray(thickness, dtype=np.float64)
        if arr.ndim == 0:
            super().__init__(_core.make_shell_density_thickness_constant(float(density), float(arr)))
        elif arr.ndim == 1:
            super().__init__(_core.make_shell_density_thickness_elementwise(float(density), arr.tolist()))
        else:
            raise ValueError(f"thickness must be a scalar or 1-D array, got shape {arr.shape}")


class ShellDensityElasticThickness(ShellMassField):
    """rho * h with h read live from an elastic ParameterField channel.

    Shares storage with the energy's elastic field: set_elastic_values()
    updates the thickness seen here, no manual sync.
    """

    def __init__(self, *, density: float, parameter_field, channel: int = 4) -> None:
        from pypgo.fem.fields import ParameterField

        if not isinstance(parameter_field, ParameterField):
            raise TypeError(
                f"parameter_field must be a ParameterField, got {type(parameter_field).__name__}")
        super().__init__(_core.make_shell_density_elastic_thickness(
            float(density), parameter_field._handle, int(channel)))


class SelfWeightGravity:
    """External-load provider: shell self-weight from a parameter-coupled mass field.

    Implements the ``ElasticStaticEquilibriumLayer`` external_load protocol:
    ``force()`` and ``parameter_jacobian()`` evaluated at the parameter
    field's current values.
    """

    def __init__(self, *, formulation, sim_mesh, mass_field, acceleration) -> None:
        from pypgo.fem.formulations import ShellFormulation

        if not isinstance(formulation, ShellFormulation):
            raise TypeError(
                f"formulation must be a ShellFormulation, got {type(formulation).__name__}")
        if not isinstance(mass_field, ShellMassField):
            raise TypeError(
                f"mass_field must be a ShellMassField, got {type(mass_field).__name__}")
        self._formulation = formulation
        self._sim_mesh = sim_mesh
        self._mass_field = mass_field
        self._acceleration = np.asarray(acceleration, dtype=np.float64).reshape(3)

    def force(self) -> np.ndarray:
        return self._formulation.body_force(self._sim_mesh, self._acceleration, self._mass_field)

    def parameter_jacobian(self):
        return self._formulation.body_force_parameter_jacobian(
            self._sim_mesh, self._acceleration, self._mass_field)
