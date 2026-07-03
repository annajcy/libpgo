"""Contact energy implementations.

This is the side of the contact package expected to grow: each contact energy
type lives here.  When this file gets heavy, promote it to an ``energies/``
subpackage (one module per energy type).
"""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo._utils import (
    finite_scalar,
    nonnegative_scalar,
    positive_scalar,
    triangle_array,
)
from pypgo.energy import PotentialEnergy
from pypgo.contact.base import StatefulContactMixin
from pypgo.contact.params import (
    FloorParameters,
    FrictionParameters,
    IPCParameters,
    ObstacleSpec,
    SampledPenaltyParameters,
)
from pypgo.contact.surface import ContactSurface

_UNSET = object()


class FloorEnergy(PotentialEnergy):
    """One-sided floor/ceiling penalty over a contact surface."""

    def __init__(
        self,
        surface: ContactSurface,
        *,
        parameters: FloorParameters | None = None,
        axis=_UNSET,
        side=_UNSET,
        height=_UNSET,
        stiffness=_UNSET,
    ) -> None:
        if not isinstance(surface, ContactSurface):
            raise TypeError("surface must be a pypgo.contact.ContactSurface")
        if parameters is not None:
            if any(value is not _UNSET for value in (axis, side, height, stiffness)):
                raise ValueError("parameters cannot be combined with split floor fields")
            if not isinstance(parameters, FloorParameters):
                raise TypeError("parameters must be a FloorParameters")
        else:
            parameters = FloorParameters(
                axis="z" if axis is _UNSET else axis,
                side="keep_above" if side is _UNSET else side,
                height=0.0 if height is _UNSET else height,
                stiffness=1.0 if stiffness is _UNSET else stiffness,
            )

        handle = _core._create_floor_contact_energy(
            surface._handle,
            parameters.axis,
            parameters.side,
            parameters.height,
            parameters.stiffness,
        )
        object.__setattr__(self, "surface", surface)
        object.__setattr__(self, "parameters", parameters)
        object.__setattr__(self, "axis", parameters.axis)
        object.__setattr__(self, "side", parameters.side)
        object.__setattr__(self, "height", parameters.height)
        object.__setattr__(self, "stiffness", parameters.stiffness)
        super().__init__(handle)

    def set_height(self, height: float) -> None:
        height = finite_scalar("height", height)
        _core._set_floor_contact_height(self._handle, height)
        object.__setattr__(self, "height", height)
        object.__setattr__(
            self,
            "parameters",
            FloorParameters(
                axis=self.axis,
                side=self.side,
                height=height,
                stiffness=self.stiffness,
            ),
        )

    def __repr__(self) -> str:
        return (
            f"FloorEnergy({self.num_dofs} DOFs, axis={self.axis!r}, "
            f"side={self.side!r}, height={self.height:g})"
        )


class SampledPenaltyEnergy(StatefulContactMixin, PotentialEnergy):
    """Long-lived sampled-penalty contact energy."""

    def __init__(
        self,
        surface: ContactSurface,
        surface_triangles,
        *,
        params: SampledPenaltyParameters | None = None,
        friction: FrictionParameters | None = None,
        obstacles=None,
    ) -> None:
        if not isinstance(surface, ContactSurface):
            raise TypeError("surface must be a pypgo.contact.ContactSurface")
        params = SampledPenaltyParameters() if params is None else params
        if not isinstance(params, SampledPenaltyParameters):
            raise TypeError("params must be a SampledPenaltyParameters")
        if friction is not None and not isinstance(friction, FrictionParameters):
            raise TypeError("friction must be a FrictionParameters or None")
        obstacle_specs = [] if obstacles is None else list(obstacles)
        if any(not isinstance(obs, ObstacleSpec) for obs in obstacle_specs):
            raise TypeError("obstacles must be an iterable of ObstacleSpec")
        if any(obs.kind != "static" for obs in obstacle_specs):
            raise ValueError("SampledPenaltyEnergy only supports static obstacles")
        triangles = triangle_array("surface_triangles", surface_triangles)
        core = _core._create_sampled_penalty_contact_energy(
            surface._handle,
            triangles,
            params.stiffness,
            params.samples,
            params.enable_self_contact,
            params.enable_external_contact,
            friction.friction_coeff if friction is not None else None,
            friction.velocity_eps if friction is not None else None,
            obstacle_specs,
        )
        object.__setattr__(self, "surface", surface)
        object.__setattr__(self, "surface_triangles", triangles.copy())
        object.__setattr__(self, "params", params)
        object.__setattr__(self, "friction", friction)
        object.__setattr__(self, "obstacles", tuple(obstacle_specs))
        super().__init__(core)

    def begin_step(self, *, time: float, timestep: float, previous_x=None) -> None:
        if self.friction is not None:
            if previous_x is None:
                raise ValueError("previous_x is required for SampledPenaltyEnergy.begin_step with friction")
            if float(timestep) <= 0.0:
                raise ValueError("timestep must be positive")
        super().begin_step(time=time, timestep=timestep, previous_x=previous_x)

    def __repr__(self) -> str:
        if self.friction is None:
            return f"SampledPenaltyEnergy({self.num_dofs} DOFs, samples={self.params.samples})"
        return (
            f"SampledPenaltyEnergy({self.num_dofs} DOFs, "
            f"samples={self.params.samples}, friction={self.friction.friction_coeff:g})"
        )


class IPCEnergy(StatefulContactMixin, PotentialEnergy):
    """Long-lived IPC contact energy over a surface."""

    def __init__(
        self,
        surface: ContactSurface,
        surface_triangles,
        *,
        params: IPCParameters | None = None,
        obstacles=None,
    ) -> None:
        if not isinstance(surface, ContactSurface):
            raise TypeError("surface must be a pypgo.contact.ContactSurface")
        params = IPCParameters() if params is None else params
        if not isinstance(params, IPCParameters):
            raise TypeError("params must be an IPCParameters")
        triangles = triangle_array("surface_triangles", surface_triangles)
        obstacle_specs = [] if obstacles is None else list(obstacles)
        if any(not isinstance(obs, ObstacleSpec) for obs in obstacle_specs):
            raise TypeError("obstacles must be an iterable of ObstacleSpec")
        core = _core._create_ipc_contact_energy(
            surface._handle,
            triangles,
            params.dhat,
            params.dhat_external,
            params.kappa,
            params.eps_ee,
            params.slackness,
            params.ccd_thickness,
            obstacle_specs,
        )
        object.__setattr__(self, "surface", surface)
        object.__setattr__(self, "surface_triangles", triangles.copy())
        object.__setattr__(self, "params", params)
        object.__setattr__(self, "obstacles", tuple(obstacle_specs))
        super().__init__(core)

    def set_moving_obstacle_time(self, time: float) -> None:
        self._handle.set_moving_obstacle_time(float(time))

    def __repr__(self) -> str:
        return f"IPCEnergy({self.num_dofs} DOFs, dhat={self.params.dhat:g})"
