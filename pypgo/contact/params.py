"""Parameter and configuration dataclasses for contact energies.

This is the side of the contact package expected to grow alongside the energy
implementations in ``energies.py``.  Each contact method defines its own
parameter value-object here.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from pypgo._utils import finite_scalar, nonnegative_scalar, positive_scalar, vertex_array, triangle_array


@dataclass(frozen=True)
class SampledPenaltyParameters:
    stiffness: float = 1.0
    samples: int = 1
    enable_self_contact: bool = True
    enable_external_contact: bool = True

    def __post_init__(self):
        object.__setattr__(self, "stiffness", nonnegative_scalar("stiffness", self.stiffness))
        if int(self.samples) <= 0:
            raise ValueError("samples must be positive")
        object.__setattr__(self, "samples", int(self.samples))
        object.__setattr__(self, "enable_self_contact", bool(self.enable_self_contact))
        object.__setattr__(self, "enable_external_contact", bool(self.enable_external_contact))


@dataclass(frozen=True)
class FrictionParameters:
    friction_coeff: float = 1.0
    velocity_eps: float = 1.0

    def __post_init__(self):
        object.__setattr__(self, "friction_coeff", nonnegative_scalar("friction_coeff", self.friction_coeff))
        object.__setattr__(self, "velocity_eps", positive_scalar("velocity_eps", self.velocity_eps))


@dataclass(frozen=True)
class IPCParameters:
    dhat: float = 1e-1
    dhat_external: float | None = None
    kappa: float = 0.1
    eps_ee: float = 0.0
    slackness: float = 1.0
    ccd_thickness: float = 0.0

    def __post_init__(self):
        dhat = positive_scalar("dhat", self.dhat)
        object.__setattr__(self, "dhat", dhat)
        if self.dhat_external is None:
            object.__setattr__(self, "dhat_external", dhat)
        else:
            object.__setattr__(self, "dhat_external", positive_scalar("dhat_external", self.dhat_external))
        object.__setattr__(self, "kappa", positive_scalar("kappa", self.kappa))
        object.__setattr__(self, "eps_ee", nonnegative_scalar("eps_ee", self.eps_ee))
        object.__setattr__(self, "slackness", positive_scalar("slackness", self.slackness))
        object.__setattr__(self, "ccd_thickness", nonnegative_scalar("ccd_thickness", self.ccd_thickness))


@dataclass(frozen=True)
class FloorParameters:
    axis: str = "z"
    side: str = "keep_above"
    height: float = 0.0
    stiffness: float = 1.0

    def __post_init__(self):
        if self.axis not in {"x", "y", "z"}:
            raise ValueError("axis must be 'x', 'y', or 'z'")
        if self.side not in {"keep_above", "keep_below"}:
            raise ValueError("side must be 'keep_above' or 'keep_below'")
        object.__setattr__(self, "height", finite_scalar("height", self.height))
        object.__setattr__(self, "stiffness", nonnegative_scalar("stiffness", self.stiffness))


@dataclass(frozen=True)
class ObstacleSpec:
    """Value object for IPC external obstacle construction."""

    kind: str
    rest_vertices: np.ndarray
    triangles: np.ndarray
    velocity: np.ndarray | None = None
    reference_time: float = 0.0

    @staticmethod
    def static(rest_vertices, triangles) -> "ObstacleSpec":
        return ObstacleSpec(
            "static",
            vertex_array("rest_vertices", rest_vertices).copy(),
            triangle_array("triangles", triangles).copy(),
        )

    @staticmethod
    def linear_velocity(
        rest_vertices,
        triangles,
        velocity,
        *,
        reference_time: float = 0.0,
        t0: float | None = None,
    ) -> "ObstacleSpec":
        vel = np.asarray(velocity, dtype=np.float64)
        if vel.shape != (3,):
            raise ValueError(f"velocity must be a 3-vector, got shape {vel.shape}")
        if t0 is not None:
            reference_time = t0
        return ObstacleSpec(
            "linear_velocity",
            vertex_array("rest_vertices", rest_vertices).copy(),
            triangle_array("triangles", triangles).copy(),
            vel.copy(),
            finite_scalar("reference_time", reference_time),
        )

    def __post_init__(self):
        if self.kind not in {"static", "linear_velocity"}:
            raise ValueError("kind must be 'static' or 'linear_velocity'")
        object.__setattr__(self, "rest_vertices", vertex_array("rest_vertices", self.rest_vertices).copy())
        object.__setattr__(self, "triangles", triangle_array("triangles", self.triangles).copy())
        if self.velocity is None:
            if self.kind == "linear_velocity":
                raise ValueError("velocity is required for linear_velocity obstacles")
        else:
            vel = np.asarray(self.velocity, dtype=np.float64)
            if vel.shape != (3,):
                raise ValueError(f"velocity must be a 3-vector, got shape {vel.shape}")
            if not np.all(np.isfinite(vel)):
                raise ValueError("velocity must be finite")
            object.__setattr__(self, "velocity", vel.copy())
        object.__setattr__(self, "reference_time", finite_scalar("reference_time", self.reference_time))

    @property
    def t0(self) -> float:
        return self.reference_time
