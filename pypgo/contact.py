"""Contact energy construction facades for pypgo."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import pypgo._core as _core
from pypgo.energy import PotentialEnergy


def _vertices(name, values):
    arr = np.asarray(values, dtype=np.float64, order="C")
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f"{name} must have shape (n, 3)")
    return arr


def _triangles(name, values):
    arr = np.asarray(values, dtype=np.int64, order="C")
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f"{name} must have shape (m, 3)")
    if arr.size and np.min(arr) < 0:
        raise ValueError(f"{name} must contain non-negative vertex indices")
    return arr


def _sparse_core(name, value):
    if isinstance(value, _core.PySparseMatrix):
        return value
    core_obj = getattr(value, "_handle", None)
    if isinstance(core_obj, _core.PySparseMatrix):
        return core_obj
    raise TypeError(f"{name} must be a pypgo.sparse.SparseMatrix")


def _finite(name, value):
    value = float(value)
    if not np.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _positive(name, value):
    value = _finite(name, value)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
    return value


def _nonnegative(name, value):
    value = _finite(name, value)
    if value < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return value


@dataclass(frozen=True)
class ContactVertexEmbedding:
    """Optional per-contact-vertex embedding metadata."""

    indices: np.ndarray
    weights: np.ndarray

    def __post_init__(self):
        indices = np.asarray(self.indices, dtype=np.int64)
        weights = np.asarray(self.weights, dtype=np.float64)
        if indices.ndim != 2 or weights.ndim != 2:
            raise ValueError("indices and weights must be 2-D arrays")
        if indices.shape != weights.shape:
            raise ValueError("indices and weights must have the same shape")
        if indices.shape[1] <= 0:
            raise ValueError("embedding arity must be positive")
        if indices.size and np.min(indices) < 0:
            raise ValueError("indices must contain non-negative vertex indices")
        if not np.all(np.isfinite(weights)):
            raise ValueError("weights must be finite")
        row_sums = weights.sum(axis=1)
        if row_sums.size and not np.allclose(row_sums, 1.0):
            raise ValueError("each embedding weight row must sum to 1")
        object.__setattr__(self, "indices", np.ascontiguousarray(indices))
        object.__setattr__(self, "weights", np.ascontiguousarray(weights))

    @property
    def embedding_arity(self) -> int:
        return int(self.indices.shape[1])


@dataclass(frozen=True)
class ContactSurface:
    """Contact surface with a surface-sized identity simulation map."""

    _handle: object
    rest_vertices: np.ndarray
    vertex_embedding: ContactVertexEmbedding | None = None

    @staticmethod
    def identity(rest_vertices, *, vertex_embedding: ContactVertexEmbedding | None = None) -> "ContactSurface":
        vertices = _vertices("rest_vertices", rest_vertices)
        if vertex_embedding is not None and not isinstance(vertex_embedding, ContactVertexEmbedding):
            raise TypeError("vertex_embedding must be a ContactVertexEmbedding")
        return ContactSurface(
            _core._create_contact_surface_identity(vertices),
            vertices.copy(),
            vertex_embedding,
        )

    @staticmethod
    def embedded(
        rest_vertices,
        surface_from_simulation,
        *,
        vertex_embedding: ContactVertexEmbedding | None = None,
    ) -> "ContactSurface":
        """Create a contact surface whose displacement is interpolated from simulation DOFs."""
        vertices = _vertices("rest_vertices", rest_vertices)
        if vertex_embedding is not None and not isinstance(vertex_embedding, ContactVertexEmbedding):
            raise TypeError("vertex_embedding must be a ContactVertexEmbedding")
        return ContactSurface(
            _core._create_contact_surface_embedded(
                vertices,
                _sparse_core("surface_from_simulation", surface_from_simulation),
            ),
            vertices.copy(),
            vertex_embedding,
        )

    @staticmethod
    def from_surface_embedding(
        embedding,
        *,
        vertex_embedding: ContactVertexEmbedding | None = None,
    ) -> "ContactSurface":
        """Create a contact surface from ``pypgo.mesh.SurfaceEmbedding``."""
        try:
            rest_surface = embedding.rest_surface
            interpolation_matrix = embedding.interpolation_matrix
        except AttributeError as exc:
            raise TypeError("embedding must be a pypgo.mesh.SurfaceEmbedding") from exc
        return ContactSurface.embedded(
            rest_surface.vertices,
            interpolation_matrix,
            vertex_embedding=vertex_embedding,
        )

    @property
    def num_surface_vertices(self) -> int:
        return self._handle.num_surface_vertices

    @property
    def num_surface_dofs(self) -> int:
        return self._handle.num_surface_dofs

    @property
    def num_simulation_dofs(self) -> int:
        return self._handle.num_simulation_dofs


@dataclass(frozen=True)
class SampledPenaltyParameters:
    stiffness: float = 1.0
    samples: int = 1
    enable_self_contact: bool = True
    enable_external_contact: bool = True

    def __post_init__(self):
        object.__setattr__(self, "stiffness", _nonnegative("stiffness", self.stiffness))
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
        object.__setattr__(self, "friction_coeff", _nonnegative("friction_coeff", self.friction_coeff))
        object.__setattr__(self, "velocity_eps", _positive("velocity_eps", self.velocity_eps))


@dataclass(frozen=True)
class IPCParameters:
    dhat: float = 1e-1
    dhat_external: float | None = None
    kappa: float = 0.1
    eps_ee: float = 0.0
    slackness: float = 1.0
    ccd_thickness: float = 0.0

    def __post_init__(self):
        dhat = _positive("dhat", self.dhat)
        object.__setattr__(self, "dhat", dhat)
        if self.dhat_external is None:
            object.__setattr__(self, "dhat_external", dhat)
        else:
            object.__setattr__(self, "dhat_external", _positive("dhat_external", self.dhat_external))
        object.__setattr__(self, "kappa", _positive("kappa", self.kappa))
        object.__setattr__(self, "eps_ee", _nonnegative("eps_ee", self.eps_ee))
        object.__setattr__(self, "slackness", _positive("slackness", self.slackness))
        object.__setattr__(self, "ccd_thickness", _nonnegative("ccd_thickness", self.ccd_thickness))


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
        object.__setattr__(self, "height", _finite("height", self.height))
        object.__setattr__(self, "stiffness", _nonnegative("stiffness", self.stiffness))


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
            _vertices("rest_vertices", rest_vertices).copy(),
            _triangles("triangles", triangles).copy(),
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
            _vertices("rest_vertices", rest_vertices).copy(),
            _triangles("triangles", triangles).copy(),
            vel.copy(),
            _finite("reference_time", reference_time),
        )

    def __post_init__(self):
        if self.kind not in {"static", "linear_velocity"}:
            raise ValueError("kind must be 'static' or 'linear_velocity'")
        object.__setattr__(self, "rest_vertices", _vertices("rest_vertices", self.rest_vertices).copy())
        object.__setattr__(self, "triangles", _triangles("triangles", self.triangles).copy())
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
        object.__setattr__(self, "reference_time", _finite("reference_time", self.reference_time))

    @property
    def t0(self) -> float:
        return self.reference_time


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
        height = _finite("height", height)
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


class _StatefulContactMixin:
    def begin_step(self, *, time: float, timestep: float, previous_x=None) -> None:
        previous = None if previous_x is None else np.asarray(previous_x, dtype=np.float64)
        self._handle.begin_step(float(time), float(timestep), previous)

    @property
    def is_step_dependent(self) -> bool:
        return bool(self._handle.is_step_dependent)


class SampledPenaltyEnergy(_StatefulContactMixin, PotentialEnergy):
    """Long-lived normal sampled-penalty contact energy."""

    def __init__(
        self,
        surface: ContactSurface,
        surface_triangles,
        *,
        params: SampledPenaltyParameters | None = None,
    ) -> None:
        if not isinstance(surface, ContactSurface):
            raise TypeError("surface must be a pypgo.contact.ContactSurface")
        params = SampledPenaltyParameters() if params is None else params
        if not isinstance(params, SampledPenaltyParameters):
            raise TypeError("params must be a SampledPenaltyParameters")
        triangles = _triangles("surface_triangles", surface_triangles)
        core = _core._create_sampled_penalty_contact_energy(
            surface._handle,
            triangles,
            params.stiffness,
            params.samples,
            params.enable_self_contact,
            params.enable_external_contact,
        )
        object.__setattr__(self, "surface", surface)
        object.__setattr__(self, "surface_triangles", triangles.copy())
        object.__setattr__(self, "params", params)
        super().__init__(core)

    def __repr__(self) -> str:
        return f"SampledPenaltyEnergy({self.num_dofs} DOFs, samples={self.params.samples})"


class IPCEnergy(_StatefulContactMixin, PotentialEnergy):
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
        triangles = _triangles("surface_triangles", surface_triangles)
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


class FrictionalSampledPenaltyEnergy(_StatefulContactMixin, PotentialEnergy):
    """Sampled-penalty contact energy with dynamic friction state."""

    def __init__(
        self,
        surface: ContactSurface,
        surface_triangles,
        *,
        params: SampledPenaltyParameters | None = None,
        friction: FrictionParameters | None = None,
    ) -> None:
        if not isinstance(surface, ContactSurface):
            raise TypeError("surface must be a pypgo.contact.ContactSurface")
        params = SampledPenaltyParameters() if params is None else params
        friction = FrictionParameters() if friction is None else friction
        if not isinstance(params, SampledPenaltyParameters):
            raise TypeError("params must be a SampledPenaltyParameters")
        if not isinstance(friction, FrictionParameters):
            raise TypeError("friction must be a FrictionParameters")
        triangles = _triangles("surface_triangles", surface_triangles)
        core = _core._create_frictional_sampled_penalty_contact_energy(
            surface._handle,
            triangles,
            params.stiffness,
            params.samples,
            params.enable_self_contact,
            params.enable_external_contact,
            friction.friction_coeff,
            friction.velocity_eps,
        )
        object.__setattr__(self, "surface", surface)
        object.__setattr__(self, "surface_triangles", triangles.copy())
        object.__setattr__(self, "params", params)
        object.__setattr__(self, "friction", friction)
        super().__init__(core)

    def begin_step(self, *, time: float, timestep: float, previous_x=None) -> None:
        if previous_x is None:
            raise ValueError("previous_x is required for FrictionalSampledPenaltyEnergy.begin_step")
        if float(timestep) <= 0.0:
            raise ValueError("timestep must be positive")
        super().begin_step(time=time, timestep=timestep, previous_x=previous_x)

    def __repr__(self) -> str:
        return (
            f"FrictionalSampledPenaltyEnergy({self.num_dofs} DOFs, "
            f"samples={self.params.samples}, friction={self.friction.friction_coeff:g})"
        )


__all__ = [
    "ContactSurface",
    "ContactVertexEmbedding",
    "FloorEnergy",
    "FloorParameters",
    "FrictionParameters",
    "FrictionalSampledPenaltyEnergy",
    "IPCEnergy",
    "IPCParameters",
    "ObstacleSpec",
    "SampledPenaltyEnergy",
    "SampledPenaltyParameters",
]
