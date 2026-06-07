"""High-level simulation scene builders.

This module is the Python orchestration layer for common simulation workflows.
The numerical kernels stay in ``pypgo.fem``, ``pypgo.contact``, and
``pypgo.sim``; this file wires them into a small, reusable runner.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from pypgo._utils import vec3_finite
import pypgo.contact as _contact
import pypgo.energy as _energy
import pypgo.fem as _fem
import pypgo.solver as _solver
from pypgo.mesh import TriMeshData
from pypgo.mesh.volume import VolumeMesh
from pypgo.sim import (
    BackwardEulerDynamicStepper,
    DynamicSimulation,
    DynamicState,
    DynamicStepper,
    SimulationMesh,
)


@dataclass(frozen=True)
class RuntimeConfig:
    """Dynamic runtime settings for a short Python-driven simulation."""

    timestep: float
    num_steps: int = 1
    gravity: Sequence[float] = (0.0, 0.0, 0.0)
    initial_displacement: Sequence[float] = (0.0, 0.0, 0.0)
    initial_velocity: Sequence[float] = (0.0, 0.0, 0.0)
    damping: tuple[float, float] = (0.0, 0.0)
    integrator: DynamicStepper | str = field(default_factory=BackwardEulerDynamicStepper)
    solver_max_iterations: int = 50
    solver_gradient_tolerance: float = 1e-6
    solver_line_search: str = "backtrack"
    solver_sparse: str = "auto"
    solver_verbose: int = 0

    def __post_init__(self):
        if float(self.timestep) <= 0.0:
            raise ValueError("timestep must be positive")
        if int(self.num_steps) < 0:
            raise ValueError("num_steps must be non-negative")
        vec3_finite("gravity", self.gravity)
        vec3_finite("initial_displacement", self.initial_displacement)
        vec3_finite("initial_velocity", self.initial_velocity)
        if len(tuple(self.damping)) != 2:
            raise ValueError("damping must contain two values")


@dataclass(frozen=True)
class IPCContactSpec:
    """IPC self/external contact settings."""

    parameters: _contact.IPCParameters = field(default_factory=_contact.IPCParameters)
    obstacles: Sequence[_contact.ObstacleSpec] = ()

    def __post_init__(self):
        if not isinstance(self.parameters, _contact.IPCParameters):
            raise TypeError("parameters must be pypgo.contact.IPCParameters")
        if any(not isinstance(obs, _contact.ObstacleSpec) for obs in self.obstacles):
            raise TypeError("obstacles must contain pypgo.contact.ObstacleSpec objects")


@dataclass(frozen=True)
class FloorSpec:
    """One floor/ceiling penalty term attached to the simulation contact surface."""

    axis: str = "z"
    side: str = "keep_above"
    height: float = 0.0
    stiffness: float = 1.0

    def to_parameters(self) -> _contact.FloorParameters:
        return _contact.FloorParameters(
            axis=self.axis,
            side=self.side,
            height=self.height,
            stiffness=self.stiffness,
        )


@dataclass(frozen=True)
class VolumeIPCSimulationSpec:
    """Input bundle for a volumetric dynamic IPC scene."""

    volume: VolumeMesh
    surface: TriMeshData
    runtime: RuntimeConfig
    contact: IPCContactSpec = field(default_factory=IPCContactSpec)
    floors: Sequence[FloorSpec] = ()
    formulation: object | None = None
    elastic: object = field(default_factory=_fem.StableNeo)
    elastic_field: object = field(default_factory=_fem.ElementwiseField)
    plastic: object = field(default_factory=lambda: _fem.VolumetricPlasticity(dofs=0))
    plastic_field: object = field(default_factory=_fem.ElementwiseField)
    deformation_options: _fem.DeformationOptions = field(default_factory=_fem.DeformationOptions)

    def __post_init__(self):
        if not isinstance(self.volume, VolumeMesh):
            raise TypeError("volume must be pypgo.mesh.volume.VolumeMesh")
        if not isinstance(self.surface, TriMeshData):
            raise TypeError("surface must be pypgo.mesh.TriMeshData")
        if not isinstance(self.runtime, RuntimeConfig):
            raise TypeError("runtime must be RuntimeConfig")
        if not isinstance(self.contact, IPCContactSpec):
            raise TypeError("contact must be IPCContactSpec")
        if any(not isinstance(floor, FloorSpec) for floor in self.floors):
            raise TypeError("floors must contain FloorSpec objects")


@dataclass(frozen=True)
class VolumeIPCSimulationBuild:
    """Immutable pieces produced by ``build_volume_ipc_simulation``."""

    mass: object
    deformation: _energy.PotentialEnergy
    contact_surface: _contact.ContactSurface
    ipc: _contact.IPCEnergy
    floors: tuple[_contact.FloorEnergy, ...]
    energy: _energy.EnergySet
    external_force: np.ndarray
    initial_state: DynamicState
    optimizer: _solver.Optimizer
    surface_map: object


class VolumeIPCSimulationRunner:
    """Small dynamic runner for a built volumetric IPC scene."""

    def __init__(
        self,
        spec: VolumeIPCSimulationSpec,
        build: VolumeIPCSimulationBuild,
        simulation: DynamicSimulation,
    ) -> None:
        self.spec = spec
        self.build = build
        self._simulation = simulation

    @property
    def num_dofs(self) -> int:
        return self._simulation.num_dofs

    @property
    def state(self) -> DynamicState:
        return self._simulation.state

    @property
    def surface(self) -> TriMeshData:
        return self.spec.surface

    @property
    def energy(self) -> _energy.EnergySet:
        return self.build.energy

    def step(self, *, external_force=None, optimizer=None):
        runtime = self.spec.runtime
        if self.build.ipc.obstacles:
            self.build.ipc.set_moving_obstacle_time(self.state.time + runtime.timestep)
        return self._simulation.step(
            external_force=self.build.external_force if external_force is None else external_force,
            optimizer=self.build.optimizer if optimizer is None else optimizer,
        )

    def run(self, num_steps: int | None = None) -> list:
        steps = self.spec.runtime.num_steps if num_steps is None else int(num_steps)
        if steps < 0:
            raise ValueError("num_steps must be non-negative")
        return [self.step() for _ in range(steps)]

    def surface_displacement(self, displacement=None) -> np.ndarray:
        disp = self.state.displacement if displacement is None else displacement
        mapped = self.build.surface_map @ np.asarray(disp, dtype=np.float64).reshape(-1)
        return np.asarray(mapped, dtype=np.float64).reshape((-1, 3))

    def deformed_surface(self, displacement=None) -> TriMeshData:
        return TriMeshData(
            self.surface.vertices + self.surface_displacement(displacement),
            self.surface.elements,
        )


def build_volume_ipc_simulation(spec: VolumeIPCSimulationSpec) -> VolumeIPCSimulationRunner:
    """Build a dynamic volumetric IPC runner from Python objects."""

    if not isinstance(spec, VolumeIPCSimulationSpec):
        raise TypeError("spec must be VolumeIPCSimulationSpec")

    formulation = spec.formulation if spec.formulation is not None else _default_volume_formulation(spec.volume)
    sim_mesh = SimulationMesh.create_volumetric(spec.volume)
    deformation_state = _fem.deformation_model_state(
        sim_mesh,
        elastic=spec.elastic,
        elastic_field=spec.elastic_field,
        plastic=spec.plastic,
        plastic_field=spec.plastic_field,
    )
    deformation = _fem.deformation_energy(
        deformation_state,
        formulation=formulation,
        options=spec.deformation_options,
    )

    mass = formulation.mass_matrix(spec.volume)
    surface_map = _surface_embedding_matrix(spec.volume, spec.surface, formulation)
    contact_surface = _contact.ContactSurface.embedded(spec.surface.vertices, surface_map)
    ipc = _contact.IPCEnergy(
        contact_surface,
        spec.surface.elements,
        params=spec.contact.parameters,
        obstacles=spec.contact.obstacles,
    )
    floors = tuple(
        _contact.FloorEnergy(contact_surface, parameters=floor.to_parameters())
        for floor in spec.floors
    )

    terms = [(deformation, 1.0), (ipc, 1.0), *((floor, 1.0) for floor in floors)]
    energy = _energy.EnergySet(terms)
    external_force = formulation.body_force(spec.volume, spec.runtime.gravity)
    initial_state = DynamicState(
        displacement=_initial_vector(
            spec.volume,
            deformation.num_dofs,
            spec.runtime.initial_displacement,
            formulation,
        ),
        velocity=_initial_vector(
            spec.volume,
            deformation.num_dofs,
            spec.runtime.initial_velocity,
            formulation,
        ),
        acceleration=np.zeros(deformation.num_dofs, dtype=np.float64),
    )
    optimizer = _solver.NewtonOptimizer(
        max_iterations=spec.runtime.solver_max_iterations,
        gradient_tolerance=spec.runtime.solver_gradient_tolerance,
        line_search=spec.runtime.solver_line_search,
        verbose=spec.runtime.solver_verbose,
        sparse_solver=spec.runtime.solver_sparse,
    )

    if hasattr(ipc, "begin_step"):
        ipc.begin_step(
            time=0.0,
            timestep=spec.runtime.timestep,
            previous_x=initial_state.displacement,
        )

    simulation = DynamicSimulation(
        mass=mass,
        state=initial_state,
        timestep=spec.runtime.timestep,
        energy=energy,
        integrator=spec.runtime.integrator,
        damping=spec.runtime.damping,
    )
    build = VolumeIPCSimulationBuild(
        mass=mass,
        deformation=deformation,
        contact_surface=contact_surface,
        ipc=ipc,
        floors=floors,
        energy=energy,
        external_force=external_force,
        initial_state=initial_state,
        optimizer=optimizer,
        surface_map=surface_map,
    )
    return VolumeIPCSimulationRunner(spec, build, simulation)


def _default_volume_formulation(volume: VolumeMesh):
    element_width = int(volume.mesh_data.elements.shape[1])
    if element_width == 4:
        return _fem.TetP1()
    if element_width == 8:
        return _fem.LinearCubic()
    raise ValueError(f"unsupported volume element width: {element_width}")


def _surface_embedding_matrix(volume: VolumeMesh, surface: TriMeshData, formulation):
    return formulation.surface_embedding_matrix(volume, surface.vertices)


def _initial_vector(volume: VolumeMesh, num_dofs: int, values, formulation) -> np.ndarray:
    vec = vec3_finite("initial vector", values)
    if num_dofs == volume.num_vertices * 3:
        return np.tile(vec, volume.num_vertices).astype(np.float64, copy=False)
    out = np.zeros(num_dofs, dtype=np.float64)
    if isinstance(formulation, _fem.TricubicHermite):
        for vertex_id in range(volume.num_vertices):
            out[vertex_id * 24:vertex_id * 24 + 3] = vec
    return out


__all__ = [
    "FloorSpec",
    "IPCContactSpec",
    "RuntimeConfig",
    "VolumeIPCSimulationBuild",
    "VolumeIPCSimulationRunner",
    "VolumeIPCSimulationSpec",
    "build_volume_ipc_simulation",
]
