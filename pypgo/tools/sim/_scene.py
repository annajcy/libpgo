"""Scene construction for the pypgo-sim CLI family.

Builds a mesh-type-independent SceneBundle from a SimConfig; the runners in
_runners.py never branch on mesh type.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

import pypgo.contact as _contact
import pypgo.energy as _energy
import pypgo.fem as _fem
from pypgo.mesh import read_obj
from pypgo.mesh.volume import VolumeMesh, read_veg
from pypgo.tools.sim._config import ConfigError, SimConfig, VertexSelector

_VOLUME_ELASTIC = {
    "stable_neo": _fem.StableNeo,
    "stvk": _fem.StVK,
    "stvk_volume": _fem.StVKVolume,
    "linear_elastic": _fem.LinearElastic,
    "mooney_rivlin": _fem.MooneyRivlin,
}

_AXIS = {"x": 0, "y": 1, "z": 2}


def resolve_vertex_selector(selector: VertexSelector, vertices: np.ndarray) -> np.ndarray:
    """Resolve a selector to sorted unique vertex indices, validating range."""
    if selector.file is not None:
        try:
            tokens = Path(selector.file).read_text().split()
        except OSError as exc:
            raise ConfigError(f"cannot read vertex file {selector.file}: {exc}") from exc
        try:
            idx = np.array([int(t) for t in tokens], dtype=np.int64)
        except ValueError as exc:
            raise ConfigError(
                f"non-integer token in vertex file {selector.file}: {exc}") from exc
    elif selector.indices is not None:
        idx = np.asarray(selector.indices, dtype=np.int64)
    else:
        region = selector.region
        coords = np.asarray(vertices, dtype=np.float64)[:, _AXIS[region.axis]]
        bound = coords.min() if region.side == "min" else coords.max()
        idx = np.flatnonzero(np.abs(coords - bound) <= region.tolerance)
    idx = np.unique(idx)
    if idx.size == 0:
        raise ConfigError("vertex selector matched no vertices")
    if idx[0] < 0 or idx[-1] >= len(vertices):
        raise ConfigError(
            f"vertex selector index out of range: [{idx[0]}, {idx[-1]}] "
            f"for {len(vertices)} vertices")
    return idx


@dataclass
class SceneBundle:
    """Everything a runner needs, independent of mesh type."""

    sim_mesh: object
    formulation: object
    deformation: object
    attachment_energies: list
    contact_energies: list
    stateful_contacts: list  # need begin_step(time, timestep, previous_x)
    ipc_contacts: list       # need set_moving_obstacle_time when obstacles exist
    mass: object
    gravity_force: np.ndarray
    fixed_dofs: np.ndarray | None
    num_dofs: int
    dofs_per_vertex: int
    surface_rest: np.ndarray       # (m, 3) rest surface vertices
    surface_triangles: np.ndarray  # (k, 3) surface triangles
    surface_map: object | None     # SparseMatrix sim-DOFs -> surface DOFs, None = identity

    def surface_positions(self, u: np.ndarray) -> np.ndarray:
        if self.surface_map is None:
            return self.surface_rest + np.asarray(u, dtype=np.float64).reshape((-1, 3))
        mapped = np.asarray(self.surface_map @ np.asarray(u, dtype=np.float64),
                            dtype=np.float64).reshape((-1, 3))
        return self.surface_rest + mapped

    def initial_vector(self, values) -> np.ndarray:
        vec = np.asarray(values, dtype=np.float64).reshape(3)
        num_vertices = self.num_dofs // self.dofs_per_vertex
        if self.dofs_per_vertex == 3:
            return np.tile(vec, num_vertices)
        out = np.zeros(self.num_dofs, dtype=np.float64)
        for v in range(num_vertices):
            out[v * self.dofs_per_vertex: v * self.dofs_per_vertex + 3] = vec
        return out

    def weighted_energies(self, *, include_gravity_potential: bool) -> list:
        terms = [(self.deformation, 1.0)]
        terms += [(e, 1.0) for e in self.attachment_energies]
        terms += [(e, 1.0) for e in self.contact_energies]
        if include_gravity_potential and float(np.linalg.norm(self.gravity_force)) > 0.0:
            terms.append((_energy.LinearEnergy(-self.gravity_force), 1.0))
        return terms


def _fixed_dofs_from_selector(selector, vertices, dofs_per_vertex) -> np.ndarray:
    idx = resolve_vertex_selector(selector, vertices)
    return (idx[:, None] * dofs_per_vertex + np.arange(dofs_per_vertex, dtype=np.int64)).ravel()


def _build_contact_energies(contact_cfgs, contact_surface, surface_triangles):
    energies, stateful, ipcs = [], [], []
    for cfg in contact_cfgs:
        if cfg.model == "floor":
            e = _contact.FloorEnergy(
                contact_surface,
                axis=cfg.axis, side=cfg.side,
                height=cfg.height, stiffness=cfg.stiffness,
            )
        elif cfg.model == "ipc":
            obstacles = []
            for obs in cfg.obstacles:
                mesh = read_obj(str(obs.mesh))
                if obs.velocity is None or not any(obs.velocity):
                    obstacles.append(
                        _contact.ObstacleSpec.static(mesh.vertices, mesh.elements))
                else:
                    obstacles.append(_contact.ObstacleSpec.linear_velocity(
                        mesh.vertices, mesh.elements, np.asarray(obs.velocity)))
            e = _contact.IPCEnergy(
                contact_surface, surface_triangles,
                params=_contact.IPCParameters(
                    dhat=cfg.dhat, dhat_external=cfg.dhat_external, kappa=cfg.kappa),
                obstacles=obstacles,
            )
            stateful.append(e)
            ipcs.append(e)
        elif cfg.model == "sampled_penalty":
            e = _contact.SampledPenaltyEnergy(
                contact_surface, surface_triangles,
                params=_contact.SampledPenaltyParameters(
                    stiffness=cfg.stiffness, samples=cfg.samples,
                    enable_self_contact=cfg.enable_self_contact,
                    enable_external_contact=cfg.enable_external_contact),
            )
            stateful.append(e)
        else:  # frictional_sampled_penalty — validated in _config
            e = _contact.FrictionalSampledPenaltyEnergy(
                contact_surface, surface_triangles,
                params=_contact.SampledPenaltyParameters(
                    stiffness=cfg.stiffness, samples=cfg.samples,
                    enable_self_contact=cfg.enable_self_contact,
                    enable_external_contact=cfg.enable_external_contact),
                friction=_contact.FrictionParameters(
                    friction_coeff=cfg.friction_coeff,
                    velocity_eps=cfg.velocity_eps),
            )
            stateful.append(e)
        energies.append(e)
    return energies, stateful, ipcs


def _volume_formulation(cfg: SimConfig, volume: VolumeMesh):
    element_width = int(volume.mesh_data.elements.shape[1])
    expected_width = 4 if cfg.mesh_type == "tet" else 8
    if element_width != expected_width:
        raise ConfigError(
            f"{cfg.mesh_type} command expects {expected_width}-vertex elements, "
            f"but {cfg.mesh.volume} has {element_width}-vertex elements")
    if cfg.mesh.formulation in ("auto", "tet-linear") and cfg.mesh_type == "tet":
        return _fem.TetLinear()
    if cfg.mesh.formulation in ("auto", "cubic-linear"):
        return _fem.CubicLinear()
    return _fem.CubicTricubicHermite()


def _build_volume_scene(cfg: SimConfig) -> SceneBundle:
    volume = VolumeMesh.from_veg_file(read_veg(str(cfg.mesh.volume)))
    surface = read_obj(str(cfg.mesh.surface))
    fm = _volume_formulation(cfg, volume)

    sim_mesh = _fem.SimulationMesh.create_volumetric(volume)
    deformation = _fem.deformation_energy(
        sim_mesh,
        elastic=_VOLUME_ELASTIC[cfg.material.model](),
        elastic_field=_fem.ElementwiseField(),
        plastic=_fem.VolumetricPlasticity(dofs=0),
        plastic_field=_fem.ElementwiseField(),
        formulation=fm,
    )
    num_dofs = deformation.num_dofs
    dofs_per_vertex = num_dofs // volume.num_vertices

    mass_field = (
        _fem.VolumeDensity(cfg.material.density)
        if cfg.material.density is not None
        else _fem.volume_density(volume)
    )
    mass = fm.mass_matrix(sim_mesh, mass_field)
    gravity = np.asarray(cfg.loads.gravity, dtype=np.float64)
    gravity_force = (
        fm.body_force(sim_mesh, gravity, mass_field)
        if float(np.linalg.norm(gravity)) > 0.0
        else np.zeros(num_dofs, dtype=np.float64)
    )

    surface_map = fm.surface_embedding_matrix(volume, surface.vertices)
    contact_surface = _contact.ContactSurface.embedded(surface.vertices, surface_map)
    contact_energies, stateful, ipcs = _build_contact_energies(
        cfg.contact, contact_surface, surface.elements)

    rest_vertices = np.asarray(volume.mesh_data.vertices, dtype=np.float64)
    fixed_dofs = None
    if cfg.constraints.fixed is not None:
        fixed_dofs = _fixed_dofs_from_selector(
            cfg.constraints.fixed, rest_vertices, dofs_per_vertex)

    attachments = []
    for att in cfg.constraints.attachments:
        if dofs_per_vertex != 3:
            raise ConfigError(
                "attachments are not supported for the tricubic Hermite "
                "formulation (24 DOFs per vertex)")
        idx = resolve_vertex_selector(att.vertices, rest_vertices)
        attachments.append(_energy.VertexAttachment(
            sim_mesh=sim_mesh,
            vertex_indices=idx,
            target_positions=np.zeros(3 * idx.size, dtype=np.float64),
            coeff=att.coeff,
            is_displacement=True,
        ))

    return SceneBundle(
        sim_mesh=sim_mesh, formulation=fm, deformation=deformation,
        attachment_energies=attachments, contact_energies=contact_energies,
        stateful_contacts=stateful, ipc_contacts=ipcs,
        mass=mass, gravity_force=np.asarray(gravity_force, dtype=np.float64),
        fixed_dofs=fixed_dofs, num_dofs=num_dofs, dofs_per_vertex=dofs_per_vertex,
        surface_rest=np.asarray(surface.vertices, dtype=np.float64),
        surface_triangles=np.asarray(surface.elements),
        surface_map=surface_map,
    )


def _build_shell_scene(cfg: SimConfig) -> SceneBundle:
    surface = read_obj(str(cfg.mesh.surface))
    material = _fem.KoiterStVKShellMaterial(
        thickness=cfg.material.thickness,
        E_membrane=cfg.material.E_membrane,
        nu_membrane=cfg.material.nu_membrane,
    )
    sim_mesh = _fem.SimulationMesh.create_shell(surface, material)
    fm = _fem.KoiterShell()
    deformation = _fem.deformation_energy(
        sim_mesh,
        elastic=_fem.KoiterStVK(),
        elastic_field=_fem.ElementwiseField(),
        plastic=_fem.ShellPlasticity(dofs=0),
        plastic_field=_fem.ElementwiseField(),
        formulation=fm,
    )
    num_dofs = deformation.num_dofs
    rest_vertices = np.asarray(surface.vertices, dtype=np.float64)

    if cfg.material.mass.areal_density is not None:
        mass_field = _fem.ShellArealDensity(cfg.material.mass.areal_density)
    else:
        mass_field = _fem.ShellDensityThickness(
            density=cfg.material.mass.density, thickness=cfg.material.thickness)
    mass = fm.mass_matrix(sim_mesh, mass_field)
    gravity = np.asarray(cfg.loads.gravity, dtype=np.float64)
    gravity_force = (
        fm.body_force(sim_mesh, gravity, mass_field)
        if float(np.linalg.norm(gravity)) > 0.0
        else np.zeros(num_dofs, dtype=np.float64)
    )

    contact_surface = _contact.ContactSurface.identity(surface.vertices)
    contact_energies, stateful, ipcs = _build_contact_energies(
        cfg.contact, contact_surface, surface.elements)

    fixed_dofs = None
    if cfg.constraints.fixed is not None:
        fixed_dofs = _fixed_dofs_from_selector(cfg.constraints.fixed, rest_vertices, 3)

    attachments = []
    for att in cfg.constraints.attachments:
        idx = resolve_vertex_selector(att.vertices, rest_vertices)
        attachments.append(_energy.VertexAttachment(
            sim_mesh=sim_mesh,
            vertex_indices=idx,
            target_positions=np.zeros(3 * idx.size, dtype=np.float64),
            coeff=att.coeff,
            is_displacement=True,
        ))

    return SceneBundle(
        sim_mesh=sim_mesh, formulation=fm, deformation=deformation,
        attachment_energies=attachments, contact_energies=contact_energies,
        stateful_contacts=stateful, ipc_contacts=ipcs,
        mass=mass, gravity_force=np.asarray(gravity_force, dtype=np.float64),
        fixed_dofs=fixed_dofs, num_dofs=num_dofs, dofs_per_vertex=3,
        surface_rest=rest_vertices,
        surface_triangles=np.asarray(surface.elements),
        surface_map=None,
    )


def build_scene(cfg: SimConfig) -> SceneBundle:
    if cfg.mesh_type == "shell":
        return _build_shell_scene(cfg)
    return _build_volume_scene(cfg)
