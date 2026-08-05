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

def _load_mesh(loader, path, kind):
    try:
        return loader(str(path))
    except (OSError, RuntimeError) as exc:
        raise ConfigError(f"cannot load {kind} {path}: {exc}") from exc


_VOLUME_ELASTIC = {
    "neo_hookean": _fem.NeoHookeanDefinition,
    "stable_neo": _fem.StableNeoDefinition,
    "stvk": _fem.StVKDefinition,
    "stvk_volume": _fem.StVKVolumeDefinition,
    "linear_elastic": _fem.LinearElasticDefinition,
    "mooney_rivlin": _fem.MooneyRivlinDefinition,
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
class MovingAttachment:
    """A VertexAttachment whose targets are updated each dynamic step."""

    energy: object        # VertexAttachment instance
    velocity: np.ndarray  # (3,) velocity vector in displacement/time units
    num_vertices: int     # number of vertices in the attachment group


@dataclass
class SceneBundle:
    """Everything a runner needs, independent of mesh type."""

    mesh: object
    formulation: object
    deformation: object
    attachment_energies: list
    contact_energies: list
    stateful_contacts: list  # need begin_step(time, timestep, previous_x)
    moving_attachments: list  # MovingAttachment entries updated each dynamic step
    mass: object | None
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


def _surface_attachment_energy(att, surface_rest, surface_map, num_dofs):
    """Soft pin on embedded surface vertices via pypgo.energy.EmbeddedVertexAttachment.

    Defined through the surface embedding, so it works for any formulation
    (incl. tricubic Hermite) and pins the same physical points on every
    simulation mesh.
    """
    idx = resolve_vertex_selector(att.vertices, surface_rest)
    return _energy.EmbeddedVertexAttachment(
        embedding=surface_map, vertex_indices=idx, coeff=att.coeff,
        num_dofs=num_dofs)


def _obstacle_specs(cfg):
    obstacles = []
    for obs in cfg.obstacles:
        mesh = _load_mesh(read_obj, obs.mesh, "obstacle mesh")
        if obs.velocity is None or not any(obs.velocity):
            obstacles.append(
                _contact.ObstacleSpec.static(mesh.vertices, mesh.elements))
        else:
            obstacles.append(_contact.ObstacleSpec.linear_velocity(
                mesh.vertices, mesh.elements, np.asarray(obs.velocity)))
    return obstacles


def _build_contact_energies(contact_cfgs, contact_surface, surface_triangles):
    energies, stateful = [], []
    for cfg in contact_cfgs:
        if cfg.model == "floor":
            e = _contact.FloorEnergy(
                contact_surface,
                axis=cfg.axis, side=cfg.side,
                height=cfg.height, stiffness=cfg.stiffness,
            )
        elif cfg.model == "ipc":
            e = _contact.IPCEnergy(
                contact_surface, surface_triangles,
                params=_contact.IPCParameters(
                    dhat=cfg.dhat, dhat_external=cfg.dhat_external, kappa=cfg.kappa),
                obstacles=_obstacle_specs(cfg),
            )
            stateful.append(e)
        elif cfg.model == "sampled_penalty":
            friction = None
            if cfg.friction_coeff > 0.0:
                friction = _contact.FrictionParameters(
                    friction_coeff=cfg.friction_coeff,
                    velocity_eps=cfg.velocity_eps)
            e = _contact.SampledPenaltyEnergy(
                contact_surface, surface_triangles,
                params=_contact.SampledPenaltyParameters(
                    stiffness=cfg.stiffness, samples=cfg.samples,
                    enable_self_contact=cfg.enable_self_contact,
                    enable_external_contact=cfg.enable_external_contact),
                friction=friction,
                obstacles=_obstacle_specs(cfg),
            )
            if e.is_step_dependent:
                stateful.append(e)
        energies.append(e)
    return energies, stateful


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


def _fixed_channel_order(definition):
    return {
        "stable_neo": ("E", "nu"),
        "neo_hookean": ("E", "nu"),
        "stvk": ("E", "nu"),
        "linear": ("E", "nu"),
        "stvk_vol": ("E", "nu", "J"),
        "mooney_rivlin": ("mu01", "mu10", "v1"),
    }.get(definition.name, ())


def _build_material(mesh, elastic, plastic, *, volume=None,
                    elastic_values=None, plastic_values=None, constant=False):
    def fixed_values(definition):
        names = _fixed_channel_order(definition)
        if len(names) != definition.num_fixed_channels:
            raise ConfigError(
                f"no Python fixed-channel order registered for {definition.name}")
        if not names:
            return np.empty((mesh.num_elements, 0), dtype=np.float64)
        if volume is None:
            raise ConfigError(
                "fixed material channels require imported material data")
        materials = volume.to_veg_file().materials
        assignments = volume.element_material_indices
        return np.asarray([
            [getattr(materials[int(assignments[element])], name)
             for name in names]
            for element in range(mesh.num_elements)
        ], dtype=np.float64)

    def initial_values(values, definition):
        channels = definition.num_optimizable_channels
        if values is None:
            if channels:
                raise ConfigError(
                    f"material model requires explicit elementwise optimizer values")
            return np.empty(0, dtype=np.float64)
        array = np.asarray(values, dtype=np.float64)
        if constant and array.size == channels:
            array = np.broadcast_to(array.reshape(1, channels),
                                    (mesh.num_elements, channels))
        if array.size != mesh.num_elements * channels:
            raise ConfigError(
                f"material state requires {mesh.num_elements * channels} values")
        return np.ascontiguousarray(array.reshape(-1))

    binding = _fem.MaterialBinding(
        elastic=_fem.ElasticMaterialBinding(
            elastic, mesh.num_elements, fixed_values(elastic),
        ),
        plastic=_fem.PlasticMaterialBinding(
            plastic, mesh.num_elements, fixed_values(plastic),
        ),
    )
    state = _fem.MaterialState(
        elastic_values=initial_values(elastic_values, elastic),
        plastic_values=initial_values(plastic_values, plastic),
    )
    return binding, state


def _build_volume_scene(cfg: SimConfig) -> SceneBundle:
    volume = VolumeMesh(_load_mesh(read_veg, cfg.mesh.volume, "volume mesh"))
    surface = _load_mesh(read_obj, cfg.mesh.surface, "surface mesh")
    fm = _volume_formulation(cfg, volume)

    mesh = _fem.SimulationMesh(volume)
    elastic = _VOLUME_ELASTIC[cfg.material.model]()
    plastic = _fem.VolumetricPlasticityDefinition(dofs=0)
    material_binding, material_state = _build_material(
        mesh, elastic, plastic,
        volume=volume)
    deformation_operator = _fem.DeformationEnergyOperator(
        mesh, material_binding,
        formulation=fm,
        options=_fem.DeformationOptions(
            enable_material_max_step=cfg.material.enable_material_max_step),
    )
    deformation = _fem.DeformationPotentialEnergy(
        deformation_operator, material_state)
    num_dofs = deformation.num_dofs
    dofs_per_vertex = num_dofs // volume.num_vertices

    element_densities = (
        cfg.material.density
        if cfg.material.density is not None
        else volume.element_densities
    )
    mass = fm.mass_matrix(mesh, element_densities) if cfg.mode == "dynamic" else None
    gravity = np.asarray(cfg.loads.gravity, dtype=np.float64)
    gravity_force = (
        fm.body_force(mesh, gravity, element_densities)
        if float(np.linalg.norm(gravity)) > 0.0
        else np.zeros(num_dofs, dtype=np.float64)
    )

    surface_map = fm.surface_embedding_matrix(volume, surface.vertices)
    contact_surface = _contact.ContactSurface.embedded(surface.vertices, surface_map)
    contact_energies, stateful = _build_contact_energies(
        cfg.contact, contact_surface, surface.elements)

    rest_vertices = np.asarray(volume.mesh_data.vertices, dtype=np.float64)
    fixed_dofs = None
    if cfg.constraints.fixed is not None:
        fixed_dofs = _fixed_dofs_from_selector(
            cfg.constraints.fixed, rest_vertices, dofs_per_vertex)

    attachments = []
    moving_attachments = []
    for att in cfg.constraints.attachments:
        if dofs_per_vertex != 3:
            raise ConfigError(
                "attachments are not supported for the tricubic Hermite "
                "formulation (24 DOFs per vertex)")
        idx = resolve_vertex_selector(att.vertices, rest_vertices)
        energy = _energy.VertexAttachment(
            mesh=mesh,
            vertex_indices=idx,
            target_positions=np.zeros(3 * idx.size, dtype=np.float64),
            coeff=att.coeff,
            is_displacement=True,
        )
        attachments.append(energy)
        if att.movement is not None and any(v != 0.0 for v in att.movement):
            moving_attachments.append(MovingAttachment(
                energy=energy,
                velocity=np.asarray(att.movement, dtype=np.float64),
                num_vertices=int(idx.size),
            ))

    surface_rest_arr = np.asarray(surface.vertices, dtype=np.float64)
    for satt in cfg.constraints.surface_attachments:
        attachments.append(_surface_attachment_energy(
            satt, surface_rest_arr, surface_map, num_dofs))

    return SceneBundle(
        mesh=mesh, formulation=fm, deformation=deformation,
        attachment_energies=attachments, contact_energies=contact_energies,
        stateful_contacts=stateful, moving_attachments=moving_attachments,
        mass=mass, gravity_force=np.asarray(gravity_force, dtype=np.float64),
        fixed_dofs=fixed_dofs, num_dofs=num_dofs, dofs_per_vertex=dofs_per_vertex,
        surface_rest=surface_rest_arr,
        surface_triangles=np.asarray(surface.elements),
        surface_map=surface_map,
    )


def _build_shell_scene(cfg: SimConfig) -> SceneBundle:
    surface = _load_mesh(read_obj, cfg.mesh.surface, "shell mesh")
    mesh = _fem.SimulationMesh(surface)
    fm = _fem.KoiterShell()
    elastic = _fem.KoiterStVKDefinition()
    plastic = _fem.ShellPlasticityDefinition(dofs=0)
    shell_values = np.array([
        cfg.material.E_membrane, cfg.material.nu_membrane,
        cfg.material.E_membrane, cfg.material.nu_membrane,
        cfg.material.thickness,
    ], dtype=np.float64)
    material_binding, material_state = _build_material(
        mesh, elastic, plastic, elastic_values=shell_values, constant=True)
    deformation_operator = _fem.DeformationEnergyOperator(
        mesh, material_binding,
        formulation=fm,
        options=_fem.DeformationOptions(
            enable_material_max_step=cfg.material.enable_material_max_step),
    )
    deformation = _fem.DeformationPotentialEnergy(
        deformation_operator, material_state)
    num_dofs = deformation.num_dofs
    rest_vertices = np.asarray(surface.vertices, dtype=np.float64)

    if cfg.material.mass.areal_density is not None:
        areal_density = cfg.material.mass.areal_density
    else:
        areal_density = cfg.material.mass.density * cfg.material.thickness
    mass = fm.mass_matrix(mesh, areal_density) if cfg.mode == "dynamic" else None
    gravity = np.asarray(cfg.loads.gravity, dtype=np.float64)
    gravity_force = (
        fm.body_force(mesh, gravity, areal_density)
        if float(np.linalg.norm(gravity)) > 0.0
        else np.zeros(num_dofs, dtype=np.float64)
    )

    contact_surface = _contact.ContactSurface.identity(rest_vertices)
    contact_energies, stateful = _build_contact_energies(
        cfg.contact, contact_surface, surface.elements)

    fixed_dofs = None
    if cfg.constraints.fixed is not None:
        fixed_dofs = _fixed_dofs_from_selector(cfg.constraints.fixed, rest_vertices, 3)

    attachments = []
    moving_attachments = []
    for att in cfg.constraints.attachments:
        # shell is always 3 DOFs/vertex; no Hermite guard needed here
        idx = resolve_vertex_selector(att.vertices, rest_vertices)
        energy = _energy.VertexAttachment(
            mesh=mesh,
            vertex_indices=idx,
            target_positions=np.zeros(3 * idx.size, dtype=np.float64),
            coeff=att.coeff,
            is_displacement=True,
        )
        attachments.append(energy)
        if att.movement is not None and any(v != 0.0 for v in att.movement):
            moving_attachments.append(MovingAttachment(
                energy=energy,
                velocity=np.asarray(att.movement, dtype=np.float64),
                num_vertices=int(idx.size),
            ))

    for satt in cfg.constraints.surface_attachments:
        attachments.append(_surface_attachment_energy(
            satt, rest_vertices, None, num_dofs))

    return SceneBundle(
        mesh=mesh, formulation=fm, deformation=deformation,
        attachment_energies=attachments, contact_energies=contact_energies,
        stateful_contacts=stateful, moving_attachments=moving_attachments,
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
