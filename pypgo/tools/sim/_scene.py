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

    asset: object
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
    simulation asset.
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


def _source_channel_values(asset, name):
    """Resolve one scalar channel from neutral imported data."""
    aliases = {
        "E_membrane": "E", "E_bending": "E",
        "nu_membrane": "nu", "nu_bending": "nu",
        "thickness": "h",
    }
    source_name = aliases.get(name, name)
    data = asset.material_data
    for field in data.fields:
        if source_name not in field.channel_names:
            continue
        channel = field.channel_names.index(source_name)
        rows = field.value_rows
        return np.asarray(
            [rows[int(field.element_to_row[e]), channel] for e in range(data.num_elements)],
            dtype=np.float64,
        )
    values = np.empty(data.num_elements, dtype=np.float64)
    for e, material_index in enumerate(data.element_material_indices):
        if material_index < 0:
            raise ConfigError(f"asset has no material assignment for element {e}")
        prop = data.materials[int(material_index)].properties
        if source_name not in prop:
            raise ConfigError(f"asset is missing imported material channel {name!r}")
        value = np.asarray(prop[source_name], dtype=np.float64)
        if value.size != 1:
            raise ConfigError(f"imported channel {name!r} is not scalar")
        values[e] = float(value.reshape(-1)[0])
    return values


def _build_material_state(asset, elastic, plastic, *, elastic_values=None, plastic_values=None,
                          constant=False):
    layout_type = _fem.ConstantParameterLayout if constant else _fem.ElementwiseParameterLayout

    def identity_field(field_type, names, selected_layout_type):
        count = len(names)
        return field_type(
            names,
            selected_layout_type(asset.num_elements, count),
            _fem.IdentityMaterialEvaluator(count),
        )

    elastic_fixed = _fem.FixedParameterField(
        elastic.fixed_channel_names,
        _fem.ElementwiseParameterLayout(
            asset.num_elements, len(elastic.fixed_channel_names)),
        _fem.IdentityMaterialEvaluator(len(elastic.fixed_channel_names)),
    )
    plastic_fixed = _fem.FixedParameterField(
        plastic.fixed_channel_names,
        _fem.ElementwiseParameterLayout(
            asset.num_elements, len(plastic.fixed_channel_names)),
        _fem.IdentityMaterialEvaluator(len(plastic.fixed_channel_names)),
    )
    elastic_optimizable = identity_field(
        _fem.OptimizableParameterField,
        elastic.optimizable_channel_names, layout_type)
    plastic_optimizable = identity_field(
        _fem.OptimizableParameterField,
        plastic.optimizable_channel_names, layout_type)
    parameterization = _fem.MaterialParameterization(
        _fem.ElasticParameterization(elastic, elastic_fixed, elastic_optimizable),
        _fem.PlasticParameterization(plastic, plastic_fixed, plastic_optimizable),
    )

    def fixed_values(definition, field):
        values = np.empty(field.num_global_parameters, dtype=np.float64)
        for channel, name in enumerate(definition.fixed_channel_names):
            source = _source_channel_values(asset, name)
            for element, value in enumerate(source):
                values[element * len(definition.fixed_channel_names) + channel] = value
        return values

    def initial_values(values, field):
        if values is None:
            if field.num_global_parameters:
                raise ConfigError(
                    f"material model requires explicit {field.num_global_parameters} optimizer values")
            return np.empty(0, dtype=np.float64)
        array = np.asarray(values, dtype=np.float64)
        return np.ascontiguousarray(array.reshape(-1))

    data = _fem.MaterialParameterData(
        elastic=(fixed_values(elastic, elastic_fixed),
                 initial_values(elastic_values, elastic_optimizable)),
        plastic=(fixed_values(plastic, plastic_fixed),
                 initial_values(plastic_values, plastic_optimizable)),
    )
    return parameterization, data


def _build_volume_scene(cfg: SimConfig) -> SceneBundle:
    volume = VolumeMesh.from_veg_file(_load_mesh(read_veg, cfg.mesh.volume, "volume mesh"))
    surface = _load_mesh(read_obj, cfg.mesh.surface, "surface mesh")
    fm = _volume_formulation(cfg, volume)

    asset = _fem.SimulationAsset.create_volumetric(volume)
    elastic = _VOLUME_ELASTIC[cfg.material.model]()
    plastic = _fem.VolumetricPlasticityDefinition(dofs=0)
    parameterization, parameter_data = _build_material_state(
        asset, elastic, plastic)
    assignment = _fem.MaterialAssignment(asset, parameterization, parameter_data)
    deformation = _fem.DeformationEnergy(
        assignment,
        formulation=fm,
        options=_fem.DeformationOptions(
            enable_material_max_step=cfg.material.enable_material_max_step),
    )
    num_dofs = deformation.num_dofs
    dofs_per_vertex = num_dofs // volume.num_vertices

    mass_field = (
        _fem.VolumeDensity(cfg.material.density)
        if cfg.material.density is not None
        else _fem.volume_density(volume)
    )
    mass = fm.mass_matrix(asset, mass_field) if cfg.mode == "dynamic" else None
    gravity = np.asarray(cfg.loads.gravity, dtype=np.float64)
    gravity_force = (
        fm.body_force(asset, gravity, mass_field)
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
            asset=asset,
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
        asset=asset, formulation=fm, deformation=deformation,
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
    material = _fem.KoiterStVKShellMaterial(
        thickness=cfg.material.thickness,
        E_membrane=cfg.material.E_membrane,
        nu_membrane=cfg.material.nu_membrane,
    )
    asset = _fem.SimulationAsset.create_shell(surface, material)
    fm = _fem.KoiterShell()
    elastic = _fem.KoiterStVKDefinition()
    plastic = _fem.ShellPlasticityDefinition(dofs=0)
    shell_values = np.array([
        material.E_membrane, material.nu_membrane,
        material.E_membrane, material.nu_membrane, material.thickness,
    ], dtype=np.float64)
    parameterization, parameter_data = _build_material_state(
        asset, elastic, plastic, elastic_values=shell_values, constant=True)
    assignment = _fem.MaterialAssignment(asset, parameterization, parameter_data)
    deformation = _fem.DeformationEnergy(
        assignment,
        formulation=fm,
        options=_fem.DeformationOptions(
            enable_material_max_step=cfg.material.enable_material_max_step),
    )
    num_dofs = deformation.num_dofs
    rest_vertices = np.asarray(surface.vertices, dtype=np.float64)

    if cfg.material.mass.areal_density is not None:
        areal_density = _fem.ShellArealDensity(cfg.material.mass.areal_density)
    else:
        areal_density = _fem.ShellArealDensity.from_density_thickness(
            density=cfg.material.mass.density, thickness=cfg.material.thickness)
    mass = fm.mass_matrix(asset, areal_density) if cfg.mode == "dynamic" else None
    gravity = np.asarray(cfg.loads.gravity, dtype=np.float64)
    gravity_force = (
        fm.body_force(asset, gravity, areal_density)
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
            asset=asset,
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
        asset=asset, formulation=fm, deformation=deformation,
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
