"""Typed configuration for the pypgo-sim CLI family.

Precedence: built-in defaults < JSON config < CLI overrides.
Paths inside a JSON file resolve relative to the JSON file's directory;
override paths are taken as-is (the CLI layer resolves them against CWD).
"""

from __future__ import annotations

import copy
import json
import sys
from dataclasses import dataclass
from pathlib import Path

MESH_TYPES = ("tet", "cubic", "shell")
VOLUME_FORMULATIONS = ("auto", "tet-linear", "cubic-linear", "cubic-tricubic-hermite")
VOLUME_ELASTIC_MODELS = ("stable_neo", "stvk", "stvk_volume", "linear_elastic", "mooney_rivlin")
CONTACT_MODELS = ("ipc", "floor", "sampled_penalty")
INTEGRATORS = ("implicit_euler", "trbdf2")
LINE_SEARCH_METHODS = ("backtrack", "simple", "golden", "brents")


class ConfigError(ValueError):
    """Invalid or inconsistent sim configuration."""


# ---------------------------------------------------------------------------
# Vertex selectors
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RegionSelector:
    """Select vertices whose coordinate along `axis` is within `tolerance`
    of the mesh min/max along that axis."""

    axis: str
    side: str
    tolerance: float = 1e-6

    def __post_init__(self):
        if self.axis not in ("x", "y", "z"):
            raise ConfigError(f"region axis must be x/y/z, got {self.axis!r}")
        if self.side not in ("min", "max"):
            raise ConfigError(f"region side must be min/max, got {self.side!r}")
        if not float(self.tolerance) >= 0.0:
            raise ConfigError("region tolerance must be non-negative")


@dataclass(frozen=True)
class VertexSelector:
    """Exactly one of file / indices / region."""

    file: Path | None = None
    indices: tuple[int, ...] | None = None
    region: RegionSelector | None = None

    def __post_init__(self):
        sources = [s for s in (self.file, self.indices, self.region) if s is not None]
        if len(sources) != 1:
            raise ConfigError(
                "vertex selector needs exactly one of file / indices / region")


def _selector_from_payload(payload, label: str) -> VertexSelector:
    if not isinstance(payload, dict):
        raise ConfigError(f"{label} must be an object")
    file = payload.get("file")
    indices = payload.get("indices")
    region = payload.get("region")
    region_selector: RegionSelector | None = None
    if region is not None:
        if not isinstance(region, dict) or "axis" not in region or "side" not in region:
            raise ConfigError(f"{label}.region needs 'axis' and 'side'")
        region_selector = RegionSelector(
            axis=region["axis"],
            side=region["side"],
            tolerance=float(region.get("tolerance", 1e-6)),
        )
    return VertexSelector(
        file=Path(file) if file is not None else None,
        indices=tuple(int(i) for i in indices) if indices is not None else None,
        region=region_selector,
    )


# ---------------------------------------------------------------------------
# Section dataclasses
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MeshConfig:
    volume: Path | None = None
    surface: Path | None = None
    formulation: str = "auto"


@dataclass(frozen=True)
class VolumeMaterialConfig:
    model: str = "stable_neo"
    density: float | None = None  # None -> per-region densities from the veg file
    enable_material_max_step: bool = True


@dataclass(frozen=True)
class ShellMassConfig:
    areal_density: float | None = None  # kg/m^2
    density: float | None = None        # kg/m^3, combined with material thickness


@dataclass(frozen=True)
class ShellMaterialConfig:
    thickness: float = 1e-3
    E_membrane: float = 1e6
    nu_membrane: float = 0.4
    mass: ShellMassConfig = ShellMassConfig(density=1000.0)
    enable_material_max_step: bool = True


@dataclass(frozen=True)
class AttachmentConfig:
    vertices: VertexSelector
    coeff: float = 1e5
    movement: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class SurfaceAttachmentConfig:
    """Soft pin on EMBEDDED SURFACE vertices: coeff * ||(W u)_i||^2.

    Defined on the shared surface mesh, so the constraint is identical across
    formulations (incl. tricubic Hermite) — no cross-simulation-mesh mapping.
    """

    vertices: VertexSelector
    coeff: float = 1e5


@dataclass(frozen=True)
class ConstraintsConfig:
    fixed: VertexSelector | None = None
    attachments: tuple[AttachmentConfig, ...] = ()
    surface_attachments: tuple[SurfaceAttachmentConfig, ...] = ()


@dataclass(frozen=True)
class LoadsConfig:
    gravity: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ObstacleConfig:
    mesh: Path
    velocity: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class ContactConfig:
    model: str
    # ipc
    dhat: float = 1e-3
    dhat_external: float | None = None
    kappa: float = 1000.0
    obstacles: tuple[ObstacleConfig, ...] = ()
    # floor
    axis: str = "z"
    side: str = "keep_above"
    height: float = 0.0
    # floor / sampled penalty
    stiffness: float = 1.0
    # sampled penalty
    samples: int = 1
    enable_self_contact: bool = True
    enable_external_contact: bool = True
    # friction
    friction_coeff: float = 0.0
    velocity_eps: float = 1e-4


@dataclass(frozen=True)
class InitialStateConfig:
    displacement: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class SolverConfig:
    max_iterations: int = 50
    gradient_tolerance: float = 1e-6
    verbose: int = 0
    damping_scale: float | None = None
    line_search: str = "simple"
    line_search_max_iterations: int = 100
    line_search_shrink: float = 0.5
    line_search_armijo_c: float = 1e-4
    line_search_initial_alpha: float = 1.0
    sparse_solver: str = "auto"


@dataclass(frozen=True)
class DynamicConfig:
    timestep: float | None = None
    num_steps: int = 1
    integrator: str = "implicit_euler"
    damping: tuple[float, float] = (0.0, 0.0)
    resume: str | Path | None = None


@dataclass(frozen=True)
class OutputConfig:
    directory: Path | None = None
    write_surfaces: bool = False
    write_states: bool = False
    write_checkpoints: bool = False
    write_stress: bool = False
    write_abc: bool = False
    dump_interval: int = 1


@dataclass(frozen=True)
class SimConfig:
    mesh_type: str
    mode: str
    mesh: MeshConfig
    material: VolumeMaterialConfig | ShellMaterialConfig
    constraints: ConstraintsConfig
    loads: LoadsConfig
    contact: tuple[ContactConfig, ...]
    initial_state: InitialStateConfig
    solver: SolverConfig
    dynamic: DynamicConfig
    output: OutputConfig


# ---------------------------------------------------------------------------
# JSON loading and merging
# ---------------------------------------------------------------------------


def _resolve(base_dir: Path, value) -> str:
    p = Path(value)
    return str(p if p.is_absolute() else (base_dir / p))


def _resolve_json_paths(payload: dict, base_dir: Path) -> dict:
    """Rewrite path-valued JSON fields to be absolute w.r.t. the JSON dir."""
    mesh = payload.get("mesh", {})
    for key in ("volume", "surface"):
        if mesh.get(key) is not None:
            mesh[key] = _resolve(base_dir, mesh[key])
    constraints = payload.get("constraints", {})
    fixed = constraints.get("fixed")
    if fixed and fixed.get("file") is not None:
        fixed["file"] = _resolve(base_dir, fixed["file"])
    for group in ("attachments", "surface_attachments"):
        for att in constraints.get(group, []):
            sel = att.get("vertices", {})
            if sel.get("file") is not None:
                sel["file"] = _resolve(base_dir, sel["file"])
    for contact in payload.get("contact", []):
        for obs in contact.get("obstacles", []):
            if obs.get("mesh") is not None:
                obs["mesh"] = _resolve(base_dir, obs["mesh"])
    output = payload.get("output", {})
    if output.get("directory") is not None:
        output["directory"] = _resolve(base_dir, output["directory"])
    dynamic = payload.get("dynamic", {})
    if dynamic.get("resume") is not None and dynamic["resume"] != "latest":
        dynamic["resume"] = _resolve(base_dir, dynamic["resume"])
    return payload


def _set_dotted(payload: dict, dotted: str, value) -> None:
    keys = dotted.split(".")
    cur = payload
    for key in keys[:-1]:
        cur = cur.setdefault(key, {})
    cur[keys[-1]] = value


def _vec3(value, label: str) -> tuple[float, float, float]:
    vec = tuple(float(v) for v in value)
    if len(vec) != 3:
        raise ConfigError(f"{label} must be a 3-vector, got {value!r}")
    return vec


def _build_contact(payload, label: str) -> ContactConfig:
    if not isinstance(payload, dict) or "model" not in payload:
        raise ConfigError(f"{label}: each contact entry needs a 'model'")
    model = payload["model"]
    if model not in CONTACT_MODELS:
        raise ConfigError(
            f"{label}: contact model must be one of {CONTACT_MODELS}, got {model!r}")
    if model != "sampled_penalty" and ("friction_coeff" in payload or "velocity_eps" in payload):
        raise ConfigError(f"{label}: friction fields are only supported for sampled_penalty contact")
    obstacles = tuple(
        ObstacleConfig(
            mesh=Path(obs["mesh"]),
            velocity=_vec3(obs["velocity"], f"{label}.obstacles.velocity")
            if obs.get("velocity") is not None else None,
        )
        for obs in payload.get("obstacles", [])
    )
    if obstacles and model not in ("ipc", "sampled_penalty"):
        raise ConfigError(f"{label}: obstacles are only supported for ipc or sampled_penalty contact")
    if model == "sampled_penalty" and any(obs.velocity is not None for obs in obstacles):
        raise ConfigError(f"{label}: sampled_penalty contact only supports static obstacles; moving obstacles require ipc")
    if model == "floor":
        floor_axis = payload.get("axis", "z")
        floor_side = payload.get("side", "keep_above")
        if floor_axis not in ("x", "y", "z"):
            raise ConfigError(
                f"{label}: floor axis must be x/y/z, got {floor_axis!r}")
        if floor_side not in ("keep_above", "keep_below"):
            raise ConfigError(
                f"{label}: floor side must be keep_above/keep_below, got {floor_side!r}")
    return ContactConfig(
        model=model,
        dhat=float(payload.get("dhat", 1e-3)),
        dhat_external=float(payload["dhat_external"])
        if payload.get("dhat_external") is not None else None,
        kappa=float(payload.get("kappa", 1000.0)),
        obstacles=obstacles,
        axis=payload.get("axis", "z"),
        side=payload.get("side", "keep_above"),
        height=float(payload.get("height", 0.0)),
        stiffness=float(payload.get("stiffness", 1.0)),
        samples=int(payload.get("samples", 1)),
        enable_self_contact=bool(payload.get("enable_self_contact", True)),
        enable_external_contact=bool(payload.get("enable_external_contact", True)),
        friction_coeff=float(payload.get("friction_coeff", 0.0)),
        velocity_eps=float(payload.get("velocity_eps", 1e-4)),
    )


def _build_attachment(att: dict) -> AttachmentConfig:
    if "vertices" not in att:
        raise ConfigError("constraints.attachments entries need 'vertices'")
    movement = None
    if att.get("movement") is not None:
        movement = _vec3(att["movement"], "constraints.attachments.movement")
    return AttachmentConfig(
        vertices=_selector_from_payload(att["vertices"], "constraints.attachments.vertices"),
        coeff=float(att.get("coeff", 1e5)),
        movement=movement,
    )


def _build_surface_attachment(att: dict) -> SurfaceAttachmentConfig:
    if "vertices" not in att:
        raise ConfigError("constraints.surface_attachments entries need 'vertices'")
    return SurfaceAttachmentConfig(
        vertices=_selector_from_payload(
            att["vertices"], "constraints.surface_attachments.vertices"),
        coeff=float(att.get("coeff", 1e5)),
    )


def load_config(*, mesh_type: str, mode: str, json_path=None,
                overrides: dict | None = None) -> SimConfig:
    if mesh_type not in MESH_TYPES:
        raise ConfigError(f"mesh_type must be one of {MESH_TYPES}, got {mesh_type!r}")
    if mode not in ("static", "dynamic"):
        raise ConfigError(f"mode must be 'static' or 'dynamic', got {mode!r}")

    payload: dict = {}
    if json_path is not None:
        json_path = Path(json_path)
        try:
            payload = copy.deepcopy(json.loads(json_path.read_text()))
        except (OSError, json.JSONDecodeError) as exc:
            raise ConfigError(f"cannot read config {json_path}: {exc}") from exc
        if not isinstance(payload, dict):
            raise ConfigError(f"config root must be a JSON object: {json_path}")
        payload = _resolve_json_paths(payload, json_path.parent)

    for dotted, value in (overrides or {}).items():
        _set_dotted(payload, dotted, value)

    declared = payload.get("type")
    if declared is not None and declared != mesh_type:
        raise ConfigError(
            f"config 'type' is {declared!r} but this command runs mesh type {mesh_type!r}")

    mesh_payload = payload.get("mesh", {})
    mesh = MeshConfig(
        volume=Path(mesh_payload["volume"]) if mesh_payload.get("volume") else None,
        surface=Path(mesh_payload["surface"]) if mesh_payload.get("surface") else None,
        formulation=mesh_payload.get("formulation", "auto"),
    )
    if mesh_type == "shell":
        if mesh.volume is not None:
            raise ConfigError("shell meshes take mesh.surface only, not mesh.volume")
        if mesh.surface is None:
            raise ConfigError("mesh.surface is required for shell")
        if mesh.formulation != "auto":
            raise ConfigError("mesh.formulation is not configurable for shell")
    else:
        if mesh.volume is None:
            raise ConfigError(f"mesh.volume is required for {mesh_type}")
        if mesh.surface is None:
            raise ConfigError(f"mesh.surface is required for {mesh_type}")
        if mesh.formulation not in VOLUME_FORMULATIONS:
            raise ConfigError(
                f"mesh.formulation must be one of {VOLUME_FORMULATIONS}, "
                f"got {mesh.formulation!r}")
        if mesh.formulation == "cubic-tricubic-hermite" and mesh_type != "cubic":
            raise ConfigError("formulation cubic-tricubic-hermite requires mesh type cubic")
        if mesh.formulation == "tet-linear" and mesh_type != "tet":
            raise ConfigError("formulation tet-linear requires mesh type tet")
        if mesh.formulation == "cubic-linear" and mesh_type != "cubic":
            raise ConfigError("formulation cubic-linear requires mesh type cubic")

    mat_payload = payload.get("material", {})
    if mesh_type == "shell":
        mass_payload = mat_payload.get("mass", {"density": 1000.0})
        mass = ShellMassConfig(
            areal_density=float(mass_payload["areal_density"])
            if mass_payload.get("areal_density") is not None else None,
            density=float(mass_payload["density"])
            if mass_payload.get("density") is not None else None,
        )
        if (mass.areal_density is None) == (mass.density is None):
            raise ConfigError(
                "shell material.mass needs exactly one of areal_density / density")
        material: VolumeMaterialConfig | ShellMaterialConfig = ShellMaterialConfig(
            thickness=float(mat_payload.get("thickness", 1e-3)),
            E_membrane=float(mat_payload.get("E_membrane", 1e6)),
            nu_membrane=float(mat_payload.get("nu_membrane", 0.4)),
            mass=mass,
            enable_material_max_step=bool(mat_payload.get("enable_material_max_step", True)),
        )
    else:
        material = VolumeMaterialConfig(
            model=mat_payload.get("model", "stable_neo"),
            density=float(mat_payload["density"])
            if mat_payload.get("density") is not None else None,
            enable_material_max_step=bool(mat_payload.get("enable_material_max_step", True)),
        )
        if material.model not in VOLUME_ELASTIC_MODELS:
            raise ConfigError(
                f"material.model must be one of {VOLUME_ELASTIC_MODELS}, "
                f"got {material.model!r}")

    cons_payload = payload.get("constraints", {})
    constraints = ConstraintsConfig(
        fixed=_selector_from_payload(cons_payload["fixed"], "constraints.fixed")
        if cons_payload.get("fixed") is not None else None,
        attachments=tuple(
            _build_attachment(att)
            for att in cons_payload.get("attachments", [])
        ),
        surface_attachments=tuple(
            _build_surface_attachment(att)
            for att in cons_payload.get("surface_attachments", [])
        ),
    )

    loads = LoadsConfig(
        gravity=_vec3(payload.get("loads", {}).get("gravity", (0.0, 0.0, 0.0)),
                      "loads.gravity"))

    contact = tuple(
        _build_contact(c, f"contact[{i}]")
        for i, c in enumerate(payload.get("contact", []))
    )

    if mode == "static" and any(
        att.movement is not None and any(v != 0.0 for v in att.movement)
        for att in constraints.attachments
    ):
        raise ConfigError(
            "attachment movement requires dynamic mode "
            "(movement targets only make sense with time stepping)")

    if mode == "static" and any(
        c.model == "sampled_penalty" and c.friction_coeff > 0.0 for c in contact
    ):
        raise ConfigError("sampled_penalty contact with friction requires dynamic mode")

    dyn_payload = payload.get("dynamic", {})
    if mode == "static" and dyn_payload.get("resume") is not None:
        raise ConfigError("dynamic.resume requires dynamic mode")

    init_payload = payload.get("initial_state", {})
    initial_state = InitialStateConfig(
        displacement=_vec3(init_payload.get("displacement", (0.0, 0.0, 0.0)),
                           "initial_state.displacement"),
        velocity=_vec3(init_payload.get("velocity", (0.0, 0.0, 0.0)),
                       "initial_state.velocity"),
    )

    solver_payload = payload.get("solver", {})
    line_search = solver_payload.get("line_search", "simple")
    if line_search not in LINE_SEARCH_METHODS:
        raise ConfigError(
            f"solver.line_search must be one of {LINE_SEARCH_METHODS}, "
            f"got {line_search!r}")
    damping_raw = solver_payload.get("damping", None)
    damping_scale_raw = solver_payload.get("damping_scale", None)
    if damping_raw is not None:
        damping_enabled = bool(damping_raw)
        if damping_enabled:
            damping_scale = 1.0 if damping_scale_raw is None else float(damping_scale_raw)
        else:
            damping_scale = None
    else:
        damping_scale = None if damping_scale_raw is None else float(damping_scale_raw)

    solver = SolverConfig(
        max_iterations=int(solver_payload.get("max_iterations", 50)),
        gradient_tolerance=float(solver_payload.get("gradient_tolerance", 1e-6)),
        verbose=int(solver_payload.get("verbose", 0)),
        damping_scale=damping_scale,
        line_search=line_search,
        line_search_max_iterations=int(
            solver_payload.get("line_search_max_iterations", 100)),
        line_search_shrink=float(solver_payload.get("line_search_shrink", 0.5)),
        line_search_armijo_c=float(solver_payload.get("line_search_armijo_c", 1e-4)),
        line_search_initial_alpha=float(
            solver_payload.get("line_search_initial_alpha", 1.0)),
        sparse_solver=solver_payload.get("sparse_solver", "auto"),
    )

    dyn_payload = payload.get("dynamic", {})
    resume_raw = dyn_payload.get("resume")
    resume = None
    if resume_raw is not None:
        resume = "latest" if resume_raw == "latest" else Path(resume_raw)
    dynamic = DynamicConfig(
        timestep=float(dyn_payload["timestep"])
        if dyn_payload.get("timestep") is not None else None,
        num_steps=int(dyn_payload.get("num_steps", 1)),
        integrator=dyn_payload.get("integrator", "implicit_euler"),
        damping=tuple(float(v) for v in dyn_payload.get("damping", (0.0, 0.0))),
        resume=resume,
    )
    if len(dynamic.damping) != 2:
        raise ConfigError("dynamic.damping must be a [mass, stiffness] pair")
    if mode == "dynamic":
        if dynamic.timestep is None or not dynamic.timestep > 0.0:
            raise ConfigError("dynamic.timestep is required and must be positive")
        if dynamic.integrator not in INTEGRATORS:
            raise ConfigError(
                f"dynamic.integrator must be one of {INTEGRATORS}, "
                f"got {dynamic.integrator!r}")
        if dynamic.num_steps < 0:
            raise ConfigError("dynamic.num_steps must be non-negative")
    elif dyn_payload:
        print("warning: 'dynamic' section is ignored in static mode", file=sys.stderr)

    out_payload = payload.get("output", {})
    dump_interval = int(out_payload.get("dump_interval", 1))
    if dump_interval < 1:
        raise ConfigError(
            f"output.dump_interval must be >= 1, got {dump_interval}")
    write_stress = bool(out_payload.get("write_stress", False))
    output = OutputConfig(
        directory=Path(out_payload["directory"])
        if out_payload.get("directory") else None,
        write_surfaces=bool(out_payload.get("write_surfaces", False)),
        write_states=bool(out_payload.get("write_states", False)),
        write_checkpoints=bool(out_payload.get("write_checkpoints", False)),
        write_stress=write_stress,
        write_abc=bool(out_payload.get("write_abc", False)),
        dump_interval=dump_interval,
    )
    if output.directory is None:
        raise ConfigError("output.directory is required (JSON or --output-dir)")

    return SimConfig(
        mesh_type=mesh_type, mode=mode, mesh=mesh, material=material,
        constraints=constraints, loads=loads, contact=contact,
        initial_state=initial_state, solver=solver, dynamic=dynamic, output=output,
    )
