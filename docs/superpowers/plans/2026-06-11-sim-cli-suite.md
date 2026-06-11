# pypgo-sim CLI Suite Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Six thin sim CLIs (`pypgo-sim-{shell,cubic,tet}-{static,dynamic}`) sharing one config/scene/runner core, with six runnable JSON example configs under `examples/sim_configs/` backed only by `examples/assets/`, each covered by a smoke test.

**Architecture:** `pypgo/tools/sim/` gains `_config.py` (typed dataclasses, JSON load + dotted CLI overrides, precedence defaults < JSON < CLI, JSON-relative path resolution), `_scene.py` (vertex selectors + `build_volume_scene` / `build_shell_scene` returning a uniform `SceneBundle`), `_runners.py` (`run_static` / `run_dynamic`, zero mesh-type branches), `_outputs.py` (summary.json + surface OBJ writers), `_cli.py` (shared argparse + `run_cli`), and six ~10-line entry modules. Existing `volume_ipc.py` stays untouched (backward compat). All example scene parameters are copied from already-verified demo notebooks.

**Tech Stack:** Python 3.12, pypgo (`fem`/`contact`/`energy`/`solver`/`sim` packages), argparse, pytest.

**Key API facts (verified against the codebase, do not re-derive):**
- `ShellFormulation.mass_matrix(sim_mesh, mass_field)` / `.body_force(sim_mesh, accel3, mass_field)` exist (`pypgo/fem/formulations.py:99`); mass fields: `ShellArealDensity(areal)`, `ShellDensityThickness(density=..., thickness=...)` (`pypgo/fem/mass.py`).
- `VolumetricFormulation.mass_matrix/body_force(sim_mesh, ..., VolumeMassField)`; `volume_density(volume)` reads per-region density from veg; `VolumeDensity(scalar_or_array)` overrides.
- `DynamicSimulation(mass=, state=, timestep=, energy=, integrator=str|stepper, damping=(a,b), fixed_dofs=[...])`; `sim.step(external_force=, optimizer=)` (`pypgo/sim/simulation.py:49`).
- `ContactSurface.identity(rest_vertices)` for shells; `ContactSurface.embedded(surface_vertices, surface_map)` with `fm.surface_embedding_matrix(volume, surface.vertices)` for volumes (works for tricubic Hermite too — see `generate_tricubic_hermite_box_drop_ipc_demo.py:93`).
- Contact energies (`pypgo/contact/energies.py`): `FloorEnergy(surface, parameters=FloorParameters(axis, side, height, stiffness))`; `IPCEnergy(surface, triangles, params=IPCParameters(dhat, dhat_external, kappa), obstacles=[ObstacleSpec...])` with `.set_moving_obstacle_time(t)`; `SampledPenaltyEnergy(surface, triangles, params=SampledPenaltyParameters(stiffness, samples, enable_self_contact, enable_external_contact))`; `FrictionalSampledPenaltyEnergy(..., friction=FrictionParameters(friction_coeff, velocity_eps))`. Stateful ones get exactly ONE `begin_step(time=0.0, timestep=dt, previous_x=x0)` before the loop (demo convention, see `generate_contact_api_demo.py:209,832,1033`).
- `ObstacleSpec.static(verts, tris)` / `.linear_velocity(verts, tris, velocity)`.
- Static solve: `OptimizationProblem(objective=EnergySet([...]))`, `problem.fix_variables(dofs, values, num_dofs=n)`, `NewtonOptimizer(max_iterations=, gradient_tolerance=).solve(problem, x0)` returns `SolverResult` with `.converged/.status/.iterations/.final_gradient_max_norm`.
- Gravity in statics: `LinearEnergy(-gravity_force)` (`pypgo/energy/algebraic.py:13`).
- Soft pins: `VertexAttachment(sim_mesh=, vertex_indices=, target_positions=, coeff=, is_displacement=True)` — assumes 3 DOFs/vertex.
- Deformation energy: volume → `deformation_energy(sim_mesh, elastic=StableNeo(), elastic_field=ElementwiseField(), plastic=VolumetricPlasticity(dofs=0), plastic_field=ElementwiseField(), formulation=fm)`; shell → `elastic=KoiterStVK(), plastic=ShellPlasticity(dofs=0)`, `formulation=KoiterShell()`, sim mesh from `SimulationMesh.create_shell(surface, KoiterStVKShellMaterial(thickness=, E_membrane=, nu_membrane=))`.
- `volume.mesh_data.vertices` is the (n,3) rest vertex array; `volume.mesh_data.elements.shape[1]` is 4 (tet) or 8 (cubic).
- Tricubic Hermite: 24 DOFs/vertex; initial 3-vectors pad into `[v*24 : v*24+3]` (see `volume_ipc.py:42`).
- Mesh IO: `read_obj(path) -> TriMeshData(.vertices, .elements)`, `write_obj(path, TriMeshData)`, `VolumeMesh.from_veg_file(read_veg(path))`.

**Asset geometry facts (measured, used by example configs):**
- `assets/veg/tet/dragon.veg`: 161 vertices, y ∈ [-0.3434, 0.4779] → bottom clamp region: axis y, side min, tolerance 0.05.
- `assets/obj/bunny.obj`: min y = 0.0335 → floor height 0.013 (bbox bottom − 0.02, demo convention).
- `assets/obj/shell.obj`: 33×33 grid in the z=0 plane, x ∈ [−0.5, 0.5], y ∈ [1.7435, 2.7435]; the y=max edge has 33 vertices.
- `assets/fixed/dragon-fixed.txt` indexes `dragon_big.veg` (indices up to 36645 > 161) — do NOT pair it with `dragon.veg`; example configs use region selectors instead.

---

### Task 1: `_config.py` — typed config, JSON load, overrides

**Files:**
- Create: `pypgo/tools/sim/_config.py`
- Test: `tests/pypgo/test_sim_config.py`

- [ ] **Step 1: Write the failing tests**

```python
"""Tests for pypgo.tools.sim._config."""

import json

import pytest

from pypgo.tools.sim._config import (
    ConfigError,
    RegionSelector,
    SimConfig,
    VertexSelector,
    load_config,
)


def _write(tmp_path, payload, name="scene.json"):
    p = tmp_path / name
    p.write_text(json.dumps(payload))
    return p


def test_minimal_volume_dynamic_config(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "model.veg", "surface": "model.obj"},
        "dynamic": {"timestep": 0.001, "num_steps": 5},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)
    assert cfg.mesh_type == "tet"
    assert cfg.mode == "dynamic"
    # JSON-relative path resolution
    assert cfg.mesh.volume == tmp_path / "model.veg"
    assert cfg.mesh.surface == tmp_path / "model.obj"
    assert cfg.output.directory == tmp_path / "out"
    assert cfg.dynamic.timestep == 0.001
    assert cfg.dynamic.num_steps == 5
    # defaults
    assert cfg.mesh.formulation == "auto"
    assert cfg.material.model == "stable_neo"
    assert cfg.dynamic.integrator == "implicit_euler"
    assert cfg.solver.max_iterations == 50
    assert cfg.loads.gravity == (0.0, 0.0, 0.0)
    assert cfg.contact == ()


def test_cli_overrides_beat_json(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "model.veg", "surface": "model.obj"},
        "dynamic": {"timestep": 0.001, "num_steps": 100},
        "output": {"directory": "out"},
    })
    cfg = load_config(
        mesh_type="tet", mode="dynamic", json_path=cfg_path,
        overrides={"dynamic.num_steps": 2, "loads.gravity": (0.0, -9.81, 0.0)},
    )
    assert cfg.dynamic.num_steps == 2
    assert cfg.dynamic.timestep == 0.001  # untouched JSON value survives
    assert cfg.loads.gravity == (0.0, -9.81, 0.0)


def test_pure_cli_no_json():
    cfg = load_config(
        mesh_type="cubic", mode="dynamic",
        overrides={
            "mesh.volume": "/abs/model.veg",
            "mesh.surface": "/abs/model.obj",
            "dynamic.timestep": 0.01,
            "output.directory": "/abs/out",
        },
    )
    assert str(cfg.mesh.volume) == "/abs/model.veg"
    assert cfg.dynamic.timestep == 0.01


def test_type_field_mismatch_rejected(tmp_path):
    cfg_path = _write(tmp_path, {
        "type": "shell",
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"timestep": 0.01},
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="type"):
        load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)


def test_dynamic_requires_timestep(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="timestep"):
        load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)


def test_static_ignores_dynamic_section(tmp_path, capsys):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"timestep": 0.01},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="tet", mode="static", json_path=cfg_path)
    assert cfg.mode == "static"  # no error; warning printed to stderr
    assert "dynamic" in capsys.readouterr().err


def test_volume_mesh_required_for_tet():
    with pytest.raises(ConfigError, match="volume"):
        load_config(mesh_type="tet", mode="static",
                    overrides={"mesh.surface": "/abs/m.obj",
                               "output.directory": "/abs/out"})


def test_shell_config(tmp_path):
    cfg_path = _write(tmp_path, {
        "type": "shell",
        "mesh": {"surface": "shell.obj"},
        "material": {"thickness": 0.002, "E_membrane": 2.0e4,
                     "nu_membrane": 0.35, "mass": {"density": 500.0}},
        "constraints": {"fixed": {"region": {"axis": "y", "side": "max"}}},
        "loads": {"gravity": [0.0, 0.0, -9.81]},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="shell", mode="static", json_path=cfg_path)
    assert cfg.material.thickness == 0.002
    assert cfg.material.mass.density == 500.0
    assert cfg.material.mass.areal_density is None
    assert cfg.constraints.fixed.region == RegionSelector(axis="y", side="max")


def test_shell_rejects_volume_mesh(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "shell.obj"},
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="volume"):
        load_config(mesh_type="shell", mode="static", json_path=cfg_path)


def test_shell_mass_requires_exactly_one_source(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"surface": "shell.obj"},
        "material": {"mass": {"density": 500.0, "areal_density": 1.0}},
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="mass"):
        load_config(mesh_type="shell", mode="static", json_path=cfg_path)


def test_vertex_selector_exactly_one_source():
    with pytest.raises(ConfigError):
        VertexSelector(indices=(1, 2), region=RegionSelector(axis="x", side="min"))
    with pytest.raises(ConfigError):
        VertexSelector()


def test_contact_config(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "contact": [
            {"model": "ipc", "dhat": 0.002, "dhat_external": 0.005,
             "kappa": 3000.0,
             "obstacles": [{"mesh": "bottom.obj"},
                           {"mesh": "mover.obj", "velocity": [0.0, 0.1, 0.0]}]},
            {"model": "floor", "axis": "y", "height": 0.013, "stiffness": 5000.0},
        ],
        "dynamic": {"timestep": 0.001},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)
    ipc, floor = cfg.contact
    assert ipc.model == "ipc" and ipc.kappa == 3000.0
    assert ipc.obstacles[0].mesh == tmp_path / "bottom.obj"
    assert ipc.obstacles[0].velocity is None
    assert ipc.obstacles[1].velocity == (0.0, 0.1, 0.0)
    assert floor.model == "floor" and floor.height == 0.013


def test_unknown_contact_model_rejected(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "contact": [{"model": "magic"}],
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="contact"):
        load_config(mesh_type="tet", mode="static", json_path=cfg_path)


def test_obstacles_only_for_ipc(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "contact": [{"model": "floor", "obstacles": [{"mesh": "b.obj"}]}],
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="obstacle"):
        load_config(mesh_type="tet", mode="static", json_path=cfg_path)


def test_output_directory_required():
    with pytest.raises(ConfigError, match="output"):
        load_config(mesh_type="tet", mode="static",
                    overrides={"mesh.volume": "/m.veg", "mesh.surface": "/m.obj"})


def test_hermite_formulation_only_for_cubic(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj",
                 "formulation": "cubic-tricubic-hermite"},
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="formulation"):
        load_config(mesh_type="tet", mode="static", json_path=cfg_path)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/pypgo/test_sim_config.py -x -q`
Expected: FAIL with `ModuleNotFoundError`/`ImportError` on `pypgo.tools.sim._config`.

- [ ] **Step 3: Implement `pypgo/tools/sim/_config.py`**

```python
"""Typed configuration for the pypgo-sim CLI family.

Precedence: built-in defaults < JSON config < CLI overrides.
Paths inside a JSON file resolve relative to the JSON file's directory;
override paths are taken as-is (the CLI layer resolves them against CWD).
"""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from pathlib import Path

MESH_TYPES = ("tet", "cubic", "shell")
VOLUME_FORMULATIONS = ("auto", "tet-linear", "cubic-linear", "cubic-tricubic-hermite")
VOLUME_ELASTIC_MODELS = ("stable_neo", "stvk", "stvk_volume", "linear_elastic", "mooney_rivlin")
CONTACT_MODELS = ("ipc", "floor", "sampled_penalty", "frictional_sampled_penalty")
INTEGRATORS = ("implicit_euler", "trbdf2")


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
    return VertexSelector(
        file=Path(file) if file is not None else None,
        indices=tuple(int(i) for i in indices) if indices is not None else None,
        region=RegionSelector(
            axis=region["axis"], side=region["side"],
            tolerance=float(region.get("tolerance", 1e-6)),
        ) if region is not None else None,
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


@dataclass(frozen=True)
class AttachmentConfig:
    vertices: VertexSelector
    coeff: float = 1e5


@dataclass(frozen=True)
class ConstraintsConfig:
    fixed: VertexSelector | None = None
    attachments: tuple[AttachmentConfig, ...] = ()


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
    friction_coeff: float = 0.3
    velocity_eps: float = 1e-4


@dataclass(frozen=True)
class InitialStateConfig:
    displacement: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class SolverConfig:
    max_iterations: int = 50
    gradient_tolerance: float = 1e-6


@dataclass(frozen=True)
class DynamicConfig:
    timestep: float | None = None
    num_steps: int = 1
    integrator: str = "implicit_euler"
    damping: tuple[float, float] = (0.0, 0.0)


@dataclass(frozen=True)
class OutputConfig:
    directory: Path | None = None
    write_surfaces: bool = False


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
    for att in constraints.get("attachments", []):
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
    obstacles = tuple(
        ObstacleConfig(
            mesh=Path(obs["mesh"]),
            velocity=_vec3(obs["velocity"], f"{label}.obstacles.velocity")
            if obs.get("velocity") is not None else None,
        )
        for obs in payload.get("obstacles", [])
    )
    if obstacles and model != "ipc":
        raise ConfigError(f"{label}: obstacles are only supported for ipc contact")
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
        friction_coeff=float(payload.get("friction_coeff", 0.3)),
        velocity_eps=float(payload.get("velocity_eps", 1e-4)),
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
            payload = json.loads(json_path.read_text())
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
        )
    else:
        material = VolumeMaterialConfig(
            model=mat_payload.get("model", "stable_neo"),
            density=float(mat_payload["density"])
            if mat_payload.get("density") is not None else None,
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
            AttachmentConfig(
                vertices=_selector_from_payload(
                    att["vertices"], "constraints.attachments.vertices"),
                coeff=float(att.get("coeff", 1e5)),
            )
            for att in cons_payload.get("attachments", [])
        ),
    )

    loads = LoadsConfig(
        gravity=_vec3(payload.get("loads", {}).get("gravity", (0.0, 0.0, 0.0)),
                      "loads.gravity"))

    contact = tuple(
        _build_contact(c, f"contact[{i}]")
        for i, c in enumerate(payload.get("contact", []))
    )

    init_payload = payload.get("initial_state", {})
    initial_state = InitialStateConfig(
        displacement=_vec3(init_payload.get("displacement", (0.0, 0.0, 0.0)),
                           "initial_state.displacement"),
        velocity=_vec3(init_payload.get("velocity", (0.0, 0.0, 0.0)),
                       "initial_state.velocity"),
    )

    solver_payload = payload.get("solver", {})
    solver = SolverConfig(
        max_iterations=int(solver_payload.get("max_iterations", 50)),
        gradient_tolerance=float(solver_payload.get("gradient_tolerance", 1e-6)),
    )

    dyn_payload = payload.get("dynamic", {})
    dynamic = DynamicConfig(
        timestep=float(dyn_payload["timestep"])
        if dyn_payload.get("timestep") is not None else None,
        num_steps=int(dyn_payload.get("num_steps", 1)),
        integrator=dyn_payload.get("integrator", "implicit_euler"),
        damping=tuple(float(v) for v in dyn_payload.get("damping", (0.0, 0.0))),
    )
    if mode == "dynamic":
        if dynamic.timestep is None or not dynamic.timestep > 0.0:
            raise ConfigError("dynamic.timestep is required and must be positive")
        if dynamic.integrator not in INTEGRATORS:
            raise ConfigError(
                f"dynamic.integrator must be one of {INTEGRATORS}, "
                f"got {dynamic.integrator!r}")
        if dynamic.num_steps < 0:
            raise ConfigError("dynamic.num_steps must be non-negative")
        if len(dynamic.damping) != 2:
            raise ConfigError("dynamic.damping must be a [mass, stiffness] pair")
    elif dyn_payload:
        print("warning: 'dynamic' section is ignored in static mode", file=sys.stderr)

    out_payload = payload.get("output", {})
    output = OutputConfig(
        directory=Path(out_payload["directory"])
        if out_payload.get("directory") else None,
        write_surfaces=bool(out_payload.get("write_surfaces", False)),
    )
    if output.directory is None:
        raise ConfigError("output.directory is required (JSON or --output-dir)")

    return SimConfig(
        mesh_type=mesh_type, mode=mode, mesh=mesh, material=material,
        constraints=constraints, loads=loads, contact=contact,
        initial_state=initial_state, solver=solver, dynamic=dynamic, output=output,
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/pypgo/test_sim_config.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add pypgo/tools/sim/_config.py tests/pypgo/test_sim_config.py
git commit -m "feat(sim-cli): typed SimConfig with JSON load, CLI overrides, validation"
```

---

### Task 2: `_scene.py` — vertex selectors + volume scene builder

**Files:**
- Create: `pypgo/tools/sim/_scene.py`
- Test: `tests/pypgo/test_sim_scene.py`

The test builds a tiny single-tet veg in tmp_path (same pattern as `tests/pypgo/test_tool_clis.py:134` does for the volume_ipc test — read that test first and reuse its mesh-construction helper code verbatim if it has one; otherwise use the box assets from `examples/assets`, they are small).

- [ ] **Step 1: Write the failing tests**

```python
"""Tests for pypgo.tools.sim._scene."""

from pathlib import Path

import numpy as np
import pytest

from pypgo.tools.sim._config import (
    ConfigError, RegionSelector, VertexSelector, load_config,
)
from pypgo.tools.sim._scene import build_scene, resolve_vertex_selector

ASSETS = Path(__file__).resolve().parents[2] / "examples" / "assets"


def test_resolve_selector_indices():
    verts = np.zeros((5, 3))
    sel = VertexSelector(indices=(3, 1, 3))
    assert resolve_vertex_selector(sel, verts).tolist() == [1, 3]


def test_resolve_selector_region_max_y():
    verts = np.array([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]])
    sel = VertexSelector(region=RegionSelector(axis="y", side="max", tolerance=1e-9))
    assert resolve_vertex_selector(sel, verts).tolist() == [1, 2]


def test_resolve_selector_file(tmp_path):
    f = tmp_path / "fixed.txt"
    f.write_text("2 0\n1\n")
    sel = VertexSelector(file=f)
    assert resolve_vertex_selector(sel, np.zeros((4, 3))).tolist() == [0, 1, 2]


def test_resolve_selector_out_of_range():
    sel = VertexSelector(indices=(10,))
    with pytest.raises(ConfigError, match="range"):
        resolve_vertex_selector(sel, np.zeros((3, 3)))


def test_resolve_selector_empty_region():
    sel = VertexSelector(indices=())
    with pytest.raises(ConfigError, match="no vertices"):
        resolve_vertex_selector(sel, np.zeros((3, 3)))


@pytest.fixture(scope="module")
def tet_box_cfg_payload():
    return {
        "mesh.volume": str(ASSETS / "veg" / "tet" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "output.directory": "/tmp/unused",
    }


def test_build_volume_scene_tet_static(tet_box_cfg_payload):
    cfg = load_config(mesh_type="tet", mode="static", overrides={
        **tet_box_cfg_payload,
        "loads.gravity": (0.0, -9.81, 0.0),
        "constraints.fixed": {"region": {"axis": "y", "side": "max",
                                         "tolerance": 1e-3}},
    })
    bundle = build_scene(cfg)
    n = bundle.num_dofs
    assert bundle.dofs_per_vertex == 3
    assert bundle.mass.shape == (n, n)
    assert bundle.gravity_force.shape == (n,)
    assert float(np.linalg.norm(bundle.gravity_force)) > 0.0
    assert bundle.fixed_dofs is not None and bundle.fixed_dofs.size % 3 == 0
    # surface mapping: zero displacement keeps the rest surface
    rest = bundle.surface_positions(np.zeros(n))
    np.testing.assert_allclose(rest, bundle.surface_rest)
    # initial vector tiling
    v = bundle.initial_vector((0.0, -3.0, 0.0))
    assert v.shape == (n,)
    assert v[1] == -3.0 and v[0] == 0.0


def test_build_volume_scene_with_contact_and_attachment(tet_box_cfg_payload):
    cfg = load_config(mesh_type="tet", mode="dynamic", overrides={
        **tet_box_cfg_payload,
        "dynamic.timestep": 0.001,
        "contact": [
            {"model": "floor", "axis": "y", "height": 0.0, "stiffness": 100.0},
            {"model": "ipc", "dhat": 0.002, "kappa": 100.0,
             "obstacles": [{"mesh": str(ASSETS / "obj" / "bottom.obj")}]},
        ],
        "constraints.attachments": [
            {"vertices": {"region": {"axis": "y", "side": "max",
                                     "tolerance": 1e-3}}, "coeff": 1e4},
        ],
    })
    bundle = build_scene(cfg)
    assert len(bundle.contact_energies) == 2
    assert len(bundle.attachment_energies) == 1
    # the IPC energy is stateful and tracks obstacles
    assert len(bundle.stateful_contacts) == 1
    assert len(bundle.ipc_contacts) == 1
    assert len(bundle.ipc_contacts[0].obstacles) == 1


def test_build_volume_scene_hermite(tet_box_cfg_payload):
    cfg = load_config(mesh_type="cubic", mode="dynamic", overrides={
        "mesh.volume": str(ASSETS / "veg" / "cubic" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "mesh.formulation": "cubic-tricubic-hermite",
        "dynamic.timestep": 0.001,
        "output.directory": "/tmp/unused",
    })
    bundle = build_scene(cfg)
    assert bundle.dofs_per_vertex == 24
    vec = bundle.initial_vector((0.0, -3.0, 0.0))
    assert vec[1] == -3.0           # first vertex position-y DOF
    assert vec[3] == 0.0            # derivative DOFs stay zero
    assert vec[25] == -3.0          # second vertex position-y DOF


def test_hermite_rejects_attachments():
    cfg = load_config(mesh_type="cubic", mode="static", overrides={
        "mesh.volume": str(ASSETS / "veg" / "cubic" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "mesh.formulation": "cubic-tricubic-hermite",
        "constraints.attachments": [
            {"vertices": {"indices": [0]}, "coeff": 1.0},
        ],
        "output.directory": "/tmp/unused",
    })
    with pytest.raises(ConfigError, match="attachment"):
        build_scene(cfg)


def test_formulation_auto_detects_element_width(tet_box_cfg_payload):
    cfg = load_config(mesh_type="tet", mode="static", overrides=tet_box_cfg_payload)
    bundle = build_scene(cfg)
    assert type(bundle.formulation).__name__ == "TetLinear"


def test_formulation_mismatch_rejected():
    # cubic command pointed at a tet veg with formulation auto -> clear error
    cfg = load_config(mesh_type="cubic", mode="static", overrides={
        "mesh.volume": str(ASSETS / "veg" / "tet" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "output.directory": "/tmp/unused",
    })
    with pytest.raises(ConfigError, match="element"):
        build_scene(cfg)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/pypgo/test_sim_scene.py -x -q`
Expected: FAIL with `ImportError` on `pypgo.tools.sim._scene`.

- [ ] **Step 3: Implement `pypgo/tools/sim/_scene.py`** (volume path; shell builder raises `NotImplementedError` until Task 3)

```python
"""Scene construction for the pypgo-sim CLI family.

Builds a mesh-type-independent SceneBundle from a SimConfig; the runners in
_runners.py never branch on mesh type.
"""

from __future__ import annotations

from dataclasses import dataclass, field
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
        idx = np.array([int(t) for t in tokens], dtype=np.int64)
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
    return (idx[:, None] * dofs_per_vertex + np.arange(3, dtype=np.int64)).ravel()


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
    raise NotImplementedError  # Task 3


def build_scene(cfg: SimConfig) -> SceneBundle:
    if cfg.mesh_type == "shell":
        return _build_shell_scene(cfg)
    return _build_volume_scene(cfg)
```

Note for the implementer: `VertexAttachment` lives in `pypgo.energy` (check `pypgo/energy/__init__.py` exports; if it is not re-exported there, import from `pypgo.energy.attachment`). Same check for `FloorParameters` etc. in `pypgo.contact` (`pypgo/contact/__init__.py`). Fix imports to match the actual `__init__` exports — do not edit the `__init__` files unless an export is genuinely missing.

- [ ] **Step 4: Run tests (skip the still-failing shell ones — there are none in this file) and verify pass**

Run: `python -m pytest tests/pypgo/test_sim_scene.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add pypgo/tools/sim/_scene.py tests/pypgo/test_sim_scene.py
git commit -m "feat(sim-cli): SceneBundle + vertex selectors + volume scene builder"
```

---

### Task 3: `_scene.py` — shell scene builder

**Files:**
- Modify: `pypgo/tools/sim/_scene.py` (replace `_build_shell_scene` stub)
- Test: `tests/pypgo/test_sim_scene.py` (append)

- [ ] **Step 1: Append failing tests**

```python
def _shell_overrides():
    return {
        "mesh.surface": str(ASSETS / "obj" / "shell.obj"),
        "material": {"thickness": 0.001, "E_membrane": 1.0e6,
                     "nu_membrane": 0.4, "mass": {"density": 1000.0}},
        "output.directory": "/tmp/unused",
    }


def test_build_shell_scene_static():
    cfg = load_config(mesh_type="shell", mode="static", overrides={
        **_shell_overrides(),
        "loads.gravity": (0.0, 0.0, -9.81),
        "constraints.fixed": {"region": {"axis": "y", "side": "max",
                                         "tolerance": 1e-6}},
    })
    bundle = build_scene(cfg)
    assert bundle.dofs_per_vertex == 3
    assert bundle.surface_map is None
    n = bundle.num_dofs
    assert bundle.mass.shape == (n, n)
    assert float(np.linalg.norm(bundle.gravity_force)) > 0.0
    # shell.obj y=max edge has 33 vertices -> 99 fixed DOFs
    assert bundle.fixed_dofs.size == 99
    # identity surface mapping
    u = np.zeros(n)
    u[2] = -0.5
    pos = bundle.surface_positions(u)
    assert pos[0, 2] == pytest.approx(bundle.surface_rest[0, 2] - 0.5)


def test_build_shell_scene_areal_density_and_contact():
    cfg = load_config(mesh_type="shell", mode="dynamic", overrides={
        "mesh.surface": str(ASSETS / "obj" / "shell.obj"),
        "material": {"mass": {"areal_density": 1.0}},
        "dynamic.timestep": 0.0005,
        "contact": [
            {"model": "frictional_sampled_penalty", "stiffness": 10.0,
             "friction_coeff": 0.3, "velocity_eps": 1e-4},
            {"model": "floor", "axis": "z", "height": -0.1, "stiffness": 5000.0},
        ],
        "output.directory": "/tmp/unused",
    })
    bundle = build_scene(cfg)
    assert len(bundle.contact_energies) == 2
    assert len(bundle.stateful_contacts) == 1  # frictional penalty only
    assert bundle.mass.shape == (bundle.num_dofs, bundle.num_dofs)
```

- [ ] **Step 2: Run tests to verify the new ones fail**

Run: `python -m pytest tests/pypgo/test_sim_scene.py -q -k shell`
Expected: 2 FAIL with `NotImplementedError`.

- [ ] **Step 3: Implement `_build_shell_scene`**

Replace the stub in `pypgo/tools/sim/_scene.py`:

```python
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
```

Check the actual names exported by `pypgo.fem` (`pypgo/fem/__init__.py`): `KoiterStVKShellMaterial`, `ShellPlasticity`, `ShellArealDensity`, `ShellDensityThickness`, `KoiterShell`, `KoiterStVK` — adjust import paths if any are not re-exported at package level.

- [ ] **Step 4: Run all scene tests and verify pass**

Run: `python -m pytest tests/pypgo/test_sim_scene.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add pypgo/tools/sim/_scene.py tests/pypgo/test_sim_scene.py
git commit -m "feat(sim-cli): shell scene builder using shell mass/body-force operators"
```

---

### Task 4: `_outputs.py` + `_runners.py`

**Files:**
- Create: `pypgo/tools/sim/_outputs.py`
- Create: `pypgo/tools/sim/_runners.py`
- Test: `tests/pypgo/test_sim_runners.py`

- [ ] **Step 1: Write the failing tests**

```python
"""Tests for pypgo.tools.sim._runners (static + dynamic, volume + shell)."""

import json
from pathlib import Path

import numpy as np
import pytest

from pypgo.tools.sim._config import load_config
from pypgo.tools.sim._runners import run_dynamic, run_static
from pypgo.tools.sim._scene import build_scene

ASSETS = Path(__file__).resolve().parents[2] / "examples" / "assets"


def test_run_static_volume_box_hang(tmp_path):
    cfg = load_config(mesh_type="tet", mode="static", overrides={
        "mesh.volume": str(ASSETS / "veg" / "tet" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "constraints.fixed": {"region": {"axis": "y", "side": "max",
                                         "tolerance": 1e-3}},
        "loads.gravity": (0.0, -9.81, 0.0),
        "solver.max_iterations": 20,
        "solver.gradient_tolerance": 1e-4,
        "output.directory": str(tmp_path),
        "output.write_surfaces": True,
    })
    summary = run_static(build_scene(cfg), cfg)
    assert summary["mode"] == "static"
    assert summary["num_dofs"] > 0
    assert isinstance(summary["converged"], bool)
    saved = json.loads((tmp_path / "summary.json").read_text())
    assert saved["mode"] == "static"
    assert (tmp_path / "final_surface.obj").exists()


def test_run_dynamic_volume_two_steps(tmp_path):
    cfg = load_config(mesh_type="tet", mode="dynamic", overrides={
        "mesh.volume": str(ASSETS / "veg" / "tet" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "loads.gravity": (0.0, -9.81, 0.0),
        "dynamic.timestep": 0.001,
        "dynamic.num_steps": 2,
        "output.directory": str(tmp_path),
        "output.write_surfaces": True,
    })
    summary = run_dynamic(build_scene(cfg), cfg)
    assert summary["mode"] == "dynamic"
    assert summary["num_frames"] == 2
    assert summary["final_time"] == pytest.approx(0.002)
    assert (tmp_path / "surface" / "surface0000.obj").exists()
    assert (tmp_path / "surface" / "surface0001.obj").exists()


def test_run_dynamic_shell_two_steps(tmp_path):
    cfg = load_config(mesh_type="shell", mode="dynamic", overrides={
        "mesh.surface": str(ASSETS / "obj" / "shell.obj"),
        "material": {"mass": {"areal_density": 1.0}},
        "loads.gravity": (0.0, 0.0, -9.81),
        "initial_state.velocity": (0.0, 0.0, -1.0),
        "dynamic.timestep": 0.0005,
        "dynamic.num_steps": 2,
        "contact": [{"model": "floor", "axis": "z", "height": -0.1,
                     "stiffness": 5000.0}],
        "output.directory": str(tmp_path),
    })
    summary = run_dynamic(build_scene(cfg), cfg)
    assert summary["num_frames"] == 2
    # falling: mean z displacement is negative
    assert summary["frames"][-1]["accepted"] is True


def test_run_static_shell_drape(tmp_path):
    cfg = load_config(mesh_type="shell", mode="static", overrides={
        "mesh.surface": str(ASSETS / "obj" / "shell.obj"),
        "material": {"thickness": 0.001, "E_membrane": 1.0e6,
                     "nu_membrane": 0.4, "mass": {"density": 1000.0}},
        "constraints.fixed": {"region": {"axis": "y", "side": "max",
                                         "tolerance": 1e-6}},
        "loads.gravity": (0.0, 0.0, -9.81),
        "solver.max_iterations": 10,
        "solver.gradient_tolerance": 1e-4,
        "output.directory": str(tmp_path),
        "output.write_surfaces": True,
    })
    summary = run_static(build_scene(cfg), cfg)
    assert (tmp_path / "final_surface.obj").exists()
    # clamped edge stays put: fixed DOFs of the solution remain ~0
    assert summary["max_fixed_abs_u"] == pytest.approx(0.0, abs=1e-12)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/pypgo/test_sim_runners.py -x -q`
Expected: FAIL with `ImportError` on `pypgo.tools.sim._runners`.

- [ ] **Step 3: Implement `pypgo/tools/sim/_outputs.py`**

```python
"""Output writers for the pypgo-sim CLI family."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from pypgo.mesh import TriMeshData, write_obj


def write_summary(output_dir: Path, payload: dict) -> Path:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / "summary.json"
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")
    return path


def write_surface(path: Path, vertices: np.ndarray, triangles: np.ndarray) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    write_obj(str(path), TriMeshData(np.asarray(vertices, dtype=np.float64),
                                     np.asarray(triangles)))
```

- [ ] **Step 4: Implement `pypgo/tools/sim/_runners.py`**

```python
"""Static and dynamic runners — mesh-type independent."""

from __future__ import annotations

import numpy as np

import pypgo.energy as _energy
import pypgo.solver as _solver
from pypgo.sim import DynamicSimulation, DynamicState
from pypgo.tools.sim._outputs import write_summary, write_surface
from pypgo.tools.sim._scene import SceneBundle


def _make_optimizer(cfg):
    return _solver.NewtonOptimizer(
        max_iterations=cfg.solver.max_iterations,
        gradient_tolerance=cfg.solver.gradient_tolerance,
    )


def run_static(bundle: SceneBundle, cfg) -> dict:
    x0 = bundle.initial_vector(cfg.initial_state.displacement)
    objective = _energy.EnergySet(
        bundle.weighted_energies(include_gravity_potential=True))
    problem = _solver.OptimizationProblem(objective=objective)
    if bundle.fixed_dofs is not None:
        problem.fix_variables(
            bundle.fixed_dofs.tolist(), x0[bundle.fixed_dofs],
            num_dofs=bundle.num_dofs)
    for e in bundle.stateful_contacts:
        e.begin_step(time=0.0, timestep=1.0, previous_x=x0)

    result = _make_optimizer(cfg).solve(problem, x0)

    summary = {
        "mode": "static",
        "mesh_type": cfg.mesh_type,
        "num_dofs": bundle.num_dofs,
        "converged": bool(result.converged),
        "status": result.status.name,
        "iterations": int(result.iterations),
        "final_gradient_max_norm": float(result.final_gradient_max_norm),
        "max_abs_u": float(np.max(np.abs(result.x))),
        "max_fixed_abs_u": float(np.max(np.abs(result.x[bundle.fixed_dofs])))
        if bundle.fixed_dofs is not None else None,
    }
    if cfg.output.write_surfaces:
        write_surface(cfg.output.directory / "final_surface.obj",
                      bundle.surface_positions(result.x), bundle.surface_triangles)
    write_summary(cfg.output.directory, summary)
    return summary


def run_dynamic(bundle: SceneBundle, cfg) -> dict:
    dt = cfg.dynamic.timestep
    x0 = bundle.initial_vector(cfg.initial_state.displacement)
    v0 = bundle.initial_vector(cfg.initial_state.velocity)
    state = DynamicState(
        displacement=x0, velocity=v0,
        acceleration=np.zeros(bundle.num_dofs, dtype=np.float64))

    energy = _energy.EnergySet(
        bundle.weighted_energies(include_gravity_potential=False))
    sim = DynamicSimulation(
        mass=bundle.mass, state=state, timestep=dt, energy=energy,
        integrator=cfg.dynamic.integrator, damping=cfg.dynamic.damping,
        fixed_dofs=bundle.fixed_dofs.tolist()
        if bundle.fixed_dofs is not None else None,
    )
    for e in bundle.stateful_contacts:
        e.begin_step(time=0.0, timestep=dt, previous_x=x0)

    optimizer = _make_optimizer(cfg)
    frames = []
    for _ in range(cfg.dynamic.num_steps):
        for ipc in bundle.ipc_contacts:
            if ipc.obstacles:
                ipc.set_moving_obstacle_time(sim.state.time + dt)
        frame = sim.step(external_force=bundle.gravity_force, optimizer=optimizer)
        frames.append(frame)
        if cfg.output.write_surfaces:
            write_surface(
                cfg.output.directory / "surface" / f"surface{frame.frame_index:04d}.obj",
                bundle.surface_positions(frame.displacement),
                bundle.surface_triangles)
        if not frame.accepted:
            break

    summary = {
        "mode": "dynamic",
        "mesh_type": cfg.mesh_type,
        "num_dofs": bundle.num_dofs,
        "num_frames": len(frames),
        "final_time": float(sim.state.time),
        "final_timestep_id": int(sim.state.timestep_id),
        "frames": [
            {"frame_index": f.frame_index,
             "accepted": bool(f.accepted),
             "status": f.solver_result.status.name,
             "iterations": int(f.solver_result.iterations)}
            for f in frames
        ],
    }
    write_summary(cfg.output.directory, summary)
    return summary
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `python -m pytest tests/pypgo/test_sim_runners.py -q`
Expected: all PASS. (These run real small solves; allow ~1–2 min.)

- [ ] **Step 6: Commit**

```bash
git add pypgo/tools/sim/_outputs.py pypgo/tools/sim/_runners.py tests/pypgo/test_sim_runners.py
git commit -m "feat(sim-cli): shared static/dynamic runners and output writers"
```

---

### Task 5: `_cli.py`, six entry modules, console scripts

**Files:**
- Create: `pypgo/tools/sim/_cli.py`
- Create: `pypgo/tools/sim/shell_static.py`, `shell_dynamic.py`, `cubic_static.py`, `cubic_dynamic.py`, `tet_static.py`, `tet_dynamic.py`
- Modify: `pypgo/tools/sim/__init__.py`
- Modify: `setup.py` (console_scripts, around line 117)
- Test: `tests/pypgo/test_tool_clis.py` (append)

- [ ] **Step 1: Append failing tests to `tests/pypgo/test_tool_clis.py`**

```python
def test_sim_cli_tet_dynamic_runs_from_args(tmp_path):
    from pypgo.tools.sim import tet_dynamic

    assets = Path(__file__).resolve().parents[2] / "examples" / "assets"
    out = tmp_path / "out"
    ret = tet_dynamic.main([
        "--volume", str(assets / "veg" / "tet" / "box.veg"),
        "--surface", str(assets / "obj" / "box.obj"),
        "--output-dir", str(out),
        "--timestep", "0.001",
        "--num-steps", "1",
        "--gravity", "0", "-9.81", "0",
    ])
    assert ret == 0
    assert (out / "summary.json").exists()


def test_sim_cli_config_error_exits_2(tmp_path):
    from pypgo.tools.sim import tet_static

    with pytest.raises(SystemExit) as excinfo:
        tet_static.main(["--output-dir", str(tmp_path)])  # missing meshes
    assert excinfo.value.code == 2


def test_setup_declares_sim_console_scripts():
    setup_text = (Path(__file__).resolve().parents[2] / "setup.py").read_text()
    for name in ("shell-static", "shell-dynamic", "cubic-static",
                 "cubic-dynamic", "tet-static", "tet-dynamic"):
        module = name.replace("-", "_")
        assert f"pypgo-sim-{name}=pypgo.tools.sim.{module}:main" in setup_text
```

(Ensure `from pathlib import Path` and `import pytest` are already imported at the top of the file; add if missing.)

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/pypgo/test_tool_clis.py -q -k sim_cli or setup_declares_sim`
Expected: FAIL (`ImportError: cannot import name 'tet_dynamic'`, missing console scripts).

- [ ] **Step 3: Implement `pypgo/tools/sim/_cli.py`**

```python
"""Shared argparse layer for the pypgo-sim CLI family.

Precedence: defaults < JSON (--config) < explicit CLI flags.
CLI-given paths resolve against the CWD; JSON paths against the JSON dir.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from pypgo.tools.sim._config import (
    ConfigError, INTEGRATORS, VOLUME_FORMULATIONS, load_config,
)


def build_parser(*, prog: str, mesh_type: str, mode: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=prog,
        description=f"Run a {mesh_type} {mode} simulation "
                    f"(defaults < JSON --config < CLI flags).",
    )
    parser.add_argument("--config", type=Path, default=None,
                        help="JSON scene config; paths inside resolve relative to it")
    if mesh_type == "shell":
        parser.add_argument("--surface", type=Path, default=None,
                            help="shell OBJ mesh")
    else:
        parser.add_argument("--volume", type=Path, default=None,
                            help="input .veg volume mesh")
        parser.add_argument("--surface", type=Path, default=None,
                            help="embedded/contact OBJ surface")
        parser.add_argument("--formulation", choices=VOLUME_FORMULATIONS,
                            default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--write-surfaces", action="store_true", default=None,
                        help="write deformed surface OBJ output")
    parser.add_argument("--gravity", nargs=3, type=float, default=None,
                        metavar=("GX", "GY", "GZ"))
    parser.add_argument("--solver-max-iterations", type=int, default=None)
    parser.add_argument("--solver-gradient-tolerance", type=float, default=None)
    if mode == "dynamic":
        parser.add_argument("--timestep", type=float, default=None)
        parser.add_argument("--num-steps", type=int, default=None)
        parser.add_argument("--integrator", choices=INTEGRATORS, default=None)
        parser.add_argument("--damping", nargs=2, type=float, default=None,
                            metavar=("MASS", "STIFFNESS"))
    return parser


def _overrides_from_args(args, *, mesh_type: str, mode: str) -> dict:
    overrides: dict = {}

    def put(dotted, value):
        if value is not None:
            overrides[dotted] = value

    def put_path(dotted, value):
        if value is not None:
            overrides[dotted] = str(Path(value).resolve())

    if mesh_type != "shell":
        put_path("mesh.volume", args.volume)
        put("mesh.formulation", args.formulation)
    put_path("mesh.surface", args.surface)
    put_path("output.directory", args.output_dir)
    put("output.write_surfaces", args.write_surfaces)
    put("loads.gravity", tuple(args.gravity) if args.gravity is not None else None)
    put("solver.max_iterations", args.solver_max_iterations)
    put("solver.gradient_tolerance", args.solver_gradient_tolerance)
    if mode == "dynamic":
        put("dynamic.timestep", args.timestep)
        put("dynamic.num_steps", args.num_steps)
        put("dynamic.integrator", args.integrator)
        put("dynamic.damping",
            tuple(args.damping) if args.damping is not None else None)
    return overrides


def run_cli(*, mesh_type: str, mode: str, prog: str, argv=None) -> int:
    parser = build_parser(prog=prog, mesh_type=mesh_type, mode=mode)
    args = parser.parse_args(argv)
    from pypgo.tools.sim._runners import run_dynamic, run_static
    from pypgo.tools.sim._scene import build_scene

    try:
        cfg = load_config(
            mesh_type=mesh_type, mode=mode, json_path=args.config,
            overrides=_overrides_from_args(args, mesh_type=mesh_type, mode=mode))
        bundle = build_scene(cfg)
        runner = run_static if mode == "static" else run_dynamic
        summary = runner(bundle, cfg)
    except ConfigError as exc:
        parser.error(str(exc))  # prints to stderr and exits with code 2
    print(f"{prog}: wrote {cfg.output.directory / 'summary.json'}")
    if mode == "dynamic":
        print(f"{prog}: {summary['num_frames']} frames, "
              f"final time {summary['final_time']:.6f}")
    else:
        print(f"{prog}: converged={summary['converged']} "
              f"iterations={summary['iterations']}")
    return 0
```

- [ ] **Step 4: Create the six entry modules**

`pypgo/tools/sim/tet_static.py` (the other five are identical apart from mesh_type/mode/prog):

```python
"""pypgo-sim-tet-static — static tet FEM solve from JSON/CLI scene config."""

from pypgo.tools.sim._cli import run_cli


def main(argv=None) -> int:
    return run_cli(mesh_type="tet", mode="static",
                   prog="pypgo-sim-tet-static", argv=argv)


if __name__ == "__main__":
    raise SystemExit(main())
```

Create likewise:
- `tet_dynamic.py`: `mesh_type="tet", mode="dynamic", prog="pypgo-sim-tet-dynamic"`
- `cubic_static.py`: `mesh_type="cubic", mode="static", prog="pypgo-sim-cubic-static"`
- `cubic_dynamic.py`: `mesh_type="cubic", mode="dynamic", prog="pypgo-sim-cubic-dynamic"`
- `shell_static.py`: `mesh_type="shell", mode="static", prog="pypgo-sim-shell-static"`
- `shell_dynamic.py`: `mesh_type="shell", mode="dynamic", prog="pypgo-sim-shell-dynamic"`

Update `pypgo/tools/sim/__init__.py`:

```python
"""Simulation CLI tools."""

__all__ = [
    "volume_ipc",
    "shell_static", "shell_dynamic",
    "cubic_static", "cubic_dynamic",
    "tet_static", "tet_dynamic",
]
```

- [ ] **Step 5: Register console scripts in `setup.py`**

In the `console_scripts` list (after the `pypgo-volume-ipc` line at `setup.py:117`), add:

```python
            "pypgo-sim-shell-static=pypgo.tools.sim.shell_static:main",
            "pypgo-sim-shell-dynamic=pypgo.tools.sim.shell_dynamic:main",
            "pypgo-sim-cubic-static=pypgo.tools.sim.cubic_static:main",
            "pypgo-sim-cubic-dynamic=pypgo.tools.sim.cubic_dynamic:main",
            "pypgo-sim-tet-static=pypgo.tools.sim.tet_static:main",
            "pypgo-sim-tet-dynamic=pypgo.tools.sim.tet_dynamic:main",
```

- [ ] **Step 6: Run the new tests plus the pre-existing CLI tests**

Run: `python -m pytest tests/pypgo/test_tool_clis.py -q`
Expected: all PASS (including the old volume_ipc test — proves no regression there).

- [ ] **Step 7: Commit**

```bash
git add pypgo/tools/sim/ setup.py tests/pypgo/test_tool_clis.py
git commit -m "feat(sim-cli): six pypgo-sim entry points sharing one CLI/config layer"
```

---

### Task 6: example JSON configs + smoke tests + README

**Files:**
- Create: `examples/sim_configs/tet_static_dragon.json`
- Create: `examples/sim_configs/tet_dynamic_bunny_floor.json`
- Create: `examples/sim_configs/cubic_static_box_hang.json`
- Create: `examples/sim_configs/cubic_dynamic_box_ipc.json`
- Create: `examples/sim_configs/shell_static_drape.json`
- Create: `examples/sim_configs/shell_dynamic_floor.json`
- Create: `examples/sim_configs/README.md`
- Test: `tests/pypgo/test_sim_cli_examples.py`

All scene parameters below are copied from verified demos (`generate_contact_api_demo.py` scene dicts, `generate_static_solve_*` demos). Output directories point into `examples/outputs/` but the smoke test always overrides `--output-dir` to a tmpdir.

- [ ] **Step 1: Write the failing smoke test**

```python
"""Smoke tests: every examples/sim_configs/*.json runs through its CLI."""

import json
from pathlib import Path

import pytest

from pypgo.tools.sim import (
    cubic_dynamic, cubic_static, shell_dynamic, shell_static,
    tet_dynamic, tet_static,
)

CONFIG_DIR = Path(__file__).resolve().parents[2] / "examples" / "sim_configs"

DYNAMIC_CASES = [
    (tet_dynamic, "tet_dynamic_bunny_floor.json"),
    (cubic_dynamic, "cubic_dynamic_box_ipc.json"),
    (shell_dynamic, "shell_dynamic_floor.json"),
]

STATIC_CASES = [
    (tet_static, "tet_static_dragon.json"),
    (cubic_static, "cubic_static_box_hang.json"),
    (shell_static, "shell_static_drape.json"),
]


def test_all_example_configs_are_covered():
    covered = {name for _, name in DYNAMIC_CASES + STATIC_CASES}
    on_disk = {p.name for p in CONFIG_DIR.glob("*.json")}
    assert covered == on_disk


@pytest.mark.parametrize("module,config", DYNAMIC_CASES,
                         ids=[c for _, c in DYNAMIC_CASES])
def test_dynamic_example_runs(tmp_path, module, config):
    ret = module.main([
        "--config", str(CONFIG_DIR / config),
        "--output-dir", str(tmp_path),
        "--num-steps", "2",
    ])
    assert ret == 0
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["mode"] == "dynamic"
    assert summary["num_frames"] == 2
    assert all(f["accepted"] for f in summary["frames"])
    surface_dir = tmp_path / "surface"
    if summary.get("num_frames") and (surface_dir).exists():
        assert (surface_dir / "surface0000.obj").exists()


@pytest.mark.parametrize("module,config", STATIC_CASES,
                         ids=[c for _, c in STATIC_CASES])
def test_static_example_runs(tmp_path, module, config):
    ret = module.main([
        "--config", str(CONFIG_DIR / config),
        "--output-dir", str(tmp_path),
        "--solver-max-iterations", "5",
    ])
    assert ret == 0
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["mode"] == "static"
    assert summary["iterations"] >= 1
```

- [ ] **Step 2: Run to verify failure**

Run: `python -m pytest tests/pypgo/test_sim_cli_examples.py -x -q`
Expected: FAIL (config dir does not exist).

- [ ] **Step 3: Create the six configs**

`examples/sim_configs/tet_static_dragon.json` — dragon under gravity, soft-pinned bottom band (mirrors `static_solve_dragon_gravity_demo`, small `dragon.veg`; the demo's `dragon-fixed.txt` indexes `dragon_big.veg` and must not be used here):

```json
{
  "type": "tet",
  "mesh": {
    "volume": "../assets/veg/tet/dragon.veg",
    "surface": "../assets/obj/dragon.obj"
  },
  "material": { "model": "stable_neo" },
  "constraints": {
    "attachments": [
      {
        "vertices": { "region": { "axis": "y", "side": "min", "tolerance": 0.05 } },
        "coeff": 100000.0
      }
    ]
  },
  "loads": { "gravity": [0.0, -9.81, 0.0] },
  "solver": { "max_iterations": 200, "gradient_tolerance": 0.0001 },
  "output": { "directory": "../outputs/sim/tet-static-dragon", "write_surfaces": true }
}
```

`examples/sim_configs/tet_dynamic_bunny_floor.json` — bunny drops onto an implicit floor (mirrors contact demo §2; bunny bbox min y = 0.0335, floor 0.02 below → 0.013):

```json
{
  "type": "tet",
  "mesh": {
    "volume": "../assets/veg/tet/bunny.veg",
    "surface": "../assets/obj/bunny.obj"
  },
  "material": { "model": "stable_neo" },
  "loads": { "gravity": [0.0, -9.81, 0.0] },
  "initial_state": { "velocity": [0.0, -1.0, 0.0] },
  "contact": [
    { "model": "floor", "axis": "y", "side": "keep_above",
      "height": 0.013, "stiffness": 5000.0 }
  ],
  "solver": { "max_iterations": 200, "gradient_tolerance": 0.0001 },
  "dynamic": { "timestep": 0.001, "num_steps": 80 },
  "output": { "directory": "../outputs/sim/tet-dynamic-bunny-floor", "write_surfaces": true }
}
```

`examples/sim_configs/cubic_static_box_hang.json` — cubic box hangs from its clamped top face (mirrors `static_solve_box_hang_demo`):

```json
{
  "type": "cubic",
  "mesh": {
    "volume": "../assets/veg/cubic/box.veg",
    "surface": "../assets/obj/box.obj"
  },
  "material": { "model": "stable_neo" },
  "constraints": {
    "fixed": { "region": { "axis": "y", "side": "max", "tolerance": 0.001 } }
  },
  "loads": { "gravity": [0.0, -9.81, 0.0] },
  "solver": { "max_iterations": 200, "gradient_tolerance": 0.0001 },
  "output": { "directory": "../outputs/sim/cubic-static-box-hang", "write_surfaces": true }
}
```

`examples/sim_configs/cubic_dynamic_box_ipc.json` — cubic box falls onto `bottom.obj` via IPC (mirrors contact demo §1 `IPC_SCENE`):

```json
{
  "type": "cubic",
  "mesh": {
    "volume": "../assets/veg/cubic/box.veg",
    "surface": "../assets/obj/box.obj"
  },
  "material": { "model": "stable_neo" },
  "loads": { "gravity": [0.0, -9.81, 0.0] },
  "initial_state": { "velocity": [0.0, -3.0, 0.0] },
  "contact": [
    { "model": "ipc", "dhat": 0.002, "dhat_external": 0.005, "kappa": 3000.0,
      "obstacles": [ { "mesh": "../assets/obj/bottom.obj" } ] }
  ],
  "solver": { "max_iterations": 200, "gradient_tolerance": 0.0001 },
  "dynamic": { "timestep": 0.001, "num_steps": 100 },
  "output": { "directory": "../outputs/sim/cubic-dynamic-box-ipc", "write_surfaces": true }
}
```

`examples/sim_configs/shell_static_drape.json` — Koiter shell clamped along its y=max edge sags under gravity (shell.obj lies in the z=0 plane):

```json
{
  "type": "shell",
  "mesh": { "surface": "../assets/obj/shell.obj" },
  "material": {
    "thickness": 0.001, "E_membrane": 1000000.0, "nu_membrane": 0.4,
    "mass": { "density": 1000.0 }
  },
  "constraints": {
    "fixed": { "region": { "axis": "y", "side": "max", "tolerance": 0.000001 } }
  },
  "loads": { "gravity": [0.0, 0.0, -9.81] },
  "solver": { "max_iterations": 200, "gradient_tolerance": 0.0001 },
  "output": { "directory": "../outputs/sim/shell-static-drape", "write_surfaces": true }
}
```

`examples/sim_configs/shell_dynamic_floor.json` — shell falls onto a floor with friction (mirrors contact demo §4 `FRICTIONAL_SCENE`; areal mass 1000·0.001 = 1 kg/m² matches the demo's total_mass=1):

```json
{
  "type": "shell",
  "mesh": { "surface": "../assets/obj/shell.obj" },
  "material": {
    "thickness": 0.001, "E_membrane": 1000000.0, "nu_membrane": 0.4,
    "mass": { "density": 1000.0 }
  },
  "loads": { "gravity": [0.0, 0.0, -9.81] },
  "initial_state": { "velocity": [0.0, 0.0, -2.0] },
  "contact": [
    { "model": "frictional_sampled_penalty", "stiffness": 10.0, "samples": 1,
      "friction_coeff": 0.3, "velocity_eps": 0.0001 },
    { "model": "floor", "axis": "z", "side": "keep_above",
      "height": -0.1, "stiffness": 5000.0 }
  ],
  "solver": { "max_iterations": 200, "gradient_tolerance": 0.0001 },
  "dynamic": { "timestep": 0.0005, "num_steps": 80 },
  "output": { "directory": "../outputs/sim/shell-dynamic-floor", "write_surfaces": true }
}
```

- [ ] **Step 4: Write `examples/sim_configs/README.md`**

```markdown
# Sim CLI example scenes

Each JSON here is a complete scene for one of the `pypgo-sim-*` CLIs.
Paths inside a config resolve relative to the config file, so these run
from any CWD:

```bash
pypgo-sim-tet-static     --config examples/sim_configs/tet_static_dragon.json
pypgo-sim-tet-dynamic    --config examples/sim_configs/tet_dynamic_bunny_floor.json
pypgo-sim-cubic-static   --config examples/sim_configs/cubic_static_box_hang.json
pypgo-sim-cubic-dynamic  --config examples/sim_configs/cubic_dynamic_box_ipc.json
pypgo-sim-shell-static   --config examples/sim_configs/shell_static_drape.json
pypgo-sim-shell-dynamic  --config examples/sim_configs/shell_dynamic_floor.json
```

CLI flags override JSON values (defaults < JSON < CLI), e.g. append
`--num-steps 5 --output-dir /tmp/run` for a quick look.
Every config is exercised end-to-end by
`tests/pypgo/test_sim_cli_examples.py`.

Scene-to-feature map:

| config | features exercised |
|---|---|
| tet_static_dragon | static solve, gravity, soft vertex attachment (region selector) |
| tet_dynamic_bunny_floor | dynamics, FloorEnergy, initial velocity |
| cubic_static_box_hang | static solve, hard fixed DOFs (region selector) |
| cubic_dynamic_box_ipc | dynamics, IPC + static obstacle, initial velocity |
| shell_static_drape | Koiter shell static, shell self-weight, clamped edge |
| shell_dynamic_floor | shell dynamics, shell mass matrix, frictional penalty + floor |

Try `"formulation": "cubic-tricubic-hermite"` in the cubic dynamic scene
for the tricubic Hermite variant (no constraints in that scene, which the
Hermite formulation requires).
```

- [ ] **Step 5: Run the smoke tests**

Run: `python -m pytest tests/pypgo/test_sim_cli_examples.py -q`
Expected: all PASS (2 dynamic steps / 5 static iterations per case; allow a few minutes).

- [ ] **Step 6: Run the full affected test suite**

Run: `python -m pytest tests/pypgo/test_sim_config.py tests/pypgo/test_sim_scene.py tests/pypgo/test_sim_runners.py tests/pypgo/test_sim_cli_examples.py tests/pypgo/test_tool_clis.py -q`
Expected: all PASS.

- [ ] **Step 7: Commit**

```bash
git add examples/sim_configs/ tests/pypgo/test_sim_cli_examples.py
git commit -m "feat(sim-cli): six runnable example scene configs with smoke tests"
```

---

## Feature-parity checklist (verify at the end, no code)

Confirm each `volume_ipc.py` capability has a home in the new suite:
- [ ] formulation choice incl. auto + tricubic hermite → `mesh.formulation`
- [ ] IPC dhat / dhat_external / kappa → `contact[].{dhat,dhat_external,kappa}`
- [ ] gravity → `loads.gravity`
- [ ] solver max iterations / gradient tolerance → `solver.*`
- [ ] per-frame surface OBJ + summary.json → `output.write_surfaces` (+ summary always)
- [ ] Hermite initial-vector padding → `SceneBundle.initial_vector`
- [ ] moving-obstacle time updates → dynamic runner loop
- [ ] `pypgo-volume-ipc` untouched and its test still green (`test_tool_clis.py`)

---

# Addendum (2026-06-11): full coverage of example/ipc legacy cases

Gap analysis against `example/ipc/` (14 legacy C++ cases) found missing features:
moving attachment targets (`movement`), `dump-interval`, `enable-material-max-step`,
batch orchestration (`ipc_batch.json`), and missing scenes (box-squash, box-with-sphere,
shell IPC drop). `ipc-heuristic` (auto dhat/kappa) is NOT exposed in the Python binding —
out of scope, documented as such.

**Additional measured asset facts:**
- `assets/obj/bottom.obj`: 852 verts, floor slab y ∈ [-1.1519, -0.1428], x/z span ±3.9.
- `assets/obj/box-with-sphere.obj`: 3245 verts, y ∈ [-0.5328, 0.9358]; cubic veg `assets/veg/cubic/box-with-sphere.veg`, tet variant exists too.
- `assets/veg/{tet,cubic}/box.veg`: y ∈ [0.3781, 0.8781] (tet 243 verts, cubic 125) — squash clamps region y-min, pushes region y-max with movement [0, -1.0, 0].
- Legacy `fixed-vertices` entries carry `coeff` → they are SOFT attachments; `movement` is a constant velocity on the attachment target. Map squash to two attachments (coeff 5e4), one with movement.
- `VertexAttachment.set_targets(target_positions)` exists (`pypgo/energy/attachment.py:76`) — per-step target updates are cheap.

### Task 7: movement / dump_interval / enable_material_max_step

**Files:**
- Modify: `pypgo/tools/sim/_config.py`, `pypgo/tools/sim/_scene.py`, `pypgo/tools/sim/_runners.py`
- Test: `tests/pypgo/test_sim_config.py`, `tests/pypgo/test_sim_scene.py`, `tests/pypgo/test_sim_runners.py` (append)

Spec:
1. `_config.py`:
   - `AttachmentConfig` gains `movement: tuple[float,float,float] | None = None` (parsed via `_vec3` when present). Validation: if `mode == "static"` and any attachment has a non-None movement with any nonzero component → `ConfigError("attachment movement requires dynamic mode")`.
   - `OutputConfig` gains `dump_interval: int = 1`; validate `>= 1`.
   - `VolumeMaterialConfig` and `ShellMaterialConfig` gain `enable_material_max_step: bool = True`.
2. `_scene.py`:
   - Both builders pass `options=_fem.DeformationOptions(enable_material_max_step=cfg.material.enable_material_max_step)` to `deformation_energy` (enforce_spd stays default True).
   - New dataclass:
     ```python
     @dataclass
     class MovingAttachment:
         energy: object       # VertexAttachment
         velocity: np.ndarray # (3,)
         num_vertices: int
     ```
   - `SceneBundle` gains field `moving_attachments: list` (default empty via constructor arg). In both builders' attachment loops: if `att.movement` is not None and nonzero, append `MovingAttachment(energy, np.asarray(att.movement), idx.size)`.
3. `_runners.py` `run_dynamic`:
   - Before each `sim.step`, for each moving attachment set the end-of-step displacement target:
     ```python
     t_next = sim.state.time + dt
     for ma in bundle.moving_attachments:
         ma.energy.set_targets(np.tile(ma.velocity * t_next, ma.num_vertices))
     ```
   - Surface writing honors dump_interval: `if cfg.output.write_surfaces and frame.frame_index % cfg.output.dump_interval == 0`.

Tests to append (TDD, run before/after):
- config: movement parsed; static+movement rejected; `dump_interval: 0` rejected; `enable_material_max_step` round-trips.
- scene: volume scene with a moving attachment → `len(bundle.moving_attachments) == 1`, velocity correct, plain attachments don't appear there.
- runners: cubic box squash mini-run — both regions attached (coeff 5e4), top region movement [0,-1,0], gravity zero, 2 steps dt 1e-3 → assert run completes, frames accepted, and the mean y-displacement of the pushed vertices is negative (target pulls down).

Commit: `feat(sim-cli): moving attachment targets, dump interval, material max-step toggle`

### Task 8: four legacy-parity example scenes

**Files:**
- Create: `examples/sim_configs/cubic_dynamic_box_squash.json`, `tet_dynamic_box_squash.json`, `cubic_dynamic_box_sphere_ipc.json`, `shell_dynamic_ipc_drop.json`
- Modify: `tests/pypgo/test_sim_cli_examples.py` (add 4 dynamic cases), `examples/sim_configs/README.md`

Scene contents (params from legacy `example/ipc/*/box-ipc.json` etc.):

`cubic_dynamic_box_squash.json` (legacy cubic/box-squash; axes adapted to our box.veg, squash along y):
```json
{
  "type": "cubic",
  "mesh": { "volume": "../assets/veg/cubic/box.veg", "surface": "../assets/obj/box.obj" },
  "material": { "model": "stable_neo", "enable_material_max_step": false },
  "constraints": {
    "attachments": [
      { "vertices": { "region": { "axis": "y", "side": "min", "tolerance": 0.001 } }, "coeff": 50000.0 },
      { "vertices": { "region": { "axis": "y", "side": "max", "tolerance": 0.001 } }, "coeff": 50000.0,
        "movement": [0.0, -1.0, 0.0] }
    ]
  },
  "loads": { "gravity": [0.0, 0.0, 0.0] },
  "contact": [ { "model": "ipc", "dhat": 0.002, "kappa": 3000.0 } ],
  "solver": { "max_iterations": 200, "gradient_tolerance": 0.00001 },
  "dynamic": { "timestep": 0.001, "num_steps": 200 },
  "output": { "directory": "../outputs/sim/cubic-dynamic-box-squash", "write_surfaces": true, "dump_interval": 10 }
}
```
`tet_dynamic_box_squash.json`: identical except `"type": "tet"` and `../assets/veg/tet/box.veg`, output dir `tet-dynamic-box-squash`.

`cubic_dynamic_box_sphere_ipc.json` (legacy cubic/box-with-sphere-lite; self-contact box-with-sphere drops onto bottom.obj):
```json
{
  "type": "cubic",
  "mesh": { "volume": "../assets/veg/cubic/box-with-sphere.veg", "surface": "../assets/obj/box-with-sphere.obj" },
  "material": { "model": "stable_neo", "enable_material_max_step": false },
  "loads": { "gravity": [0.0, -9.81, 0.0] },
  "contact": [
    { "model": "ipc", "dhat": 0.002, "dhat_external": 0.005, "kappa": 3000.0,
      "obstacles": [ { "mesh": "../assets/obj/bottom.obj" } ] }
  ],
  "solver": { "max_iterations": 200, "gradient_tolerance": 0.001 },
  "dynamic": { "timestep": 0.001, "num_steps": 100 },
  "output": { "directory": "../outputs/sim/cubic-dynamic-box-sphere-ipc", "write_surfaces": true, "dump_interval": 10 }
}
```

`shell_dynamic_ipc_drop.json` (legacy shell/shell-drop; IPC + obstacle instead of the penalty+floor variant; explicit dhat/kappa because ipc-heuristic is unavailable in Python):
```json
{
  "type": "shell",
  "mesh": { "surface": "../assets/obj/shell.obj" },
  "material": { "thickness": 0.001, "E_membrane": 1000000.0, "nu_membrane": 0.4,
                "mass": { "density": 1000.0 } },
  "loads": { "gravity": [0.0, -9.81, 0.0] },
  "initial_state": { "velocity": [0.0, -2.0, 0.0] },
  "contact": [
    { "model": "ipc", "dhat": 0.005, "kappa": 1000.0,
      "obstacles": [ { "mesh": "../assets/obj/bottom.obj" } ] }
  ],
  "solver": { "max_iterations": 200, "gradient_tolerance": 0.0001 },
  "dynamic": { "timestep": 0.0005, "num_steps": 80 },
  "output": { "directory": "../outputs/sim/shell-dynamic-ipc-drop", "write_surfaces": true, "dump_interval": 10 }
}
```

Smoke test changes: add the 4 configs to `DYNAMIC_CASES` (modules: cubic_dynamic, tet_dynamic, cubic_dynamic, shell_dynamic). NOTE: smoke override `--num-steps 2` still applies; with dump_interval 10 only frame 0 surface is written — adjust the surface-file assertion accordingly (assert `surface0000.obj` exists when the surface dir exists, which stays valid). README: add the 4 scenes to the table; add a "Legacy example/ipc parity" note listing the mapping and stating that `ipc-heuristic` requires a C++ binding not yet exposed (use explicit dhat/kappa).

Commit: `feat(sim-cli): legacy-parity scenes — box squash, box-with-sphere, shell IPC drop`

### Task 9: pypgo-sim-batch

**Files:**
- Create: `pypgo/tools/sim/batch.py`, `examples/sim_configs/batch.json`
- Modify: `pypgo/tools/sim/__init__.py`, `setup.py`, `tests/pypgo/test_tool_clis.py`, `tests/pypgo/test_sim_cli_examples.py`, `examples/sim_configs/README.md`
- Test: `tests/pypgo/test_sim_batch.py`

Batch JSON schema (paths relative to the batch file):
```json
{
  "cases": {
    "cubic_box_ipc": { "command": "cubic-dynamic", "config": "cubic_dynamic_box_ipc.json",
                        "args": ["--num-steps", "100"] },
    "tet_static_dragon": { "command": "tet-static", "config": "tet_static_dragon.json" }
  },
  "jobs": {
    "all": "all",
    "drops": ["cubic_box_ipc"]
  }
}
```
- `command` ∈ {tet,cubic,shell}-{static,dynamic}; maps to the entry module's `main`.
- `args` (optional list of strings) appended to the case argv.
- CLI: `pypgo-sim-batch --config batch.json [--job NAME] [--list] [--output-root DIR]`.
  - `--list` prints cases and jobs, exits 0.
  - `--job` defaults to `"all"` (error if absent from jobs and not literally the implicit all).
  - `--output-root DIR` appends `--output-dir DIR/<case_name>` to each case argv.
  - Each case runs sequentially via the module main; `SystemExit`/nonzero returns are caught and recorded, the batch continues. Prints a per-case PASS/FAIL table; writes `<output-root>/batch_summary.json` when `--output-root` given. Returns 0 iff all cases passed.
  - Invalid batch config (unknown command, missing config file, unknown job, case name referenced by a job but undefined) → `ConfigError` → exit 2 via parser.error.

`examples/sim_configs/batch.json`: cases for ALL ten scene configs (commands matching their type/mode), jobs: `all`, `static` (3), `dynamic` (7), `squash` (2), `drops` (box_ipc, box_sphere, bunny_floor, shell_ipc_drop, shell_floor).

Tests (`tests/pypgo/test_sim_batch.py`):
- behavioral: write a tmp batch JSON with 2 real cases (cubic_dynamic_box_ipc + tet_static_dragon, args limiting to `--num-steps 2` / `--solver-max-iterations 3`), run `batch.main(["--config", ..., "--output-root", tmp])` → 0; per-case `summary.json` exists under `<root>/<case>/`; `batch_summary.json` lists both as passed.
- failure path: batch with one case pointing at a nonexistent scene config → main returns nonzero (or exits 2 at validation — assert the chosen behavior).
- `--list` smoke.
- structural test for the shipped `examples/sim_configs/batch.json`: every referenced config exists; every command valid; every scene JSON in the directory (except batch.json) appears in at least one case; all job case-names defined.
- `tests/pypgo/test_sim_cli_examples.py::test_all_example_configs_are_covered` must EXCLUDE `batch.json` from the on-disk set.
- `tests/pypgo/test_tool_clis.py::test_setup_declares_sim_console_scripts`: add `pypgo-sim-batch=pypgo.tools.sim.batch:main` to the expected entries.

setup.py: add `"pypgo-sim-batch=pypgo.tools.sim.batch:main"`. `__init__.py`: add `"batch"` to `__all__`. README: batch usage section.

Commit: `feat(sim-cli): pypgo-sim-batch orchestrator with example batch config`
