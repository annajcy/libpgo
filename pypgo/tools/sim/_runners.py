"""Static and dynamic runners — mesh-type independent."""

from __future__ import annotations

import json
import os
import sys
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np

import pypgo.energy as _energy
import pypgo.parallel as _parallel
import pypgo.profiling as _profiling
import pypgo.solver as _solver
from pypgo.animation import AbcWriter, has_animation_io, write_u_file
from pypgo.mesh import read_obj
from pypgo.sim import DynamicSimulation, DynamicState
from pypgo.tools.sim._outputs import write_summary, write_surface
from pypgo.tools.sim._scene import SceneBundle


def _make_optimizer(cfg):
    if cfg.solver.line_search == "simple":
        line_search = _solver.Simple(
            max_iterations=cfg.solver.line_search_max_iterations,
            shrink=cfg.solver.line_search_shrink,
        )
    elif cfg.solver.line_search == "golden":
        line_search = _solver.Golden()
    elif cfg.solver.line_search == "brents":
        line_search = _solver.Brents()
    else:
        line_search = _solver.Backtrack(
            armijo_c=cfg.solver.line_search_armijo_c,
            shrink=cfg.solver.line_search_shrink,
            initial_alpha=cfg.solver.line_search_initial_alpha,
        )

    _SPARSE_SOLVER_MAP = {
        "auto": _solver.Auto,
        "eigen_ldlt": _solver.EigenLDLT,
        "mkl_pardiso": _solver.MKLPardiso,
        "orig_pardiso": _solver.OrigPardiso,
    }
    ss_name = cfg.solver.sparse_solver
    if ss_name not in _SPARSE_SOLVER_MAP:
        raise ValueError(
            f"solver.sparse_solver must be one of {list(_SPARSE_SOLVER_MAP)}, got {ss_name!r}")
    verbose = int(os.environ.get("PGO_SOLVER_VERBOSE", cfg.solver.verbose))
    damping_scale = cfg.solver.damping_scale
    if "PGO_SOLVER_DAMPING_SCALE" in os.environ:
        damping_scale = float(os.environ["PGO_SOLVER_DAMPING_SCALE"])
    if "PGO_SOLVER_DAMPING" in os.environ:
        damping_enabled = _env_bool("PGO_SOLVER_DAMPING", damping_scale is not None)
        damping_scale = (1.0 if damping_scale is None else damping_scale) if damping_enabled else None
    damping = _solver.NoDamping() if damping_scale is None else _solver.FixedDamping(damping_scale)
    return _solver.NewtonOptimizer(
        max_iterations=cfg.solver.max_iterations,
        gradient_tolerance=cfg.solver.gradient_tolerance,
        line_search=line_search,
        damping=damping,
        sparse_solver=_SPARSE_SOLVER_MAP[ss_name](),
        verbose=verbose,
    )


def _env_enabled(name: str) -> bool:
    value = os.environ.get(name)
    return value is not None and value.lower() not in ("", "0", "false", "off", "no")


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.lower() not in ("", "0", "false", "off", "no")


def _write_dynamic_profile_row(path: Path, row: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a") as stream:
        stream.write(json.dumps(row, sort_keys=True) + "\n")


def _dynamic_profile_frame_path(output_dir: Path, frame_index: int) -> Path:
    return Path(output_dir) / "profiles" / f"frame_{frame_index:06d}_profile.json"


def _write_dynamic_profile_frame(path: Path, profile: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as stream:
        json.dump(profile, stream, indent=2, sort_keys=True)
        stream.write("\n")


def _dynamic_profile_index_row(profile: dict, profile_path: Path, output_dir: Path) -> dict:
    diagnostics = profile["solver_diagnostics"]["diagnostics"]
    row = {
        "frame_index": profile["frame_index"],
        "time": profile["time"],
        "step_wall_seconds": profile["step_wall_seconds"],
        "accepted": profile["accepted"],
        "solver_status": profile["solver_status"],
        "solver_iterations": profile["solver_iterations"],
        "final_gradient_max_norm": diagnostics.get("final_gradient_max_norm"),
        "profile_path": profile_path.relative_to(output_dir).as_posix(),
    }
    if "newton_convergence_reason_name" in diagnostics:
        row["newton_convergence_reason_name"] = diagnostics[
            "newton_convergence_reason_name"]
    if "newton_convergence_threshold" in diagnostics:
        row["newton_convergence_threshold"] = diagnostics[
            "newton_convergence_threshold"]
    return row


def _solver_diagnostics(result) -> dict:
    diagnostics = asdict(result.diagnostics)
    return {
        "status": result.status.name,
        "converged": bool(result.converged),
        "iterations": int(result.iterations),
        "raw_status_code": int(result.raw_status_code),
        "final_gradient_norm": result.final_gradient_norm,
        "final_gradient_max_norm": result.final_gradient_max_norm,
        "diagnostics": diagnostics,
    }


def _format_solver_diagnostics(frame) -> str:
    diag = frame.solver_result.diagnostics
    return (
        f"frame={frame.frame_index} accepted={bool(frame.accepted)} "
        f"status={frame.solver_result.status.name} "
        f"iters={int(frame.solver_result.iterations)} "
        f"min_feasible_alpha={diag.min_feasible_alpha:.3e} "
        f"min_line_search_alpha={diag.min_line_search_alpha:.3e} "
        f"min_effective_alpha={diag.min_effective_alpha:.3e} "
        f"contact_clamps={diag.contact_clamp_count} "
        f"material_clamps={diag.material_clamp_count} "
        f"final_grad_max={diag.final_gradient_max_norm}"
    )


_CHECKPOINT_VERSION = 2


def _checkpoint_dir(output_dir: Path) -> Path:
    return Path(output_dir) / "checkpoints"


def _checkpoint_path(output_dir: Path, frame_index: int) -> Path:
    return _checkpoint_dir(output_dir) / f"state{frame_index:04d}.npz"


def _checkpoint_frame_index(path: Path) -> int:
    return int(path.stem.removeprefix("state"))


def _checkpoint_json_value(value):
    if isinstance(value, Path):
        stat = value.stat()
        return {
            "path": str(value.resolve()),
            "size": stat.st_size,
            "mtime_ns": stat.st_mtime_ns,
        }
    if isinstance(value, dict):
        return {key: _checkpoint_json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_checkpoint_json_value(item) for item in value]
    return value


def _checkpoint_metadata(cfg, num_dofs: int) -> dict:
    """Capture every state-affecting setting while allowing a longer target run."""
    simulation = asdict(cfg)
    simulation.pop("output")
    simulation["dynamic"].pop("resume")
    simulation["dynamic"].pop("num_steps")
    return {
        "schema_version": 1,
        "num_dofs": num_dofs,
        "simulation": _checkpoint_json_value(simulation),
        "solver_environment": {
            name: os.environ.get(name)
            for name in ("PGO_SOLVER_DAMPING", "PGO_SOLVER_DAMPING_SCALE")
        },
    }


def _write_checkpoint(
    path: Path,
    *,
    frame,
    state: DynamicState,
    timestep: float,
    integrator: str,
    num_dofs: int,
    metadata: dict,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        path,
        version=np.array(_CHECKPOINT_VERSION, dtype=np.int64),
        displacement=np.asarray(frame.displacement, dtype=np.float64),
        velocity=np.asarray(frame.velocity, dtype=np.float64),
        acceleration=np.asarray(frame.acceleration, dtype=np.float64),
        timestep_id=np.array(state.timestep_id, dtype=np.int64),
        time=np.array(state.time, dtype=np.float64),
        num_dofs=np.array(num_dofs, dtype=np.int64),
        timestep=np.array(timestep, dtype=np.float64),
        integrator=np.array(integrator),
        metadata_json=np.array(json.dumps(metadata, sort_keys=True)),
    )


def _resolve_resume_checkpoint(cfg) -> Path | None:
    resume = cfg.dynamic.resume
    if resume is None:
        return None
    if resume == "latest":
        ckpts = sorted(
            _checkpoint_dir(cfg.output.directory).glob("state*.npz"),
            key=_checkpoint_frame_index,
        )
        if not ckpts:
            from pypgo.tools.sim._config import ConfigError
            raise ConfigError(
                f"dynamic.resume='latest' found no checkpoints in "
                f"{_checkpoint_dir(cfg.output.directory)}")
        return ckpts[-1]
    path = Path(resume)
    if not path.exists():
        from pypgo.tools.sim._config import ConfigError
        raise ConfigError(f"resume checkpoint not found: {path}")
    return path


def _read_checkpoint(
    path: Path,
    *,
    num_dofs: int,
    timestep: float,
    integrator: str,
    metadata: dict,
) -> DynamicState:
    from pypgo.tools.sim._config import ConfigError

    try:
        with np.load(path) as data:
            version = int(data["version"])
            displacement = np.asarray(data["displacement"], dtype=np.float64)
            velocity = np.asarray(data["velocity"], dtype=np.float64)
            acceleration = np.asarray(data["acceleration"], dtype=np.float64)
            timestep_id = int(data["timestep_id"])
            time = float(data["time"])
            ckpt_num_dofs = int(data["num_dofs"])
            ckpt_timestep = float(data["timestep"])
            ckpt_integrator = str(data["integrator"].item())
            ckpt_metadata = json.loads(str(data["metadata_json"].item()))
    except (OSError, KeyError, ValueError) as exc:
        raise ConfigError(f"cannot read resume checkpoint {path}: {exc}") from exc

    if version != _CHECKPOINT_VERSION:
        raise ConfigError(
            f"resume checkpoint version {version} is not supported "
            f"(expected {_CHECKPOINT_VERSION})")
    if ckpt_num_dofs != num_dofs:
        raise ConfigError(
            f"resume checkpoint num_dofs={ckpt_num_dofs} "
            f"does not match scene num_dofs={num_dofs}")
    if not np.isclose(ckpt_timestep, timestep, rtol=0.0, atol=1e-15):
        raise ConfigError(
            f"resume checkpoint timestep={ckpt_timestep} does not match config timestep={timestep}")
    if ckpt_integrator != integrator:
        raise ConfigError(
            f"resume checkpoint integrator={ckpt_integrator!r} "
            f"does not match config integrator={integrator!r}")
    if ckpt_metadata != metadata:
        raise ConfigError(
            "resume checkpoint metadata does not match the current simulation configuration"
        )
    for label, value in (
        ("displacement", displacement),
        ("velocity", velocity),
        ("acceleration", acceleration),
    ):
        if value.shape != (num_dofs,):
            raise ConfigError(
                f"resume checkpoint {label} shape {value.shape} "
                f"does not match ({num_dofs},)")
        if not np.all(np.isfinite(value)):
            raise ConfigError(f"resume checkpoint {label} contains non-finite values")

    return DynamicState(
        displacement=displacement,
        velocity=velocity,
        acceleration=acceleration,
        timestep_id=timestep_id,
        time=time,
    )


def _surface_frame_index(path: Path) -> int:
    return int(path.stem.removeprefix("surface"))


def _rebuild_abc_from_surfaces(bundle: SceneBundle, cfg) -> None:
    surface_dir = cfg.output.directory / "surface"
    surface_paths = sorted(surface_dir.glob("surface*.obj"), key=_surface_frame_index)
    if not surface_paths:
        return
    abc_displacements = []
    for path in surface_paths:
        surface = read_obj(str(path))
        disp = np.asarray(surface.vertices, dtype=np.float64) - bundle.surface_rest
        abc_displacements.append(np.ascontiguousarray(disp, dtype=np.float64).ravel())

    AbcWriter.dump(
        cfg.output.directory / "animation.abc",
        cfg.output.directory.name or "simulation",
        rest_positions=np.ascontiguousarray(bundle.surface_rest, dtype=np.float64).ravel(),
        triangles=bundle.surface_triangles,
        displacements=abc_displacements,
        fps=1.0 / (cfg.output.dump_interval * cfg.dynamic.timestep),
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
    # Static solver has no stepper, so contact state must be initialized here.
    # timestep=1.0 is an arbitrary pseudo-timestep (no time integration in statics).
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
        "final_gradient_max_norm": float(result.final_gradient_max_norm)
        if result.final_gradient_max_norm is not None else None,
        "max_abs_u": float(np.max(np.abs(result.x))),
        "max_fixed_abs_u": float(np.max(np.abs(result.x[bundle.fixed_dofs])))
        if bundle.fixed_dofs is not None else None,
    }
    if cfg.output.write_surfaces:
        write_surface(cfg.output.directory / "final_surface.obj",
                      bundle.surface_positions(result.x), bundle.surface_triangles)
    if cfg.output.write_states:
        states_dir = cfg.output.directory / "states"
        states_dir.mkdir(parents=True, exist_ok=True)
        write_u_file(states_dir / "deform_final.u",
                     np.asarray(result.x, dtype=np.float64).reshape(-1, 1))
    if cfg.output.write_stress:
        stress_dir = cfg.output.directory / "stress"
        stress_dir.mkdir(parents=True, exist_ok=True)
        u_final = np.asarray(result.x, dtype=np.float64)
        values = bundle.deformation.element_von_mises(u_final)
        doc = {
            "frame": 0,
            "time": 0.0,
            "stress_type": "von_mises",
            "location": "element",
            "values": values.tolist(),
        }
        (stress_dir / "von_mises_final.json").write_text(json.dumps(doc))
    if cfg.output.write_abc:
        print("warning: 'write_abc' is ignored in static mode (no animation)",
              file=sys.stderr)
    write_summary(cfg.output.directory, summary)
    return summary


def run_dynamic(bundle: SceneBundle, cfg) -> dict:
    dt = cfg.dynamic.timestep
    x0 = bundle.initial_vector(cfg.initial_state.displacement)
    v0 = bundle.initial_vector(cfg.initial_state.velocity)
    a0 = np.zeros(bundle.num_dofs, dtype=np.float64)
    checkpoint_metadata = _checkpoint_metadata(cfg, bundle.num_dofs)
    resume_path = _resolve_resume_checkpoint(cfg)
    if resume_path is not None:
        resume_state = _read_checkpoint(
            resume_path,
            num_dofs=bundle.num_dofs,
            timestep=dt,
            integrator=cfg.dynamic.integrator,
            metadata=checkpoint_metadata,
        )
        x0 = resume_state.displacement
        v0 = resume_state.velocity
        a0 = resume_state.acceleration
        initial_timestep_id = int(resume_state.timestep_id)
        initial_time = float(resume_state.time)
    else:
        initial_timestep_id = 0
        initial_time = 0.0

    state = DynamicState(
        displacement=x0, velocity=v0,
        acceleration=a0,
        timestep_id=initial_timestep_id,
        time=initial_time)

    energy = _energy.EnergySet(
        bundle.weighted_energies(include_gravity_potential=False))
    sim = DynamicSimulation(
        mass=bundle.mass, state=state, timestep=dt, energy=energy,
        integrator=cfg.dynamic.integrator, damping=cfg.dynamic.damping,
        fixed_dofs=bundle.fixed_dofs.tolist()
        if bundle.fixed_dofs is not None else None,
    )
    # Contact begin_step is dispatched by the C++ stepper on every step
    # (dispatchBeginStep), including moving-obstacle time updates — no
    # Python-side driving needed.

    if cfg.output.write_abc and not has_animation_io():
        from pypgo.tools.sim._config import ConfigError
        raise ConfigError("write_abc requires pypgo built with animation IO (Alembic)")

    optimizer = _make_optimizer(cfg)
    frames = []
    profile_enabled = _env_enabled("PGO_PROFILE_DYNAMIC")
    solve_log_enabled = _env_enabled("PGO_DYNAMIC_SOLVE_LOG")
    output_dir = Path(cfg.output.directory)
    profile_path = output_dir / "profile_dynamic.jsonl"
    if profile_enabled:
        _profiling.set_enabled(True)
        _profiling.reset()
    while int(sim.state.timestep_id) < cfg.dynamic.num_steps:
        if profile_enabled:
            _profiling.reset()
            profile_started = time.perf_counter()
        else:
            profile_started = None
        t_next = sim.state.time + dt
        for ma in bundle.moving_attachments:
            ma.energy.set_targets(np.tile(ma.velocity * t_next, ma.num_vertices))
        frame = sim.step(external_force=bundle.gravity_force, optimizer=optimizer)
        frames.append(frame)
        if solve_log_enabled:
            print("[dynamic] " + _format_solver_diagnostics(frame), flush=True)
        if cfg.output.write_checkpoints:
            _write_checkpoint(
                _checkpoint_path(cfg.output.directory, frame.frame_index),
                frame=frame,
                state=sim.state,
                timestep=dt,
                integrator=cfg.dynamic.integrator,
                num_dofs=bundle.num_dofs,
                metadata=checkpoint_metadata,
            )
        if profile_enabled:
            try:
                frame_profile_path = _dynamic_profile_frame_path(
                    output_dir, frame.frame_index)
                profile = {
                    "frame_index": int(frame.frame_index),
                    "time": float(sim.state.time),
                    "step_wall_seconds": time.perf_counter() - profile_started,
                    "accepted": bool(frame.accepted),
                    "solver_status": frame.solver_result.status.name,
                    "solver_iterations": int(frame.solver_result.iterations),
                    "solver_diagnostics": _solver_diagnostics(frame.solver_result),
                    "stage_diagnostics": [
                        _solver_diagnostics(stage)
                        for stage in frame.stage_results
                    ],
                    "runtime": asdict(_parallel.runtime_info()),
                    "sections": _profiling.snapshot(),
                    "counters": _profiling.snapshot_counters(),
                }
                _write_dynamic_profile_frame(frame_profile_path, profile)
                _write_dynamic_profile_row(
                    profile_path,
                    _dynamic_profile_index_row(
                        profile, frame_profile_path, output_dir),
                )
            except Exception as exc:
                print(f"warning: failed to write dynamic profile: {exc}", file=sys.stderr)
            finally:
                _profiling.reset()
        # Surfaces and states are written even for rejected frames — useful when
        # diagnosing divergence (the state is the last accepted one).
        if frame.frame_index % cfg.output.dump_interval == 0:
            if cfg.output.write_surfaces or cfg.output.write_abc:
                write_surface(
                    cfg.output.directory / "surface" / f"surface{frame.frame_index:04d}.obj",
                    bundle.surface_positions(frame.displacement),
                    bundle.surface_triangles)
            if cfg.output.write_states:
                states_dir = cfg.output.directory / "states"
                states_dir.mkdir(parents=True, exist_ok=True)
                write_u_file(
                    states_dir / f"deform{frame.frame_index:04d}.u",
                    np.asarray(frame.displacement, dtype=np.float64).reshape(-1, 1))
            if cfg.output.write_stress:
                stress_dir = cfg.output.directory / "stress"
                stress_dir.mkdir(parents=True, exist_ok=True)
                u_frame = np.asarray(frame.displacement, dtype=np.float64)
                values = bundle.deformation.element_von_mises(u_frame)
                doc = {
                    "frame": frame.frame_index,
                    "time": float(sim.state.time),
                    "stress_type": "von_mises",
                    "location": "element",
                    "values": values.tolist(),
                }
                (stress_dir / f"von_mises{frame.frame_index:04d}.json").write_text(
                    json.dumps(doc))
        if not frame.accepted:
            break
    if profile_enabled:
        _profiling.set_enabled(False)

    if cfg.output.write_abc:
        _rebuild_abc_from_surfaces(bundle, cfg)

    summary = {
        "mode": "dynamic",
        "mesh_type": cfg.mesh_type,
        "num_dofs": bundle.num_dofs,
        "num_frames": len(frames),
        "resumed_from": str(resume_path) if resume_path is not None else None,
        "initial_timestep_id": initial_timestep_id,
        "target_timestep_id": int(cfg.dynamic.num_steps),
        "final_time": float(sim.state.time),
        "final_timestep_id": int(sim.state.timestep_id),
        "frames": [
            {"frame_index": f.frame_index,
             "accepted": bool(f.accepted),
             "status": f.solver_result.status.name,
             "iterations": int(f.solver_result.iterations),
             "solver_diagnostics": _solver_diagnostics(f.solver_result)}
            for f in frames
        ],
    }
    write_summary(cfg.output.directory, summary)
    return summary
