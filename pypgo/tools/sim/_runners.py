"""Static and dynamic runners — mesh-type independent."""

from __future__ import annotations

import json

import numpy as np

import pypgo.energy as _energy
import pypgo.solver as _solver
from pypgo.animation import write_u_file
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
    # Contact begin_step is dispatched by the C++ stepper on every step
    # (dispatchBeginStep), including moving-obstacle time updates — no
    # Python-side driving needed.

    optimizer = _make_optimizer(cfg)
    frames = []
    for _ in range(cfg.dynamic.num_steps):
        t_next = sim.state.time + dt
        for ma in bundle.moving_attachments:
            ma.energy.set_targets(np.tile(ma.velocity * t_next, ma.num_vertices))
        frame = sim.step(external_force=bundle.gravity_force, optimizer=optimizer)
        frames.append(frame)
        # Surfaces and states are written even for rejected frames — useful when
        # diagnosing divergence (the state is the last accepted one).
        if frame.frame_index % cfg.output.dump_interval == 0:
            if cfg.output.write_surfaces:
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
