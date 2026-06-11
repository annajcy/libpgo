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
        # Surfaces are written even for rejected frames — useful when
        # diagnosing divergence (the state is the last accepted one).
        if (cfg.output.write_surfaces
                and frame.frame_index % cfg.output.dump_interval == 0):
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
