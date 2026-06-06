#!/usr/bin/env python3
"""Generate pypgo/examples/dynamic_sim_api_demo.ipynb.

Run from the repository root:

    conda run -n libpgo python pypgo/examples/scripts/generate_dynamic_sim_api_demo.py
"""

from __future__ import annotations

from notebook_builder import code, md, repo_root, write_notebook


CELLS = [
    md(
        """
        # pypgo.sim Dynamic Simulation API Demo

        This tutorial demonstrates the Python dynamic time-stepping API:
        `pypgo.sim.DynamicSimulation`, `DynamicState`, and `DynamicFrame`.

        A `DynamicSimulation` advances an implicit dynamic simulation one time
        step at a time. You give it, **once at construction**:

        - a **mass** matrix,
        - an optional **potential energy** (`pypgo.energy`) plus Rayleigh
          **damping** coefficients,
        - an initial **state** (displacement / velocity / acceleration),
        - a **timestep** and an **integrator** (`"implicit_euler"` or
          `"trbdf2"`),
        - an optional set of **fixed DOFs**.

        Then each call to `step(external_force=..., optimizer=...)` solves one
        implicit stage problem and returns a `DynamicFrame` with the new state
        and solver diagnostics. The simulation owns the dynamic problem; the
        caller supplies the optimizer.
        """
    ),
    md(
        """
        ## Outline

        1. Imports
        2. Free fall: the simplest dynamic problem (no potential energy)
        3. `DynamicState` and `DynamicFrame` fields
        4. Implicit Euler vs TRBDF2 (and `stage_results`)
        5. A mass–spring system using `pypgo.energy`
        6. Fixed DOFs (an anchored node)
        7. Rayleigh damping
        8. Driving many steps with `run`
        9. Visualisation: FEM box drop with Alembic export
        10. Clean API surface
        """
    ),
    code(
        """
        import numpy as np

        import pypgo as pgo
        import pypgo.solver as solver
        from pypgo.sim import DynamicSimulation, DynamicState, DynamicFrame
        """
    ),
    md(
        """
        ## 1. Free fall: the simplest dynamic problem

        Consider a single point mass under constant gravity. There is **no
        potential energy** — only inertia (the mass matrix) and an external
        force. We model one scalar DOF (vertical position) with mass `m`.

        Backward (implicit) Euler with a constant force `f` reduces to the exact
        recurrence

        $$ v_{k+1} = v_k + h\\,\\frac{f}{m}, \\qquad u_{k+1} = u_k + h\\,v_{k+1}, $$

        so we can check the simulator against a hand-rolled loop. (The
        continuous solution is the familiar $u(t) = \\tfrac12\\,\\tfrac{f}{m}\\,t^2$.)
        """
    ),
    code(
        """
        m = 2.0          # mass
        h = 0.02         # timestep
        g = -9.81        # gravity (points down)
        f = g * m        # constant external force = m * g

        sim = DynamicSimulation(
            mass=np.array([[m]]),                       # 1x1 mass matrix
            state=DynamicState(
                displacement=np.zeros(1),
                velocity=np.zeros(1),
                acceleration=np.zeros(1),
            ),
            timestep=h,
            integrator="implicit_euler",
            # energy is omitted: a free particle has no potential energy
        )

        print("num_dofs:", sim.num_dofs)
        print("integrator: implicit_euler")
        """
    ),
    code(
        """
        # Step the simulation and compare against the implicit-Euler recurrence.
        u_ref = v_ref = 0.0
        print(f"{'step':>4} {'time':>6} {'u (sim)':>12} {'u (exact)':>12} {'v (sim)':>10}")
        for k in range(10):
            frame = sim.step(external_force=np.array([f]))
            v_ref += h * f / m
            u_ref += h * v_ref
            print(f"{k:>4} {sim.state.time:6.3f} "
                  f"{frame.displacement[0]:12.6f} {u_ref:12.6f} {frame.velocity[0]:10.5f}")

        print("\\naccepted:", frame.accepted)
        """
    ),
    md(
        """
        The simulator tracks the analytic recurrence closely. (The tiny
        residual difference comes from the Newton solver's Hessian damping,
        which is on by default for robustness; it is not a formula error.)
        """
    ),
    md(
        """
        ## 2. `DynamicState` and `DynamicFrame`

        `sim.state` returns the current `DynamicState`. Each `step` returns a
        `DynamicFrame` describing what happened during that step.
        """
    ),
    code(
        """
        state = sim.state
        print("DynamicState fields:")
        print("  displacement :", state.displacement)
        print("  velocity     :", state.velocity)
        print("  acceleration :", state.acceleration)
        print("  timestep_id  :", state.timestep_id)
        print("  time         :", state.time)

        print("\\nDynamicFrame fields (from the last step):")
        print("  frame_index   :", frame.frame_index)
        print("  accepted      :", frame.accepted)
        print("  solver_result :", frame.solver_result.status,
              "in", frame.solver_result.iterations, "iters")
        print("  stage_results :", [r.status for r in frame.stage_results])
        """
    ),
    md(
        """
        Returned arrays are owned copies — mutating a frame does not corrupt the
        simulator's internal state.
        """
    ),
    code(
        """
        before = sim.state.displacement.copy()
        frame.displacement[:] = 12345.0           # scribble on the returned copy
        after = sim.state.displacement
        print("simulator state unchanged:", np.allclose(before, after))
        """
    ),
    md(
        """
        ## 3. Implicit Euler vs TRBDF2

        `DynamicSimulation` supports two integrators:

        - `"implicit_euler"` — first order, one implicit stage per step.
        - `"trbdf2"` — second order, **two** implicit stages per step (a
          trapezoidal-rule stage followed by a BDF2 stage), for `gamma < 1`.

        `frame.stage_results` exposes the per-stage solver result, so TRBDF2
        reports two entries.
        """
    ),
    code(
        """
        def fresh_free_fall(integrator):
            return DynamicSimulation(
                mass=np.array([[m]]),
                state=DynamicState(np.zeros(1), np.zeros(1), np.zeros(1)),
                timestep=h,
                integrator=integrator,
            )

        ibe = fresh_free_fall("implicit_euler").step(external_force=np.array([f]))
        tr = fresh_free_fall("trbdf2").step(external_force=np.array([f]))

        print("implicit_euler stages:", len(ibe.stage_results))
        print("trbdf2 stages        :", len(tr.stage_results))
        print("trbdf2 stage statuses:", [r.status for r in tr.stage_results])
        """
    ),
    md(
        """
        ## 4. A mass–spring system using `pypgo.energy`

        Now we add a potential energy. A linear spring of stiffness `k` anchored
        at the origin has energy $\\tfrac12 k\\,x^T x$, which is exactly a
        `pypgo.energy.QuadraticEnergy` with `A = k I` and no linear term.

        We drop a 3-DOF mass on this spring under gravity and watch it settle
        toward the static equilibrium where spring force balances gravity:
        $k\\,x^* = f \\Rightarrow x^* = f / k$.
        """
    ),
    code(
        """
        n = 3
        k = 50.0
        spring = pgo.energy.QuadraticEnergy(k * np.eye(n))

        gravity = np.array([0.0, 0.0, -9.81]) * m        # force per (x, y, z) DOF
        equilibrium = gravity / k

        optimizer = solver.NewtonOptimizer(max_iterations=50, gradient_tolerance=1e-9)
        sim = DynamicSimulation(
            mass=m * np.eye(n),
            state=DynamicState(np.zeros(n), np.zeros(n), np.zeros(n)),
            timestep=0.02,
            energy=spring,
            integrator="trbdf2",
            damping=(4.0, 0.0),                          # mass-proportional damping to settle
        )

        for _ in range(400):
            frame = sim.step(external_force=gravity, optimizer=optimizer)

        print("final displacement:", sim.state.displacement)
        print("expected equilibrium:", equilibrium)
        print("close to equilibrium:", np.allclose(sim.state.displacement, equilibrium, atol=2e-3))
        """
    ),
    md(
        """
        ## 5. Fixed DOFs

        Some DOFs may be prescribed (an anchored or scripted node). Pass their
        indices as `fixed_dofs` at construction (the fixed set is immutable for
        the lifetime of the simulation — rebuild to change it). Those DOFs stay
        at their current value unless you override them per step with
        `fixed_values`.
        """
    ),
    code(
        """
        n = 3
        anchored = DynamicSimulation(
            mass=np.eye(n),
            state=DynamicState(
                displacement=np.array([0.0, 0.5, 0.0]),  # DOF 1 starts at 0.5
                velocity=np.zeros(n),
                acceleration=np.zeros(n),
            ),
            timestep=0.05,
            energy=pgo.energy.QuadraticEnergy(10.0 * np.eye(n)),
            fixed_dofs=[1],                               # DOF 1 is pinned
        )
        frame = anchored.step(external_force=np.ones(n))
        print("displacement after step:", frame.displacement)
        print("DOF 1 held at 0.5:", np.isclose(frame.displacement[1], 0.5))
        """
    ),
    md(
        """
        ## 6. Rayleigh damping

        The `damping=(mass_damping, stiffness_damping)` tuple builds a Rayleigh
        damping matrix $D = a\\,M + b\\,K$ applied to the energy. Mass-proportional
        damping (`a`) damps low frequencies; stiffness-proportional damping (`b`)
        damps high frequencies. Below, more damping settles the spring faster.
        """
    ),
    code(
        """
        MAX_STEPS = 600

        def settle_time(mass_damping, steps=MAX_STEPS):
            s = DynamicSimulation(
                mass=np.eye(1),
                state=DynamicState(np.array([1.0]), np.zeros(1), np.zeros(1)),
                timestep=0.02,
                energy=pgo.energy.QuadraticEnergy(np.array([[80.0]])),
                integrator="trbdf2",
                damping=(mass_damping, 0.0),
            )
            for i in range(steps):
                f = s.step()                              # no external force; free vibration
                if abs(f.displacement[0]) < 1e-3 and abs(f.velocity[0]) < 1e-3:
                    return i
            return None                                   # not settled within the budget

        for a in (2.0, 6.0, 12.0):
            n_settle = settle_time(a)
            label = f"{n_settle} steps" if n_settle is not None else f"not within {MAX_STEPS} steps"
            print(f"mass_damping={a:>5}: settled after {label}")
        """
    ),
    md(
        """
        ## 7. Driving many steps with `run`

        `run(num_steps, **step_kwargs)` is a convenience loop that calls `step`
        repeatedly with the same keyword arguments, including an optional
        optimizer, and returns the list of `DynamicFrame`s. `run(0)` is a no-op
        that returns an empty list.
        """
    ),
    code(
        """
        sim = DynamicSimulation(
            mass=np.array([[m]]),
            state=DynamicState(np.zeros(1), np.zeros(1), np.zeros(1)),
            timestep=h,
            integrator="implicit_euler",
        )
        frames = sim.run(5, external_force=np.array([f]))
        print("num frames:", len(frames))
        print("displacements:", [round(float(fr.displacement[0]), 4) for fr in frames])
        print("run(0) returns:", sim.run(0))
        """
    ),
    md(
        """
        ## 8. Clean API surface

        The Python API exposes only `DynamicSimulation.step` / `run` and the
        `DynamicState` / `DynamicFrame` value objects. The C++ layer behind it
        uses `ImplicitEulerStepper` and `TRBDF2Stepper` directly through the
        unified `DynamicStepper` interface. High-level simulation runners build the same stepper
        service from persistent energy terms plus per-frame contact models.

        Contact energies, when added by the contact API, plug into the same
        persistent-energy + per-step lifecycle without changing this surface.
        """
    ),
    code(
        """
        import pypgo.sim as sim_mod

        public = [name for name in dir(sim_mod) if not name.startswith("_")]
        print("pypgo.sim public dynamic API:")
        for name in ("DynamicSimulation", "DynamicState", "DynamicFrame"):
            print("  ", name, "->", name in public)
        """
    ),
    md(
        """
        ## 9. Visualisation: FEM box drop with Alembic export

        Here is the **full pipeline** — load a real tet mesh, build a FEM
        deformation energy, run a dynamic simulation, map volume displacements
        to a display surface via ``SurfaceEmbedding``, and dump an Alembic
        (``.abc``) file viewable in Blender / Maya / Houdini.

        **Steps:**

        1. ``box.veg`` → ``VolumeMesh`` → ``SimulationMesh``
        2. ``deformation_model_state(...)`` + ``deformation_energy(...)``
        3. ``vol.mass_matrix()`` → sparse mass
        4. ``box.obj`` → ``TriMeshData`` (display surface)
        5. ``pgo.mesh.SurfaceEmbedding(surface, vol)`` — one-liner that
           builds the volume‑to‑surface interpolation matrix
        6. ``DynamicSimulation`` with gravity, collect per‑frame displacements
        7. ``AbcWriter(...)``
        """
    ),
    code(
        """
        import os
        from pathlib import Path

        import pypgo.fem as pf
        from pypgo.mesh.volume import VolumeMesh, read_veg

        def _find_repo_root() -> Path:
            cwd = Path.cwd().resolve()
            for candidate in (cwd, *cwd.parents):
                if (candidate / ".git").exists():
                    return candidate
            return cwd

        REPO_ROOT = _find_repo_root()
        ASSET_DIR = REPO_ROOT / "pypgo" / "examples" / "assets"
        VEG_DIR = ASSET_DIR / "veg" / "tet"

        # ── 1. Load box tet mesh ──────────────────────────────────────
        veg = read_veg(str(VEG_DIR / "box.veg"))
        vol = VolumeMesh.from_veg_file(veg)
        sim_mesh = pgo.sim.SimulationMesh.create_volumetric(vol)
        print(f"Tet mesh: {vol.num_vertices} vertices, {vol.num_elements} tets")

        # ── 2. Deformation energy (Stable Neo-Hookean, no plasticity) ─
        deformation_state = pf.deformation_model_state(
            sim_mesh,
            elastic=pf.StableNeo(),
            elastic_field=pf.ElementwiseField(),
            plastic=pf.VolumetricPlasticity(dofs=0),
            plastic_field=pf.ElementwiseField(),
        )
        energy = pf.deformation_energy(
            deformation_state,
            formulation=pf.TetP1(),
        )
        n_dof = energy.num_dofs
        print(f"Energy: {n_dof} DOFs, state_kind={energy.state_kind}")

        # ── 3. Mass matrix ────────────────────────────────────────────
        mass = vol.mass_matrix()

        # ── 4. Display surface & embedding ────────────────────────────
        surface = pgo.mesh.read_obj(str(ASSET_DIR / "obj" / "box.obj"))
        embedding = pgo.mesh.SurfaceEmbedding(surface, vol)
        print(f"Surface: {surface.num_vertices} vertices, {surface.num_elements} triangles")
        print(f"Embedding matrix: {embedding.interpolation_matrix.shape}")

        # ── 5. Build and run dynamic simulation ───────────────────────
        gravity = np.zeros(n_dof, dtype=np.float64)
        gravity[1::3] = -9.81          # y-axis gravity on every vertex

        sim = DynamicSimulation(
            mass=mass,
            state=DynamicState(np.zeros(n_dof), np.zeros(n_dof), np.zeros(n_dof)),
            timestep=0.005,
            energy=energy,
            integrator="implicit_euler",
            damping=(0.5, 0.0),
        )
        optimizer = pgo.solver.NewtonOptimizer(sparse_solver="auto")

        surf_disps = []
        for k in range(200):
            fr = sim.step(external_force=gravity, optimizer=optimizer)
            surf_disps.append(embedding.displacement(fr.displacement).ravel())
        print(f"Simulated {len(surf_disps)} frames")

        # ── 6. Dump Alembic ──────────────────────────────────────────
        out_path = str(REPO_ROOT / "box_fall.abc")
        pgo.animation.AbcWriter.dump(
            out_path, "box_fall",
            rest_positions=surface.vertices.ravel(),
            triangles=surface.elements.ravel(),
            displacements=surf_disps,
        )
        size_kb = os.path.getsize(out_path) / 1024
        print(f"Alembic written: {out_path}  ({size_kb:.1f} KB)")
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "pypgo" / "examples" / "dynamic_sim_api_demo.ipynb", CELLS)


if __name__ == "__main__":
    main()
