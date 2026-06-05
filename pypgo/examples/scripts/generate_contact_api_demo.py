#!/usr/bin/env python3
"""Generate pypgo/examples/contact_api_demo.ipynb.

Run from the repository root:

    conda run -n libpgo python pypgo/examples/scripts/generate_contact_api_demo.py
"""

from __future__ import annotations

from notebook_builder import code, md, repo_root, write_notebook


NOTEBOOK = "contact_api_demo.ipynb"


CELLS = [
    md(
        """
        # Contact API Demo - Box IPC Scene

        This notebook rebuilds the scene from
        `examples/ipc/cubic/box/box-ipc.json` using the Python API directly.
        The JSON file is not read by the notebook; its values are written out
        explicitly below.

        The scene is a cubic hexahedral box under gravity, with an embedded
        triangle display/contact surface and one static external obstacle
        (`bottom.obj`). The dynamic objective is

        `implicit dynamics + stable Neo-Hookean elasticity + IPC contact`.
        """
    ),
    md(
        """
        ## Outline

        1. Hard-code the JSON scene parameters.
        2. Load assets from `pypgo/examples/assets`.
        3. Build cubic FEM, mass, gravity, and the embedded contact surface.
        4. Construct a long-lived `pypgo.contact.IPCEnergy`.
        5. Compose deformation and contact in an `EnergySet`.
        6. Run a short preview by default, or flip one flag to run all 2000
           timesteps from the original config.
        7. Export the simulated embedded surface to Alembic.
        """
    ),
    code(
        """
        from pathlib import Path

        import numpy as np

        import pypgo as pgo
        import pypgo.contact as pc
        import pypgo.energy as pe
        import pypgo.fem as pf
        import pypgo.solver as ps
        from pypgo.animation import dump_mesh_animation
        from pypgo.mesh.veg import VolumeMesh, read_veg
        from pypgo.sim import DynamicSimulation, DynamicState
        """
    ),
    md(
        """
        ## 1. Scene Parameters

        These are the values from `box-ipc.json`, copied into Python constants.
        Keeping them in one dictionary makes it easy to compare the notebook
        against the runIPCSim config while still building the scene in memory.
        """
    ),
    code(
        """
        SCENE = {
            "cubic_mesh": "box.veg",
            "surface_mesh": "box.obj",
            "fixed_vertices": [],
            "external_objects": [
                {"filename": "bottom.obj", "movement": [0.0, 0.0, 0.0]},
            ],
            "g": np.array([0.0, -9.81, 0.0], dtype=np.float64),
            "init_vel": np.array([0.0, 0.0, 0.0], dtype=np.float64),
            "init_disp": np.array([0.0, 0.0, 0.0], dtype=np.float64),
            "scale": 1.0,
            "timestep": 0.001,
            "num_timestep": 2000,
            "damping_params": (0.0, 0.0),
            "sim_type": "dynamic",
            "solver_eps": 1.0e-4,
            "solver_max_iter": 200,
            "elastic_material": "stable-neo",
            "dump_interval": 10,
            "output": "ret-box-ipc",
            "ipc_dhat": 0.002,
            "ipc_kappa": 3000.0,
            "ipc_dhat_external": 0.005,
            "enable_material_max_step": False,
        }

        RUN_FULL_BOX_IPC = False
        PREVIEW_STEPS = 80
        NUM_STEPS_TO_RUN = SCENE["num_timestep"] if RUN_FULL_BOX_IPC else PREVIEW_STEPS

        print("dynamic scene:", SCENE["sim_type"])
        print("time step:", SCENE["timestep"])
        print("configured num timesteps:", SCENE["num_timestep"])
        print("steps this notebook will run:", NUM_STEPS_TO_RUN)
        """
    ),
    md(
        """
        ## 2. Load Assets

        The original JSON lives under `examples/ipc/...`, but this notebook uses
        the package example assets so it can run after installation:

        - `assets/veg/cubic/box.veg`
        - `assets/obj/box.obj`
        - `assets/obj/bottom.obj`
        """
    ),
    code(
        """
        PACKAGE_ROOT = Path(pgo.__file__).resolve().parent
        ASSET_DIR = PACKAGE_ROOT / "examples" / "assets"
        OUTPUT_DIR = PACKAGE_ROOT / "examples" / "outputs"
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

        CUBIC_BOX = ASSET_DIR / "veg" / "cubic" / SCENE["cubic_mesh"]
        BOX_SURFACE = ASSET_DIR / "obj" / SCENE["surface_mesh"]
        BOTTOM_SURFACE = ASSET_DIR / "obj" / SCENE["external_objects"][0]["filename"]

        veg = read_veg(str(CUBIC_BOX))
        volume = VolumeMesh.from_veg_file(veg)
        volume_mesh = volume.mesh_data
        surface_mesh = pgo.mesh.read_obj(str(BOX_SURFACE))
        bottom_mesh = pgo.mesh.read_obj(str(BOTTOM_SURFACE))

        print("volume:", volume.num_vertices, "vertices,", volume.num_elements, "cubes")
        print("volume bbox:", volume_mesh.bbox)
        print("surface:", surface_mesh.num_vertices, "vertices,", surface_mesh.num_elements, "triangles")
        print("surface bbox:", surface_mesh.bbox)
        print("bottom:", bottom_mesh.num_vertices, "vertices,", bottom_mesh.num_elements, "triangles")
        print("bottom bbox:", bottom_mesh.bbox)
        """
    ),
    md(
        """
        ## 3. FEM Energy and Gravity

        The JSON material string `stable-neo` maps to `pf.StableNeo()`, and the
        cubic `.veg` mesh uses `pf.LinearCubic()`. `enable-material-max-step`
        is set to `false`, so the deformation energy disables material
        max-step clamping here as well.

        Gravity is passed to the dynamic solver as force, not acceleration:
        `f = M a`.
        """
    ),
    code(
        """
        sim_mesh = pgo.sim.SimulationMesh.create_volumetric(volume)
        deformation_state = pf.deformation_model_state(
            sim_mesh,
            elastic=pf.StableNeo(),
            elastic_field=pf.ElementwiseField(),
            plastic=pf.VolumetricPlasticity(dofs=0),
            plastic_field=pf.ElementwiseField(),
        )
        deformation = pf.deformation_energy(
            deformation_state,
            formulation=pf.LinearCubic(),
            options=pf.DeformationOptions(
                enable_material_max_step=SCENE["enable_material_max_step"],
            ),
        )

        mass = volume.mass_matrix()
        gravity_accel_dofs = np.tile(SCENE["g"], sim_mesh.num_vertices)
        gravity_force = mass @ gravity_accel_dofs

        x0 = np.tile(SCENE["init_disp"], sim_mesh.num_vertices)
        v0 = np.tile(SCENE["init_vel"], sim_mesh.num_vertices)
        a0 = np.zeros_like(x0)

        print("simulation DOFs:", deformation.num_dofs)
        print("mass:", mass.shape, "nnz:", mass.nnz)
        print("gravity force norm:", float(np.linalg.norm(gravity_force)))
        print("initial displacement norm:", float(np.linalg.norm(x0)))
        """
    ),
    md(
        """
        ## 4. Embedded IPC Contact Surface

        `box.obj` has 194 surface vertices while the cubic volume has 125
        simulation vertices. The contact energy must therefore evaluate on the
        surface but pull gradients and Hessians back to volume DOFs.

        `pgo.mesh.SurfaceEmbedding(surface_mesh, volume)` builds that
        interpolation matrix. `pc.ContactSurface.from_surface_embedding(...)`
        turns it into the contact facade used by `IPCEnergy`.
        """
    ),
    code(
        """
        surface_embedding = pgo.mesh.SurfaceEmbedding(surface_mesh, volume)
        contact_surface = pc.ContactSurface.from_surface_embedding(surface_embedding)

        print("surface interpolation:", surface_embedding.interpolation_matrix.shape)
        print("surface interpolation nnz:", surface_embedding.interpolation_matrix.nnz)
        print("contact surface DOFs:", contact_surface.num_surface_dofs)
        print("contact simulation DOFs:", contact_surface.num_simulation_dofs)
        print("matches deformation DOFs:", contact_surface.num_simulation_dofs == deformation.num_dofs)
        """
    ),
    md(
        """
        ## 5. IPC Energy and Static External Obstacle

        The JSON external object has zero movement, so it becomes a static
        `ObstacleSpec`. If an obstacle later uses a nonzero movement vector,
        construct it as `ObstacleSpec.linear_velocity(...)` and update obstacle
        time through the same contact lifecycle.
        """
    ),
    code(
        """
        obstacle_specs = [
            pc.ObstacleSpec.static(bottom_mesh.vertices, bottom_mesh.elements)
        ]

        ipc = pc.IPCEnergy(
            contact_surface,
            surface_mesh.elements,
            params=pc.IPCParameters(
                dhat=SCENE["ipc_dhat"],
                dhat_external=SCENE["ipc_dhat_external"],
                kappa=SCENE["ipc_kappa"],
            ),
            obstacles=obstacle_specs,
        )

        ipc.begin_step(time=0.0, timestep=SCENE["timestep"], previous_x=x0)

        print(ipc)
        print("IPC value at initial state:", ipc.value(x0))
        print("IPC gradient norm at initial state:", float(np.linalg.norm(ipc.gradient(x0))))
        print("IPC is step dependent:", ipc.is_step_dependent)
        """
    ),
    md(
        """
        ## 6. Compose the Dynamic Objective

        Contact is a long-lived energy object. It is added once to the
        `EnergySet`, just like the deformation energy. `DynamicSimulation`
        dispatches `begin_step(...)` through the `EnergySet` at the start of
        each step, and the solver refreshes the contact active set at accepted
        states.
        """
    ),
    code(
        """
        total_energy = pe.EnergySet([
            (deformation, 1.0),
            (ipc, 1.0),
        ])

        sim = DynamicSimulation(
            mass=mass,
            state=DynamicState(
                displacement=x0,
                velocity=v0,
                acceleration=a0,
            ),
            timestep=SCENE["timestep"],
            energy=total_energy,
            integrator="implicit_euler",
            damping=SCENE["damping_params"],
        )

        optimizer = ps.NewtonOptimizer(
            max_iterations=SCENE["solver_max_iter"],
            gradient_tolerance=SCENE["solver_eps"],
            damping=True,
            line_search="backtrack",
            sparse_solver="auto",
        )

        print(total_energy)
        print("dynamic state time:", sim.state.time)
        print("dynamic state timestep_id:", sim.state.timestep_id)
        """
    ),
    md(
        """
        ## 7. Run the Simulation

        The original config requests 2000 steps and dumps every 10 steps. To
        keep the notebook quick by default, it runs `PREVIEW_STEPS`. Set
        `RUN_FULL_BOX_IPC = True` in the parameter cell to run all 2000 steps.
        """
    ),
    code(
        """
        def embedded_surface_displacement(volume_displacement):
            return surface_embedding.displacement(volume_displacement).reshape(-1)


        dumped_displacements = [embedded_surface_displacement(sim.state.displacement)]
        solver_statuses = []

        for step_id in range(NUM_STEPS_TO_RUN):
            frame = sim.step(external_force=gravity_force, optimizer=optimizer)
            solver_statuses.append(frame.solver_result.status.name)

            if (
                (step_id + 1) % SCENE["dump_interval"] == 0
                or step_id + 1 == NUM_STEPS_TO_RUN
            ):
                dumped_displacements.append(embedded_surface_displacement(frame.displacement))

            if step_id < 5 or (step_id + 1) % 20 == 0 or not frame.accepted:
                y_min = float((volume_mesh.vertices + frame.displacement.reshape((-1, 3)))[:, 1].min())
                print(
                    f"step {step_id + 1:4d}  time={sim.state.time:.4f}  "
                    f"accepted={frame.accepted}  status={frame.solver_result.status.name}  "
                    f"iters={frame.solver_result.iterations:3d}  y_min={y_min:.6f}  "
                    f"contact_clamps={frame.solver_result.diagnostics.contact_clamp_count}"
                )

            if not frame.accepted:
                print("stopping after rejected step")
                break

        final_state = sim.state
        final_volume_vertices = volume_mesh.vertices + final_state.displacement.reshape((-1, 3))
        print("\\naccepted steps:", final_state.timestep_id)
        print("final time:", final_state.time)
        print("final volume bbox:", (final_volume_vertices.min(axis=0), final_volume_vertices.max(axis=0)))
        print("unique solver statuses:", sorted(set(solver_statuses)))
        print("dumped frames:", len(dumped_displacements))
        """
    ),
    md(
        """
        ## 8. Export Alembic

        The exported mesh uses the original `box.obj` topology and the embedded
        volume displacement at each dumped frame. The output stem follows the
        JSON `output` field with a Python suffix so it does not collide with
        runIPCSim output.
        """
    ),
    code(
        """
        abc_path = OUTPUT_DIR / f"{SCENE['output']}-python.abc"
        dump_mesh_animation(
            abc_path,
            "box_ipc_python",
            rest_positions=surface_mesh.vertices.reshape(-1),
            displacements=dumped_displacements,
            triangles=surface_mesh.elements,
        )

        print("wrote:", abc_path)
        print("bytes:", abc_path.stat().st_size)
        """
    ),
    md(
        """
        ## 9. What This Matches

        The notebook mirrors the JSON scene at the API level:

        - cubic volume mesh: `box.veg`
        - embedded contact/display surface: `box.obj`
        - external static obstacle: `bottom.obj`
        - gravity: `[0, -9.81, 0]`
        - initial displacement and velocity: zero
        - timestep: `0.001`
        - configured timesteps: `2000`
        - solver tolerance/max iterations: `1e-4` / `200`
        - material: stable Neo-Hookean, cubic formulation
        - IPC: `dhat=0.002`, `dhat_external=0.005`, `kappa=3000.0`
        - material max-step clamping disabled
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "pypgo" / "examples" / NOTEBOOK, CELLS)


if __name__ == "__main__":
    main()
