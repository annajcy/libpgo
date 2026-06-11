#!/usr/bin/env python3
"""Generate examples/contact_api_demo.ipynb.

Run from the repository root:

    conda run -n libpgo python examples/scripts/generate_contact_api_demo.py
"""

from __future__ import annotations

from examples.scripts.notebook_builder import code, md, repo_root, write_notebook


NOTEBOOK = "contact_api_demo.ipynb"


CELLS = [
    md(
        """
        # Contact API Demo — All Contact Energy Types

        This notebook demonstrates every contact energy type in `pypgo.contact`:
        **IPC**, **FloorEnergy**, **SampledPenalty**, and **FrictionalSampledPenalty**,
        across cubic, tet, and shell meshes.

        Each section is self-contained — you can run any section independently.
        By default every section uses a short preview; flip the `RUN_FULL_SECTION*`
        flag to run the full simulation.
        """
    ),
    md(
        """
        ## Outline

        1. **IPC** — cubic box falls onto `bottom.obj` obstacle (barrier contact, embedded surface)
        2. **FloorEnergy** — bunny falls onto implicit floor plane (embedded surface)
        3. **SampledPenalty** — tet box self-contact + floor (identity surface)
        4. **FrictionalSampledPenalty** — shell falls with friction + floor
        5. **IPC Shell** — shell self-contact + floor (IPC barrier on thin shell)
        6. Summary comparison table
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
        from pypgo.animation import AbcWriter
        from pypgo.mesh import read_obj, SurfaceEmbedding, TriMeshData
        from pypgo.mesh.volume import VolumeMesh, read_veg
        from pypgo.sim import (
            BackwardEulerDynamicStepper,
            DynamicSimulation,
            DynamicState,
        )
        from pypgo.fem import SimulationMesh, KoiterStVKShellMaterial
        from pypgo.sparse import SparseMatrix

        PACKAGE_ROOT = Path(pgo.__file__).resolve().parent.parent
        ASSET_DIR = PACKAGE_ROOT / "examples" / "assets"
        OUTPUT_DIR = PACKAGE_ROOT / "examples" / "outputs" / "contact"
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        """
    ),
    # ──────────────────────────────────────────────────────────────────
    # Section 1: IPC Barrier — Cubic Box with Obstacle
    # ──────────────────────────────────────────────────────────────────
    md(
        """
        ## 1. IPC Barrier — Cubic Box Falls onto `bottom.obj` Obstacle

        Rebuilds the legacy C++ `runIPCSim` box-drop scene using the Python
        API (the same scene ships as `examples/sim_configs/cubic_dynamic_box_ipc.json`).
        A cubic hex mesh (125 verts, embedded surface 194 verts)
        falls under gravity and lands on the static `bottom.obj` obstacle via
        IPC barrier contact. Watch the `y_min` value approach the obstacle
        surface and the `contact_clamp_count` become nonzero.
        """
    ),
    md(
        """
        ### 1.1 Scene Parameters
        """
    ),
    code(
        """
        IPC_SCENE = {
            "cubic_mesh": "box.veg",
            "surface_mesh": "box.obj",
            "external_objects": [
                {"filename": "bottom.obj", "movement": [0.0, 0.0, 0.0]},
            ],
            "g": np.array([0.0, -9.81, 0.0], dtype=np.float64),
            "init_vel": np.array([0.0, -3.0, 0.0], dtype=np.float64),
            "init_disp": np.array([0.0, 0.0, 0.0], dtype=np.float64),
            "scale": 1.0,
            "timestep": 0.001,
            "num_timestep": 2000,
            "damping_params": (0.0, 0.0),
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

        RUN_FULL_SECTION1 = False
        PREVIEW_STEPS = 200
        NUM_STEPS = IPC_SCENE["num_timestep"] if RUN_FULL_SECTION1 else PREVIEW_STEPS

        print("Section 1 — IPC cubic box + bottom.obj obstacle")
        print("  steps this run:", NUM_STEPS)
        """
    ),
    md(
        """
        ### 1.2 Load Assets
        """
    ),
    code(
        """
        CUBIC_BOX = ASSET_DIR / "veg" / "cubic" / IPC_SCENE["cubic_mesh"]
        BOX_SURFACE = ASSET_DIR / "obj" / IPC_SCENE["surface_mesh"]
        BOTTOM_SURFACE = ASSET_DIR / "obj" / IPC_SCENE["external_objects"][0]["filename"]

        veg = read_veg(str(CUBIC_BOX))
        volume = VolumeMesh.from_veg_file(veg)
        volume_mesh = volume.mesh_data
        surface_mesh = read_obj(str(BOX_SURFACE))
        bottom_mesh = read_obj(str(BOTTOM_SURFACE))

        print("volume:", volume.num_vertices, "vertices,", volume.num_elements, "cubes")
        print("surface:", surface_mesh.num_vertices, "vertices,", surface_mesh.num_elements, "triangles")
        print("bottom obstacle:", bottom_mesh.num_vertices, "vertices,", bottom_mesh.num_elements, "triangles")
        """
    ),
    md(
        """
        ### 1.3 FEM Energy and Gravity
        """
    ),
    code(
        """
        sim_mesh = SimulationMesh.create_volumetric(volume)
        deformation = pf.deformation_energy(
            sim_mesh,
            elastic=pf.StableNeo(),
            elastic_field=pf.ElementwiseField(),
            plastic=pf.VolumetricPlasticity(dofs=0),
            plastic_field=pf.ElementwiseField(),
            formulation=pf.CubicLinear(),
            options=pf.DeformationOptions(
                enable_material_max_step=IPC_SCENE["enable_material_max_step"],
            ),
        )

        mass_field = pf.volume_density(volume)
        mass = pf.CubicLinear().mass_matrix(sim_mesh, mass_field)
        gravity_accel_dofs = np.tile(IPC_SCENE["g"], sim_mesh.num_vertices)
        gravity_force = mass @ gravity_accel_dofs

        x0 = np.tile(IPC_SCENE["init_disp"], sim_mesh.num_vertices)
        v0 = np.tile(IPC_SCENE["init_vel"], sim_mesh.num_vertices)
        a0 = np.zeros_like(x0)

        print("simulation DOFs:", deformation.num_dofs)
        print("mass:", mass.shape, "nnz:", mass.nnz)
        """
    ),
    md(
        """
        ### 1.4 IPC Contact with Embedded Surface
        """
    ),
    code(
        """
        surface_embedding = SurfaceEmbedding(surface_mesh, volume)
        contact_surface = pc.ContactSurface.from_surface_embedding(surface_embedding)

        obstacle_specs = [
            pc.ObstacleSpec.static(bottom_mesh.vertices, bottom_mesh.elements)
        ]

        ipc = pc.IPCEnergy(
            contact_surface,
            surface_mesh.elements,
            params=pc.IPCParameters(
                dhat=IPC_SCENE["ipc_dhat"],
                dhat_external=IPC_SCENE["ipc_dhat_external"],
                kappa=IPC_SCENE["ipc_kappa"],
            ),
            obstacles=obstacle_specs,
        )

        print(ipc)
        print("IPC value at x0:", ipc.value(x0))
        print("IPC is step dependent:", ipc.is_step_dependent)

        # Initialize IPC active set and barrier detection
        ipc.begin_step(time=0.0, timestep=IPC_SCENE["timestep"], previous_x=x0)
        """
    ),
    md(
        """
        ### 1.5 Compose and Simulate
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
            timestep=IPC_SCENE["timestep"],
            energy=total_energy,
            integrator=BackwardEulerDynamicStepper(),
            damping=IPC_SCENE["damping_params"],
        )

        optimizer = ps.NewtonOptimizer(
            max_iterations=IPC_SCENE["solver_max_iter"],
            gradient_tolerance=IPC_SCENE["solver_eps"],
            damping=True,
            line_search=ps.Backtrack(),
            sparse_solver=ps.Auto(),
        )

        def surface_disp_s1(vol_disp):
            return surface_embedding.displacement(vol_disp).reshape(-1)

        dumped = [surface_disp_s1(sim.state.displacement)]
        statuses = []

        for step_id in range(NUM_STEPS):
            frame = sim.step(external_force=gravity_force, optimizer=optimizer)
            statuses.append(frame.solver_result.status.name)

            if (step_id + 1) % IPC_SCENE["dump_interval"] == 0 or step_id + 1 == NUM_STEPS:
                dumped.append(surface_disp_s1(frame.displacement))

            if step_id < 5 or (step_id + 1) % 20 == 0 or not frame.accepted:
                y_min = float((volume_mesh.vertices + frame.displacement.reshape((-1, 3)))[:, 1].min())
                print(
                    f"  step {step_id + 1:4d}  time={sim.state.time:.4f}  "
                    f"accepted={frame.accepted}  status={frame.solver_result.status.name}  "
                    f"iters={frame.solver_result.iterations:3d}  y_min={y_min:.6f}  "
                    f"clamps={frame.solver_result.diagnostics.contact_clamp_count}"
                )
            if not frame.accepted:
                print("stopping after rejected step")
                break

        final_state = sim.state
        print("\\naccepted steps:", final_state.timestep_id)
        print("final time:", final_state.time)
        print("unique solver statuses:", sorted(set(statuses)))
        print("dumped frames:", len(dumped))

        # Export Alembic
        abc_path = OUTPUT_DIR / f"{IPC_SCENE['output']}-python.abc"
        AbcWriter.dump(abc_path, "box_ipc_python",
                       rest_positions=surface_mesh.vertices.reshape(-1),
                       triangles=surface_mesh.elements,
                       displacements=dumped)
        print("wrote:", abc_path, f"({abc_path.stat().st_size} bytes)")
        """
    ),
    # ──────────────────────────────────────────────────────────────────
    # Section 2: FloorEnergy — Tet Box on Implicit Floor Plane
    # ──────────────────────────────────────────────────────────────────
    md(
        """
        ## 2. FloorEnergy — Tet Bunny on Implicit Floor Plane

        `FloorEnergy` imposes a quadratic penalty when vertices cross an implicit
        plane. Unlike IPC, there is no barrier — it's a simple half-space penalty
        with a `set_height()` API for animated floors. FloorEnergy is **not**
        step-aware (no `begin_step` needed).

        We use the `bunny.veg` tet mesh with an **embedded** surface (`bunny.obj`)
        to demonstrate that FloorEnergy works with non-identity contact surfaces.
        """
    ),
    md(
        """
        ### 2.1 Parameters
        """
    ),
    code(
        """
        FLOOR_SCENE = {
            "tet_mesh": "bunny.veg",
            "surface_mesh": "bunny.obj",
            "g": np.array([0.0, -9.81, 0.0], dtype=np.float64),
            "timestep": 0.001,
            "num_timestep": 100,
            "solver_eps": 1.0e-4,
            "solver_max_iter": 200,
            "floor_axis": "y",
            "floor_side": "keep_above",
            "floor_offset": 0.02,       # floor placed this far below the surface bbox bottom
            "floor_stiffness": 5000.0,
            "init_vel_y": -1.0,         # initial downward velocity to hit floor sooner
            "dump_interval": 10,
            "output": "bunny-floor",
        }

        RUN_FULL_SECTION2 = False
        PREVIEW_STEPS_S2 = 80
        NUM_STEPS_S2 = FLOOR_SCENE["num_timestep"] if RUN_FULL_SECTION2 else PREVIEW_STEPS_S2

        print("Section 2 — FloorEnergy bunny")
        print("  steps this run:", NUM_STEPS_S2)
        """
    ),
    md(
        """
        ### 2.2 Load Tet Mesh and Build FEM
        """
    ),
    code(
        """
        bunny_veg_path = ASSET_DIR / "veg" / "tet" / FLOOR_SCENE["tet_mesh"]
        bunny_surface_path = ASSET_DIR / "obj" / FLOOR_SCENE["surface_mesh"]

        bunny_veg = read_veg(str(bunny_veg_path))
        bunny_vol = VolumeMesh.from_veg_file(bunny_veg)
        bunny_surf = read_obj(str(bunny_surface_path))

        print("tet volume:", bunny_vol.num_vertices, "vertices,", bunny_vol.num_elements, "tets")
        print("bunny surface:", bunny_surf.num_vertices, "vertices,", bunny_surf.num_elements, "triangles")

        bunny_sim_mesh = SimulationMesh.create_volumetric(bunny_vol)
        bunny_deformation = pf.deformation_energy(
            bunny_sim_mesh,
            elastic=pf.StableNeo(),
            elastic_field=pf.ElementwiseField(),
            plastic=pf.VolumetricPlasticity(dofs=0),
            plastic_field=pf.ElementwiseField(),
            formulation=pf.TetLinear(),
        )

        bunny_mass_field = pf.volume_density(bunny_vol)
        bunny_mass = pf.TetLinear().mass_matrix(bunny_sim_mesh, bunny_mass_field)
        bunny_gravity_accel = np.tile(FLOOR_SCENE["g"], bunny_sim_mesh.num_vertices)
        bunny_gravity_force = bunny_mass @ bunny_gravity_accel

        # Place floor just below the bunny surface's bottom
        surf_y_min = float(bunny_surf.vertices[:, 1].min())
        floor_height = surf_y_min - FLOOR_SCENE["floor_offset"]
        print(f"surface bbox y: [{surf_y_min:.3f}, {float(bunny_surf.vertices[:, 1].max()):.3f}]")
        print(f"floor height: {floor_height:.3f}  (bbox_bottom - {FLOOR_SCENE['floor_offset']})")

        x0_s2 = np.zeros(bunny_deformation.num_dofs, dtype=np.float64)
        v0_s2 = np.zeros_like(x0_s2)
        v0_s2[1::3] = FLOOR_SCENE["init_vel_y"]   # downward initial velocity
        a0_s2 = np.zeros_like(x0_s2)

        print("tet DOFs:", bunny_deformation.num_dofs)
        print("tet mass:", bunny_mass.shape)
        print(f"initial v_y: {FLOOR_SCENE['init_vel_y']}")
        """
    ),
    md(
        """
        ### 2.3 Embedded Contact Surface and FloorEnergy

        FloorEnergy works with **any** `ContactSurface`, including embedded
        (non-identity) surfaces. The surface embedding interpolates volume DOFs
        to surface DOFs before evaluating the floor penalty.
        """
    ),
    code(
        """
        bunny_embedding = SurfaceEmbedding(bunny_surf, bunny_vol)
        bunny_contact_surface = pc.ContactSurface.from_surface_embedding(bunny_embedding)

        floor = pc.FloorEnergy(
            bunny_contact_surface,
            axis=FLOOR_SCENE["floor_axis"],
            side=FLOOR_SCENE["floor_side"],
            height=floor_height,
            stiffness=FLOOR_SCENE["floor_stiffness"],
        )

        print(floor)
        print("floor energy at x0:", floor.value(x0_s2))
        print("floor height:", floor.parameters.height)
        """
    ),
    md(
        """
        ### 2.4 Simulate
        """
    ),
    code(
        """
        s2_energy = pe.EnergySet([
            (bunny_deformation, 1.0),
            (floor, 1.0),
        ])

        s2_sim = DynamicSimulation(
            mass=bunny_mass,
            state=DynamicState(displacement=x0_s2, velocity=v0_s2, acceleration=a0_s2),
            timestep=FLOOR_SCENE["timestep"],
            energy=s2_energy,
            integrator=BackwardEulerDynamicStepper(),
            damping=(0.0, 0.0),
        )

        s2_opt = ps.NewtonOptimizer(
            max_iterations=FLOOR_SCENE["solver_max_iter"],
            gradient_tolerance=FLOOR_SCENE["solver_eps"],
            damping=True,
            line_search=ps.Backtrack(),
            sparse_solver=ps.Auto(),
        )

        def s2_surface_disp(vol_disp):
            return bunny_embedding.displacement(vol_disp).reshape(-1)

        s2_dumped = [s2_surface_disp(s2_sim.state.displacement)]
        s2_statuses = []

        for step_id in range(NUM_STEPS_S2):
            frame = s2_sim.step(external_force=bunny_gravity_force, optimizer=s2_opt)
            s2_statuses.append(frame.solver_result.status.name)

            if (step_id + 1) % FLOOR_SCENE["dump_interval"] == 0 or step_id + 1 == NUM_STEPS_S2:
                s2_dumped.append(s2_surface_disp(frame.displacement))

            if step_id < 5 or (step_id + 1) % 10 == 0 or not frame.accepted:
                y_min = float((bunny_vol.mesh_data.vertices + frame.displacement.reshape((-1, 3)))[:, 1].min())
                print(
                    f"  step {step_id + 1:4d}  time={s2_sim.state.time:.4f}  "
                    f"accepted={frame.accepted}  status={frame.solver_result.status.name}  "
                    f"iters={frame.solver_result.iterations:3d}  y_min={y_min:.6f}"
                )
            if not frame.accepted:
                print("stopping after rejected step")
                break

        print("\\nSection 2 done — accepted steps:", s2_sim.state.timestep_id)
        print("unique solver statuses:", sorted(set(s2_statuses)))

        # Export
        s2_abc = OUTPUT_DIR / f"{FLOOR_SCENE['output']}-python.abc"
        AbcWriter.dump(s2_abc, "floor_bunny",
                       rest_positions=bunny_surf.vertices.reshape(-1),
                       triangles=bunny_surf.elements,
                       displacements=s2_dumped)
        print("wrote:", s2_abc, f"({s2_abc.stat().st_size} bytes)")
        """
    ),
    # ──────────────────────────────────────────────────────────────────
    # Section 3: SampledPenaltyEnergy — Tet Bunny Self-Contact
    # ──────────────────────────────────────────────────────────────────
    md(
        """
        ## 3. SampledPenaltyEnergy — Tet Box Self-Contact + Floor

        `SampledPenaltyEnergy` uses sample points on surface triangles to detect
        and penalize penetration. Unlike IPC's barrier, this is a quadratic penalty.
        It works with any `ContactSurface`: internally the surface positions are
        evaluated as `surface_rest + W @ simulation_displacement`, and contact
        forces/Hessians are pulled back through `W.T`.

        We use `volume.extract_surface_mesh()` which returns the boundary triangles
        referencing the **same** vertex positions as the volume mesh, so this
        section uses the identity surface as the simplest special case.
        """
    ),
    md(
        """
        ### 3.1 Parameters
        """
    ),
    code(
        """
        SAMPLED_SCENE = {
            "tet_mesh": "box.veg",
            "g": np.array([0.0, -9.81, 0.0], dtype=np.float64),
            "timestep": 0.001,
            "num_timestep": 100,
            "solver_eps": 1.0e-4,
            "solver_max_iter": 200,
            "sp_stiffness": 10.0,
            "sp_samples": 1,
            "sp_self_contact": True,
            "sp_external_contact": True,
            "floor_axis": "y",
            "floor_side": "keep_above",
            "floor_height": 0.3,
            "floor_stiffness": 5000.0,
            "dump_interval": 10,
            "output": "tet-box-sampled-penalty",
        }

        RUN_FULL_SECTION3 = False
        PREVIEW_STEPS_S3 = 80
        NUM_STEPS_S3 = SAMPLED_SCENE["num_timestep"] if RUN_FULL_SECTION3 else PREVIEW_STEPS_S3

        print("Section 3 — SampledPenalty tet box + floor")
        print("  steps this run:", NUM_STEPS_S3)
        """
    ),
    md(
        """
        ### 3.2 Load and Setup
        """
    ),
    code(
        """
        s3_veg_path = ASSET_DIR / "veg" / "tet" / SAMPLED_SCENE["tet_mesh"]

        s3_veg = read_veg(str(s3_veg_path))
        s3_vol = VolumeMesh.from_veg_file(s3_veg)
        s3_surface = s3_vol.extract_surface_mesh()

        print("tet volume:", s3_vol.num_vertices, "vertices,", s3_vol.num_elements, "tets")
        print("extracted surface:", s3_surface.num_vertices, "vertices,", s3_surface.num_elements, "triangles")
        print("surface has same #vertices as volume:", s3_surface.num_vertices == s3_vol.num_vertices)

        s3_sim_mesh = SimulationMesh.create_volumetric(s3_vol)
        s3_def = pf.deformation_energy(
            s3_sim_mesh,
            elastic=pf.StableNeo(),
            elastic_field=pf.ElementwiseField(),
            plastic=pf.VolumetricPlasticity(dofs=0),
            plastic_field=pf.ElementwiseField(),
            formulation=pf.TetLinear(),
        )

        s3_mass_field = pf.volume_density(s3_vol)
        s3_mass = pf.TetLinear().mass_matrix(s3_sim_mesh, s3_mass_field)
        s3_gravity_accel = np.tile(SAMPLED_SCENE["g"], s3_sim_mesh.num_vertices)
        s3_gravity = s3_mass @ s3_gravity_accel

        x0_s3 = np.zeros(s3_def.num_dofs, dtype=np.float64)
        v0_s3 = np.zeros_like(x0_s3)
        a0_s3 = np.zeros_like(x0_s3)

        print("tet DOFs:", s3_def.num_dofs)
        """
    ),
    md(
        """
        ### 3.3 Contact Surface + SampledPenalty + Floor
        """
    ),
    code(
        """
        # Identity surface: tet vertices ARE the contact surface vertices.
        # Non-identity embedded surfaces use the same SampledPenaltyEnergy API.
        s3_contact_surface = pc.ContactSurface.identity(s3_surface.vertices)

        sampled_penalty = pc.SampledPenaltyEnergy(
            s3_contact_surface,
            s3_surface.elements,
            params=pc.SampledPenaltyParameters(
                stiffness=SAMPLED_SCENE["sp_stiffness"],
                samples=SAMPLED_SCENE["sp_samples"],
                enable_self_contact=SAMPLED_SCENE["sp_self_contact"],
                enable_external_contact=SAMPLED_SCENE["sp_external_contact"],
            ),
        )

        s3_floor = pc.FloorEnergy(
            s3_contact_surface,
            axis=SAMPLED_SCENE["floor_axis"],
            side=SAMPLED_SCENE["floor_side"],
            height=SAMPLED_SCENE["floor_height"],
            stiffness=SAMPLED_SCENE["floor_stiffness"],
        )

        print(sampled_penalty)
        print("sampled penalty is step dependent:", sampled_penalty.is_step_dependent)
        print("floor energy at x0:", s3_floor.value(x0_s3))

        sampled_penalty.begin_step(time=0.0, timestep=SAMPLED_SCENE["timestep"], previous_x=x0_s3)
        """
    ),
    md(
        """
        ### 3.4 Simulate
        """
    ),
    code(
        """
        s3_energy = pe.EnergySet([
            (s3_def, 1.0),
            (sampled_penalty, 1.0),
            (s3_floor, 1.0),
        ])

        s3_sim = DynamicSimulation(
            mass=s3_mass,
            state=DynamicState(displacement=x0_s3, velocity=v0_s3, acceleration=a0_s3),
            timestep=SAMPLED_SCENE["timestep"],
            energy=s3_energy,
            integrator=BackwardEulerDynamicStepper(),
            damping=(0.0, 0.0),
        )

        s3_opt = ps.NewtonOptimizer(
            max_iterations=SAMPLED_SCENE["solver_max_iter"],
            gradient_tolerance=SAMPLED_SCENE["solver_eps"],
            damping=True,
            line_search=ps.Backtrack(),
            sparse_solver=ps.Auto(),
        )

        s3_dumped_disp = [s3_sim.state.displacement.copy()]
        s3_statuses = []

        for step_id in range(NUM_STEPS_S3):
            frame = s3_sim.step(external_force=s3_gravity, optimizer=s3_opt)
            s3_statuses.append(frame.solver_result.status.name)

            if (step_id + 1) % SAMPLED_SCENE["dump_interval"] == 0 or step_id + 1 == NUM_STEPS_S3:
                s3_dumped_disp.append(frame.displacement.copy())

            if step_id < 5 or (step_id + 1) % 10 == 0 or not frame.accepted:
                y_min = float((s3_surface.vertices + frame.displacement.reshape((-1, 3)))[:, 1].min())
                print(
                    f"  step {step_id + 1:4d}  time={s3_sim.state.time:.4f}  "
                    f"accepted={frame.accepted}  status={frame.solver_result.status.name}  "
                    f"iters={frame.solver_result.iterations:3d}  y_min={y_min:.6f}  "
                    f"clamps={frame.solver_result.diagnostics.contact_clamp_count}"
                )
            if not frame.accepted:
                print("stopping after rejected step")
                break

        print("\\nSection 3 done — accepted steps:", s3_sim.state.timestep_id)
        print("unique solver statuses:", sorted(set(s3_statuses)))

        # Export — this demo uses an identity surface, so displacement IS the surface displacement.
        s3_abc = OUTPUT_DIR / f"{SAMPLED_SCENE['output']}-python.abc"
        AbcWriter.dump(s3_abc, "tet_box_sampled_penalty",
                       rest_positions=s3_surface.vertices.reshape(-1),
                       triangles=s3_surface.elements,
                       displacements=s3_dumped_disp)
        print("wrote:", s3_abc, f"({s3_abc.stat().st_size} bytes)")
        """
    ),
    # ──────────────────────────────────────────────────────────────────
    # Section 4: FrictionalSampledPenaltyEnergy — Shell with Friction
    # ──────────────────────────────────────────────────────────────────
    md(
        """
        ## 4. FrictionalSampledPenaltyEnergy — Shell on Floor with Friction

        `FrictionalSampledPenaltyEnergy` adds Coulomb friction to the sampled
        penalty contact. It **requires** `previous_x` (the displacement from the
        prior timestep) to compute sliding velocity — the framework handles this
        automatically via `begin_step()`.

        We use a Koiter shell mesh (`shell.obj`) with the identity surface as a
        simple special case, and a `FloorEnergy` for the ground plane. Embedded
        surfaces are handled by the same mapped-contact path.
        """
    ),
    md(
        """
        ### 4.1 Parameters
        """
    ),
    code(
        """
        FRICTIONAL_SCENE = {
            "surface_mesh": "shell.obj",
            "shell_thickness": 0.001,
            "shell_E": 1.0e6,
            "shell_nu": 0.4,
            "total_mass": 1.0,
            "g": np.array([0.0, 0.0, -9.81], dtype=np.float64),
            "timestep": 0.0005,
            "num_timestep": 80,
            "solver_eps": 1.0e-4,
            "solver_max_iter": 200,
            "sp_stiffness": 10.0,
            "sp_samples": 1,
            "sp_self_contact": True,
            "sp_external_contact": True,
            "friction_coeff": 0.3,
            "velocity_eps": 1.0e-4,
            "floor_axis": "z",
            "floor_side": "keep_above",
            "floor_height": -0.1,
            "floor_stiffness": 5000.0,
            "lift_height": 2.0,
            "dump_interval": 10,
            "output": "shell-frictional-penalty",
        }

        RUN_FULL_SECTION4 = False
        PREVIEW_STEPS_S4 = 80
        NUM_STEPS_S4 = FRICTIONAL_SCENE["num_timestep"] if RUN_FULL_SECTION4 else PREVIEW_STEPS_S4

        print("Section 4 — FrictionalSampledPenalty shell + floor")
        print("  steps this run:", NUM_STEPS_S4)
        """
    ),
    md(
        """
        ### 4.2 Load Shell Mesh and Build FEM
        """
    ),
    code(
        """
        shell_path = ASSET_DIR / "obj" / FRICTIONAL_SCENE["surface_mesh"]
        shell_mesh = read_obj(str(shell_path))
        n_shell_verts = shell_mesh.num_vertices
        n_shell_dofs = 3 * n_shell_verts

        print("shell:", n_shell_verts, "vertices,", shell_mesh.num_elements, "triangles")

        shell_mat = KoiterStVKShellMaterial(
            thickness=FRICTIONAL_SCENE["shell_thickness"],
            E_membrane=FRICTIONAL_SCENE["shell_E"],
            nu_membrane=FRICTIONAL_SCENE["shell_nu"],
        )
        shell_sim_mesh = SimulationMesh.create_shell(shell_mesh, shell_mat)
        print("shell sim mesh type:", shell_sim_mesh.mesh_type)

        shell_def = pf.deformation_energy(
            shell_sim_mesh,
            elastic=pf.KoiterStVK(),
            elastic_field=pf.ElementwiseField(),
            plastic=pf.ShellPlasticity(dofs=0),
            plastic_field=pf.ElementwiseField(),
            formulation=pf.KoiterShell(),
        )
        print("shell DOFs:", shell_def.num_dofs)
        """
    ),
    md(
        """
        ### 4.3 Lumped Mass Matrix

        Koiter shells do not provide a built-in mass matrix. We construct a
        diagonal lumped mass with uniform per-vertex mass.
        """
    ),
    code(
        """
        total_mass_s4 = FRICTIONAL_SCENE["total_mass"]
        vert_mass_s4 = total_mass_s4 / n_shell_verts
        s4_rows = list(range(n_shell_dofs))
        s4_cols = list(range(n_shell_dofs))
        s4_vals = [vert_mass_s4] * n_shell_dofs

        shell_mass = SparseMatrix.from_coo(
            (n_shell_dofs, n_shell_dofs), s4_rows, s4_cols, s4_vals,
        )
        shell_gravity_accel = np.tile(FRICTIONAL_SCENE["g"], n_shell_verts)
        shell_gravity = shell_mass @ shell_gravity_accel
        print("shell mass:", shell_mass.shape, "per-vertex mass:", vert_mass_s4)
        """
    ),
    md(
        """
        ### 4.4 Initial State, Contact, and Friction
        """
    ),
    code(
        """
        # Identity contact surface — shell vertices ARE simulation DOFs
        s4_contact_surface = pc.ContactSurface.identity(shell_mesh.vertices)

        frictional_penalty = pc.FrictionalSampledPenaltyEnergy(
            s4_contact_surface,
            shell_mesh.elements,
            params=pc.SampledPenaltyParameters(
                stiffness=FRICTIONAL_SCENE["sp_stiffness"],
                samples=FRICTIONAL_SCENE["sp_samples"],
                enable_self_contact=FRICTIONAL_SCENE["sp_self_contact"],
                enable_external_contact=FRICTIONAL_SCENE["sp_external_contact"],
            ),
            friction=pc.FrictionParameters(
                friction_coeff=FRICTIONAL_SCENE["friction_coeff"],
                velocity_eps=FRICTIONAL_SCENE["velocity_eps"],
            ),
        )

        s4_floor = pc.FloorEnergy(
            s4_contact_surface,
            axis=FRICTIONAL_SCENE["floor_axis"],
            side=FRICTIONAL_SCENE["floor_side"],
            height=FRICTIONAL_SCENE["floor_height"],
            stiffness=FRICTIONAL_SCENE["floor_stiffness"],
        )

        # Initial state: lift shell up, tilt slightly
        x0_s4 = np.zeros(n_shell_dofs, dtype=np.float64)
        x0_s4[2::3] = FRICTIONAL_SCENE["lift_height"]

        # Small tilt around Y axis
        y_center = float(shell_mesh.vertices[:, 1].mean())
        tilt_angle = np.radians(2.0)
        for i in range(n_shell_verts):
            x0_s4[3 * i + 2] += (shell_mesh.vertices[i, 1] - y_center) * tilt_angle

        v0_s4 = np.zeros(n_shell_dofs, dtype=np.float64)
        v0_s4[0::3] = 0.2  # small +X velocity
        a0_s4 = np.zeros_like(x0_s4)

        print(frictional_penalty)
        print("frictional penalty is step dependent:", frictional_penalty.is_step_dependent)
        print(f"initial lift: {FRICTIONAL_SCENE['lift_height']}m, tilt: 2°")

        # FrictionalSampledPenalty REQUIRES previous_x for velocity computation
        frictional_penalty.begin_step(time=0.0, timestep=FRICTIONAL_SCENE["timestep"], previous_x=x0_s4)
        """
    ),
    md(
        """
        ### 4.5 Simulate
        """
    ),
    code(
        """
        s4_energy = pe.EnergySet([
            (shell_def, 1.0),
            (frictional_penalty, 1.0),
            (s4_floor, 1.0),
        ])

        s4_sim = DynamicSimulation(
            mass=shell_mass,
            state=DynamicState(displacement=x0_s4, velocity=v0_s4, acceleration=a0_s4),
            timestep=FRICTIONAL_SCENE["timestep"],
            energy=s4_energy,
            integrator=BackwardEulerDynamicStepper(),
            damping=(0.0, 0.0),
        )

        s4_opt = ps.NewtonOptimizer(
            max_iterations=FRICTIONAL_SCENE["solver_max_iter"],
            gradient_tolerance=FRICTIONAL_SCENE["solver_eps"],
            damping=True,
            line_search=ps.Backtrack(),
            sparse_solver=ps.Auto(),
        )

        s4_dumped = [s4_sim.state.displacement.copy()]
        s4_statuses = []

        for step_id in range(NUM_STEPS_S4):
            frame = s4_sim.step(external_force=shell_gravity, optimizer=s4_opt)
            s4_statuses.append(frame.solver_result.status.name)

            if (step_id + 1) % FRICTIONAL_SCENE["dump_interval"] == 0 or step_id + 1 == NUM_STEPS_S4:
                s4_dumped.append(frame.displacement.copy())

            if step_id < 5 or (step_id + 1) % 10 == 0 or not frame.accepted:
                z_min = float((shell_mesh.vertices + frame.displacement.reshape((-1, 3)))[:, 2].min())
                print(
                    f"  step {step_id + 1:4d}  time={s4_sim.state.time:.4f}  "
                    f"accepted={frame.accepted}  status={frame.solver_result.status.name}  "
                    f"iters={frame.solver_result.iterations:3d}  z_min={z_min:.6f}  "
                    f"clamps={frame.solver_result.diagnostics.contact_clamp_count}"
                )
            if not frame.accepted:
                print("stopping after rejected step")
                break

        print("\\nSection 4 done — accepted steps:", s4_sim.state.timestep_id)
        print("unique solver statuses:", sorted(set(s4_statuses)))

        s4_abc = OUTPUT_DIR / f"{FRICTIONAL_SCENE['output']}-python.abc"
        AbcWriter.dump(s4_abc, "shell_frictional",
                       rest_positions=shell_mesh.vertices.reshape(-1),
                       triangles=shell_mesh.elements,
                       displacements=s4_dumped)
        print("wrote:", s4_abc, f"({s4_abc.stat().st_size} bytes)")
        """
    ),
    # ──────────────────────────────────────────────────────────────────
    # Section 5: IPCEnergy — Shell with Static Obstacle
    # ──────────────────────────────────────────────────────────────────
    md(
        """
        ## 5. IPCEnergy — Shell Self-Contact with Floor

        IPC barrier contact on a thin Koiter shell. The shell is lifted, tilted,
        then falls under gravity. IPC handles self-contact (preventing the shell
        from self-intersecting as it folds), while `FloorEnergy` provides the
        ground plane. This demonstrates IPC on a non-volumetric mesh.
        """
    ),
    md(
        """
        ### 5.1 Parameters
        """
    ),
    code(
        """
        SHELL_IPC_SCENE = {
            "surface_mesh": "shell.obj",
            "shell_thickness": 0.001,
            "shell_E": 1.0e6,
            "shell_nu": 0.4,
            "total_mass": 1.0,
            "g": np.array([0.0, 0.0, -9.81], dtype=np.float64),
            "timestep": 0.0005,
            "num_timestep": 80,
            "solver_eps": 1.0e-4,
            "solver_max_iter": 200,
            "ipc_dhat": 0.005,
            "ipc_kappa": 1000.0,
            "lift_height": 1.5,
            "floor_axis": "z",
            "floor_height": -0.1,
            "floor_stiffness": 5000.0,
            "dump_interval": 10,
            "output": "shell-ipc",
        }

        RUN_FULL_SECTION5 = False
        PREVIEW_STEPS_S5 = 80
        NUM_STEPS_S5 = SHELL_IPC_SCENE["num_timestep"] if RUN_FULL_SECTION5 else PREVIEW_STEPS_S5

        print("Section 5 — IPC shell + bottom.obj obstacle")
        print("  steps this run:", NUM_STEPS_S5)
        """
    ),
    md(
        """
        ### 5.2 Load Shell Mesh and Build FEM
        """
    ),
    code(
        """
        s5_shell_path = ASSET_DIR / "obj" / SHELL_IPC_SCENE["surface_mesh"]

        s5_shell_mesh = read_obj(str(s5_shell_path))
        s5_n_verts = s5_shell_mesh.num_vertices
        s5_n_dofs = 3 * s5_n_verts

        print("shell:", s5_n_verts, "vertices,", s5_shell_mesh.num_elements, "triangles")

        s5_shell_mat = KoiterStVKShellMaterial(
            thickness=SHELL_IPC_SCENE["shell_thickness"],
            E_membrane=SHELL_IPC_SCENE["shell_E"],
            nu_membrane=SHELL_IPC_SCENE["shell_nu"],
        )
        s5_sim_mesh = SimulationMesh.create_shell(s5_shell_mesh, s5_shell_mat)

        s5_def = pf.deformation_energy(
            s5_sim_mesh,
            elastic=pf.KoiterStVK(),
            elastic_field=pf.ElementwiseField(),
            plastic=pf.ShellPlasticity(dofs=0),
            plastic_field=pf.ElementwiseField(),
            formulation=pf.KoiterShell(),
        )
        print("shell DOFs:", s5_def.num_dofs)
        """
    ),
    md(
        """
        ### 5.3 Mass Matrix, IPC Contact, and Floor
        """
    ),
    code(
        """
        # Lumped mass
        vert_mass_s5 = SHELL_IPC_SCENE["total_mass"] / s5_n_verts
        s5_rows = list(range(s5_n_dofs))
        s5_cols = list(range(s5_n_dofs))
        s5_vals = [vert_mass_s5] * s5_n_dofs
        s5_mass = SparseMatrix.from_coo(
            (s5_n_dofs, s5_n_dofs), s5_rows, s5_cols, s5_vals)
        s5_gravity_accel = np.tile(SHELL_IPC_SCENE["g"], s5_n_verts)
        s5_gravity = s5_mass @ s5_gravity_accel

        # Identity contact surface for shell
        s5_contact_surface = pc.ContactSurface.identity(s5_shell_mesh.vertices)

        # IPC for self-contact prevention, Floor for ground plane
        s5_ipc = pc.IPCEnergy(
            s5_contact_surface,
            s5_shell_mesh.elements,
            params=pc.IPCParameters(
                dhat=SHELL_IPC_SCENE["ipc_dhat"],
                kappa=SHELL_IPC_SCENE["ipc_kappa"],
            ),
        )

        s5_floor = pc.FloorEnergy(
            s5_contact_surface,
            axis=SHELL_IPC_SCENE["floor_axis"],
            height=SHELL_IPC_SCENE["floor_height"],
            stiffness=SHELL_IPC_SCENE["floor_stiffness"],
        )

        # Initial state: lift shell
        x0_s5 = np.zeros(s5_n_dofs, dtype=np.float64)
        x0_s5[2::3] = SHELL_IPC_SCENE["lift_height"]

        # Small tilt
        y_center_s5 = float(s5_shell_mesh.vertices[:, 1].mean())
        for i in range(s5_n_verts):
            x0_s5[3 * i + 2] += (s5_shell_mesh.vertices[i, 1] - y_center_s5) * np.radians(2.0)

        v0_s5 = np.zeros(s5_n_dofs, dtype=np.float64)
        a0_s5 = np.zeros_like(x0_s5)

        print(s5_ipc)
        print("IPC is step dependent:", s5_ipc.is_step_dependent)
        print(f"initial lift: {SHELL_IPC_SCENE['lift_height']}m")

        s5_ipc.begin_step(time=0.0, timestep=SHELL_IPC_SCENE["timestep"], previous_x=x0_s5)
        """
    ),
    md(
        """
        ### 5.4 Simulate
        """
    ),
    code(
        """
        s5_energy = pe.EnergySet([
            (s5_def, 1.0),
            (s5_ipc, 1.0),
            (s5_floor, 1.0),
        ])

        s5_sim = DynamicSimulation(
            mass=s5_mass,
            state=DynamicState(displacement=x0_s5, velocity=v0_s5, acceleration=a0_s5),
            timestep=SHELL_IPC_SCENE["timestep"],
            energy=s5_energy,
            integrator=BackwardEulerDynamicStepper(),
            damping=(0.0, 0.0),
        )

        s5_opt = ps.NewtonOptimizer(
            max_iterations=SHELL_IPC_SCENE["solver_max_iter"],
            gradient_tolerance=SHELL_IPC_SCENE["solver_eps"],
            damping=True,
            line_search=ps.Backtrack(),
            sparse_solver=ps.Auto(),
        )

        s5_dumped = [s5_sim.state.displacement.copy()]
        s5_statuses = []

        for step_id in range(NUM_STEPS_S5):
            frame = s5_sim.step(external_force=s5_gravity, optimizer=s5_opt)
            s5_statuses.append(frame.solver_result.status.name)

            if (step_id + 1) % SHELL_IPC_SCENE["dump_interval"] == 0 or step_id + 1 == NUM_STEPS_S5:
                s5_dumped.append(frame.displacement.copy())

            if step_id < 5 or (step_id + 1) % 10 == 0 or not frame.accepted:
                z_min = float((s5_shell_mesh.vertices + frame.displacement.reshape((-1, 3)))[:, 2].min())
                print(
                    f"  step {step_id + 1:4d}  time={s5_sim.state.time:.4f}  "
                    f"accepted={frame.accepted}  status={frame.solver_result.status.name}  "
                    f"iters={frame.solver_result.iterations:3d}  z_min={z_min:.6f}  "
                    f"clamps={frame.solver_result.diagnostics.contact_clamp_count}"
                )
            if not frame.accepted:
                print("stopping after rejected step")
                break

        print("\\nSection 5 done — accepted steps:", s5_sim.state.timestep_id)
        print("unique solver statuses:", sorted(set(s5_statuses)))

        s5_abc = OUTPUT_DIR / f"{SHELL_IPC_SCENE['output']}-python.abc"
        AbcWriter.dump(s5_abc, "shell_ipc",
                       rest_positions=s5_shell_mesh.vertices.reshape(-1),
                       triangles=s5_shell_mesh.elements,
                       displacements=s5_dumped)
        print("wrote:", s5_abc, f"({s5_abc.stat().st_size} bytes)")
        """
    ),
    # ──────────────────────────────────────────────────────────────────
    # Section 6: Summary
    # ──────────────────────────────────────────────────────────────────
    md(
        """
        ## 6. Summary

        | # | Contact Type | Mesh | Surface | Obstacle / Floor | Preview Steps | Key Param |
        |---|-------------|------|---------|------------------|---------------|-----------|
        | 1 | IPCEnergy | Cubic hex | Embedded | bottom.obj obstacle | 200 | kappa=3000, dhat=0.002 |
        | 2 | FloorEnergy | Tet | Embedded (bunny.obj) | implicit floor plane | 80 | offset=0.02, v0=-1 m/s |
        | 3 | SampledPenaltyEnergy | Tet | Identity | implicit floor plane | 80 | stiffness=10 |
        | 4 | FrictionalSampledPenalty | Shell | Identity | implicit floor plane | 80 | friction=0.3 |
        | 5 | IPCEnergy | Shell | Identity | implicit floor plane | 80 | kappa=1000, dhat=0.005 |

        ### Key Takeaways

        - **IPCEnergy** works with mesh obstacles (`ObstacleSpec`); Section 1 uses `bottom.obj`.
        - **FloorEnergy** uses an implicit plane (no mesh) and works with any contact surface.
        - **SampledPenaltyEnergy** works with any contact surface (identity or embedded); the demo uses identity for simplicity.
        - **FrictionalSampledPenaltyEnergy** adds Coulomb friction and requires `previous_x`.
        - **FrictionalSampledPenaltyEnergy** requires `previous_x` for friction velocity — the framework handles this automatically.
        - Shell meshes need a manually-constructed lumped mass; `VolumeMesh.mass_matrix()` does not apply.
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "examples" / NOTEBOOK, CELLS)


if __name__ == "__main__":
    main()
