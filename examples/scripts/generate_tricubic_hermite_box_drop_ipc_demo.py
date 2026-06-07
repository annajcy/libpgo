#!/usr/bin/env python3
"""Generate examples/tricubic_hermite_box_drop_ipc_demo.ipynb.

Run from the repository root:

    conda run -n libpgo python examples/scripts/generate_tricubic_hermite_box_drop_ipc_demo.py
"""

from __future__ import annotations

from examples.scripts.notebook_builder import code, md, repo_root, write_notebook


CELLS = [
    md(
        """
        # Tricubic Hermite Dynamic Contact Smoke Test

        This notebook exercises the Hermite dynamic chain:

        ```text
        u_sim (nvtx * 24)
          -> Hermite mass / body force
          -> Hermite surface embedding W
          -> mapped floor + IPC contact
          -> DynamicSimulation
        ```

        The run is intentionally short so it is useful as an executable API
        check, not a production-scale animation.
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
        import pypgo.sim as psim
        import pypgo.solver as ps

        ASSET_DIR = Path(pgo.__file__).resolve().parent.parent / "examples" / "assets"
        BOX_VEG = ASSET_DIR / "veg" / "cubic" / "box.veg"

        volume = pgo.mesh.volume.VolumeMesh.from_veg_file(pgo.mesh.volume.read_veg(str(BOX_VEG)))
        surface = volume.extract_surface_mesh()

        print("volume:", volume.num_vertices, "vertices,", volume.num_elements, "cubes")
        print("surface:", surface.num_vertices, "vertices,", surface.num_elements, "triangles")
        """
    ),
    md(
        """
        ## 1. Hermite Deformation, Mass, and Gravity
        """
    ),
    code(
        """
        formulation = pf.TricubicHermite()
        sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)

        deformation_state = pf.deformation_model_state(
            sim_mesh,
            elastic=pf.StableNeo(),
            elastic_field=pf.ElementwiseField(),
            plastic=pf.VolumetricPlasticity(dofs=0),
            plastic_field=pf.ElementwiseField(),
        )
        deformation = pf.deformation_energy(
            deformation_state,
            formulation=formulation,
            options=pf.DeformationOptions(enable_material_max_step=True),
        )

        mass = formulation.mass_matrix(volume)
        gravity = formulation.body_force(volume, [0.0, -9.8, 0.0])

        print("Hermite DOFs:", deformation.num_dofs)
        print("mass shape:", mass.shape, "nnz:", mass.nnz)
        print("gravity length:", gravity.shape)
        """
    ),
    md(
        """
        ## 2. Hermite Surface Embedding and Mapped Contact
        """
    ),
    code(
        """
        W = formulation.surface_embedding_matrix(volume, surface.vertices)
        contact_surface = pc.ContactSurface.embedded(surface.vertices, W)

        floor_height = float(surface.vertices[:, 1].min() - 0.01)
        floor = pc.FloorEnergy(
            contact_surface,
            axis="y",
            side="keep_above",
            height=floor_height,
            stiffness=1e5,
        )

        # IPC is constructed through the same mapped ContactSurface. For this
        # short smoke run it acts as a self-contact / CCD-aware term; the floor
        # term is what makes the drop setup concrete.
        ipc = pc.IPCEnergy(
            contact_surface,
            surface.elements,
            params=pc.IPCParameters(dhat=0.01, dhat_external=0.01, kappa=0.1),
        )

        print(contact_surface.num_simulation_dofs, "simulation DOFs in W")
        print(floor)
        print(ipc)
        print("floor rest energy:", floor.value(np.zeros(deformation.num_dofs)))
        print("ipc rest energy:", ipc.value(np.zeros(deformation.num_dofs)))
        """
    ),
    md(
        """
        ## 3. A Short Dynamic Step Sequence
        """
    ),
    code(
        """
        total = pe.EnergySet([
            (deformation, 1.0),
            (floor, 1.0),
            (ipc, 1.0),
        ])

        state0 = psim.DynamicState(
            displacement=np.zeros(deformation.num_dofs),
            velocity=np.zeros(deformation.num_dofs),
            acceleration=np.zeros(deformation.num_dofs),
        )

        sim = psim.DynamicSimulation(
            mass=mass,
            state=state0,
            timestep=0.002,
            energy=total,
            integrator=psim.BackwardEulerDynamicStepper(),
            damping=(0.0, 0.0),
        )

        optimizer = ps.NewtonOptimizer(
            max_iterations=30,
            gradient_tolerance=1e-6,
            damping=True,
            sparse_solver="auto",
        )

        frames = []
        for step in range(3):
            frame = sim.step(external_force=gravity, optimizer=optimizer)
            frames.append(frame)
            print(
                step,
                "accepted=", frame.accepted,
                "||u||=", float(np.linalg.norm(frame.displacement)),
                "iters=", frame.solver_result.iterations,
            )

        assert all(frame.accepted for frame in frames)
        """
    ),
    md(
        """
        ## 4. Surface Displacement Check
        """
    ),
    code(
        """
        surface_displacement = (W @ frames[-1].displacement).reshape(-1, 3)
        deformed_surface = pgo.mesh.TriMeshData(
            surface.vertices + surface_displacement,
            surface.elements,
        )

        print("deformed surface bbox:", deformed_surface.bbox)
        print("max surface displacement:", float(np.linalg.norm(surface_displacement, axis=1).max()))
        """
    ),
    md(
        """
        ## Summary

        The important part is the dimensional agreement:

        - deformation energy: `nvtx * 24`
        - mass matrix: `nvtx * 24` by `nvtx * 24`
        - gravity/body force: `nvtx * 24`
        - contact surface map `W`: `(surface_vertices * 3)` by `(nvtx * 24)`

        Contact itself stays surface-position based; Hermite only appears in
        the mass/body-force builders and the embedding map.
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "examples" / "tricubic_hermite_box_drop_ipc_demo.ipynb", CELLS)


if __name__ == "__main__":
    main()
