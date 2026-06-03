#!/usr/bin/env python3
"""Generate pypgo/examples/static_solve_box_hang_demo.ipynb.

Run from the repository root:

    conda run -n libpgo python pypgo/examples/scripts/generate_static_solve_box_hang_demo.py
"""

from __future__ import annotations

from notebook_builder import code, md, repo_root, write_notebook


CELLS = [
    md(
        """
        # Static Solve — Hanging Cubic Box

        This notebook builds a complete static equilibrium solve using only
        Python-facing APIs and assets from `pypgo/examples/assets`.

        The setup mirrors a "box hanging under gravity" scene:

        1. Load a cubic `.veg` volume mesh.
        2. Build a `SimulationMesh`.
        3. Construct a stable Neo-Hookean deformation energy.
        4. Convert gravity into a linear potential energy.
        5. Fix a small top-corner patch directly in the notebook.
        6. Solve the static energy minimization with Newton.
        7. Visualize the rest and solved shapes.

        This is a **static solve**. It does not use the dynamic time integrator
        or contact/IP C lifecycle.
        """
    ),
    code(
        """
        from pathlib import Path

        import numpy as np
        import pypgo as pgo
        import pypgo.energy as pe
        import pypgo.fem as pf
        import pypgo.solver as ps
        from pypgo import vis
        from pypgo.mesh.veg import VolumeMesh, read_veg
        """
    ),
    md(
        """
        ## 1. Locate the Example Assets

        The notebook intentionally uses `pypgo/examples/assets` rather than
        any simulator-specific example directory. The fixed vertex set is
        derived from the loaded mesh in the next step.
        """
    ),
    code(
        """
        PACKAGE_ROOT = Path(pgo.__file__).resolve().parent
        ASSET_DIR = PACKAGE_ROOT / "examples" / "assets"
        OUTPUT_DIR = PACKAGE_ROOT / "examples" / "outputs"
        CUBIC_BOX = ASSET_DIR / "veg" / "cubic" / "box.veg"
        BOX_SURFACE = ASSET_DIR / "obj" / "box.obj"

        print("volume asset:", CUBIC_BOX)
        print("embedded surface asset:", BOX_SURFACE)
        print("output dir:", OUTPUT_DIR)
        """
    ),
    md(
        """
        ## 2. Load the Cubic Volume Mesh

        `read_veg()` returns a `VegFile` payload. `VolumeMesh.from_veg_file()`
        turns that payload into a solver-facing volumetric mesh with geometry,
        material regions, and mass-matrix support.
        """
    ),
    code(
        """
        veg = read_veg(str(CUBIC_BOX))
        volume = VolumeMesh.from_veg_file(veg)
        cubic_data = volume.mesh_data

        bbox_min, bbox_max = cubic_data.bbox
        embedded_surface = pgo.mesh.read_obj(str(BOX_SURFACE))
        surface_embedding = pgo.mesh.SurfaceEmbedding(embedded_surface, volume)

        x_levels = np.unique(np.round(cubic_data.vertices[:, 0], decimals=12))
        z_levels = np.unique(np.round(cubic_data.vertices[:, 2], decimals=12))
        corner_patch_mask = (
            np.isclose(cubic_data.vertices[:, 1], bbox_max[1])
            & (cubic_data.vertices[:, 0] <= x_levels[1])
            & (cubic_data.vertices[:, 2] <= z_levels[1])
        )
        fixed_vertices = np.flatnonzero(corner_patch_mask).astype(np.int64)

        print(volume)
        print("geometry:", cubic_data.num_vertices, "vertices,", cubic_data.num_elements, "cubes")
        print("material:", volume.material)
        print("bbox:", cubic_data.bbox)
        print("embedded surface:", embedded_surface.num_vertices, "vertices,", embedded_surface.num_elements, "triangles")
        print(
            "surface interpolation:",
            surface_embedding.interpolation_matrix.shape,
            "nnz:",
            surface_embedding.interpolation_matrix.nnz,
        )
        print("fixed top-corner patch vertices:", fixed_vertices.tolist())
        print("num fixed vertices:", len(fixed_vertices))

        vis.plot_volume_surface(cubic_data, titles=["rest cubic box"], show_edges=True)
        """
    ),
    md(
        """
        ## 3. Build the Simulation Mesh and Deformation Energy

        The asset is a cubic/hexahedral mesh, so the formulation is
        `LinearCubic()`. The JSON-style `stable-neo` material maps to
        `pf.StableNeo()`.
        """
    ),
    code(
        """
        sim_mesh = pgo.sim.SimulationMesh.create_volumetric(volume)
        deformation_state = pf.deformation_model_state(
            sim_mesh,
            elastic=pf.StableNeo(),
            elastic_field=pf.ElementwiseField(),
            plastic=pf.VolumetricPlasticity(dofs=6),
            plastic_field=pf.ElementwiseField(),
        )

        deformation = pf.deformation_energy(
            deformation_state,
            formulation=pf.LinearCubic(),
        )

        print("mesh_type:", sim_mesh.mesh_type)
        print("vertices:", sim_mesh.num_vertices)
        print("elements:", sim_mesh.num_elements)
        print("DOFs:", deformation.num_dofs)
        print("state_kind:", deformation.state_kind)
        """
    ),
    md(
        """
        ## 4. Convert Gravity into a Linear Potential

        Static solve minimizes potential energy. If `f` is the gravity force
        vector, the external potential is `-f^T u`, so we pass `-f` to
        `LinearEnergy`.

        For this compact tutorial we densify the small mass matrix. For larger
        meshes, a sparse matvec helper would be preferable.
        """
    ),
    code(
        """
        mass = volume.mass_matrix().to_dense()
        gravity_accel = np.array([0.0, -981, 0.0], dtype=np.float64)
        gravity_accel_dofs = np.tile(gravity_accel, sim_mesh.num_vertices)

        gravity_force = mass @ gravity_accel_dofs
        gravity_energy = pe.LinearEnergy(-gravity_force)

        print("mass matrix:", mass.shape)
        print("gravity force norm:", float(np.linalg.norm(gravity_force)))
        print("linear energy DOFs:", gravity_energy.num_dofs)
        """
    ),
    md(
        """
        ## 5. Convert Fixed Vertices to Fixed DOFs

        Each fixed vertex contributes three displacement DOFs: `x`, `y`, and
        `z`. `OptimizationProblem.fix_variables` stores the prescribed values
        explicitly; here they are zero displacement from `x0`.
        """
    ),
    code(
        """
        fixed_dofs = (3 * fixed_vertices[:, None] + np.arange(3, dtype=np.int64)).ravel()
        x0 = np.zeros(deformation.num_dofs, dtype=np.float64)

        print("fixed DOFs:", fixed_dofs.tolist())
        print("initial fixed values:", x0[fixed_dofs])
        """
    ),
    md(
        """
        ## 6. Compose the Static Objective and Solve

        The static objective is:

        &&
        E(u) = E_\\text{elastic}(u) - f_\\text{gravity}^T u
        &&

        The fixed boundary is handled by equality variable bounds on the
        `OptimizationProblem`, not by adding another energy term.
        """
    ),
    code(
        """
        total_energy = pe.EnergySet([
            (deformation, 1.0),
            (gravity_energy, 1.0),
        ])

        problem = ps.OptimizationProblem(objective=total_energy)
        problem.fix_variables(fixed_dofs.tolist(), x0[fixed_dofs], num_dofs=x0.size)
        optimizer = ps.NewtonOptimizer(
            max_iterations=200,
            gradient_tolerance=1e-4,
            damping=True,
            line_search="backtrack",
        )
        result = optimizer.solve(problem, x0)

        print("status:", result.status.name)
        print("converged:", result.converged)
        print("iterations:", result.iterations)
        print("final objective:", result.final_objective)
        print("final gradient max norm:", result.final_gradient_max_norm)
        print("max |u|:", float(np.max(np.abs(result.x))))
        print("fixed values after solve:", result.x[fixed_dofs])
        """
    ),
    md(
        """
        ## 7. Visualize the Solved Shape

        The solver state is a displacement vector. Reshape it to `(n, 3)` and
        add it to the rest vertices to build a deformed `CubicMeshData`.
        The display surface is driven by `pgo.mesh.SurfaceEmbedding`, which
        preserves the `box.obj` surface topology while interpolating volume
        displacements.
        """
    ),
    code(
        """
        displacement = result.x.reshape((-1, 3))
        deformed_vertices = cubic_data.vertices + displacement
        deformed_cubic = pgo.mesh.CubicMeshData(deformed_vertices, cubic_data.elements)
        deformed_surface = surface_embedding.deform(result.x)

        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        deformed_obj = OUTPUT_DIR / "static_solve_box_hang_deformed.obj"
        pgo.mesh.write_obj(str(deformed_obj), deformed_surface)

        print("rest bbox:    ", cubic_data.bbox)
        print("deformed bbox:", deformed_cubic.bbox)
        print("deformed surface:", deformed_surface.num_vertices, "vertices,", deformed_surface.num_elements, "triangles")
        print("wrote OBJ:", deformed_obj)

        vis.plot_surface(
            [embedded_surface, deformed_surface],
            titles=["embedded rest surface", "embedded static solve"],
            colors=["lightgray", "salmon"],
            show_edges=True,
            window_size=(900, 420),
        )
        """
    ),
    md(
        """
        ## 8. Static Solve with Given Plastic Parameters

        The plastic model creates the per-element plastic field used by the
        FEM energy. For volumetric `dofs=6`, each row is a symmetric plastic
        deformation-gradient parameter:

        $$
        [F_{xx}, F_{xy}, F_{xz}, F_{yy}, F_{yz}, F_{zz}]
        $$

        Here we prescribe a spatially varying field on the same box: elements
        higher in the box want more stretch in `x` and a mild compression in
        `y`.  The static solve still optimizes only displacement `u`; the
        plastic parameters are fixed inputs.
        """
    ),
    code(
        """
        element_centers = cubic_data.vertices[cubic_data.elements].mean(axis=1)
        y01 = (element_centers[:, 1] - bbox_min[1]) / (bbox_max[1] - bbox_min[1])

        plastic_model = pf.VolumetricPlasticity(dofs=6)
        plastic_values = deformation_state.plastic_field.values
        plastic_values[:, 0] = 1.00 + 0.18 * y01  # F_xx
        plastic_values[:, 3] = 1.00 - 0.06 * y01  # F_yy
        plastic_values[:, 5] = 1.00               # F_zz
        deformation_state_spatial = pf.deformation_model_state(
            sim_mesh,
            elastic=pf.StableNeo(),
            elastic_field=pf.ElementwiseField(),
            plastic=plastic_model,
            plastic_field=pf.ElementwiseField(values=plastic_values),
        )

        deformation_plastic = pf.deformation_energy(
            deformation_state_spatial,
            formulation=pf.LinearCubic(),
        )

        print("plastic field shape:", deformation_state_spatial.plastic_field.values.shape)
        print("F_xx range:", float(plastic_values[:, 0].min()), float(plastic_values[:, 0].max()))
        print("F_yy range:", float(plastic_values[:, 3].min()), float(plastic_values[:, 3].max()))
        print("field values match input:", np.allclose(deformation_state_spatial.plastic_field.values, plastic_values))

        total_energy_plastic = pe.EnergySet([
            (deformation_plastic, 1.0)
        ])

        problem_plastic = ps.OptimizationProblem(objective=total_energy_plastic)
        problem_plastic.fix_variables(fixed_dofs.tolist(), x0[fixed_dofs], num_dofs=x0.size)
        result_plastic = optimizer.solve(problem_plastic, x0)

        print("status:", result_plastic.status.name)
        print("converged:", result_plastic.converged)
        print("iterations:", result_plastic.iterations)
        print("final objective:", result_plastic.final_objective)
        print("final gradient max norm:", result_plastic.final_gradient_max_norm)
        print("max |u|:", float(np.max(np.abs(result_plastic.x))))
        print("fixed values after plastic solve:", result_plastic.x[fixed_dofs])
        """
    ),
    code(
        """
        plastic_displacement = result_plastic.x.reshape((-1, 3))
        plastic_deformed_vertices = cubic_data.vertices + plastic_displacement
        plastic_deformed_cubic = pgo.mesh.CubicMeshData(plastic_deformed_vertices, cubic_data.elements)
        plastic_deformed_surface = surface_embedding.deform(result_plastic.x)

        plastic_obj = OUTPUT_DIR / "static_solve_box_hang_plastic_deformed.obj"
        pgo.mesh.write_obj(str(plastic_obj), plastic_deformed_surface)

        print("plastic deformed bbox:", plastic_deformed_cubic.bbox)
        print(
            "plastic deformed surface:",
            plastic_deformed_surface.num_vertices,
            "vertices,",
            plastic_deformed_surface.num_elements,
            "triangles",
        )
        print("wrote plastic OBJ:", plastic_obj)

        vis.plot_surface(
            [embedded_surface, plastic_deformed_surface],
            titles=[
                "embedded rest surface",
                "given plastic params static solve",
            ],
            colors=["lightgray", "cornflowerblue"],
            show_edges=True,
            window_size=(1100, 420),
        )
        """
    ),
    md(
        """
        ## 9. Optional: Soft Pin Energy

        A JSON-style `coeff` pin is a soft penalty. For a hard static boundary,
        prefer hard variable bounds as above. If you want a soft attachment
        term instead, use `VertexAttachment` and include it in the `EnergySet`.
        """
    ),
    code(
        """
        soft_pin = pe.VertexAttachment(
            sim_mesh=sim_mesh,
            vertex_indices=fixed_vertices,
            target_positions=np.zeros(3 * len(fixed_vertices), dtype=np.float64),
            coeff=1e4,
            is_displacement=True,
        )

        print("soft pin:", soft_pin)
        print("soft pin energy at solved state:", soft_pin.value(result.x))

        # add soft pin energy to the total energy and solve again
        total_energy_with_soft_pin = pe.EnergySet([
            (deformation, 1.0),
            (gravity_energy, 1.0),
            (soft_pin, 1.0),
        ])
        problem_with_soft_pin = ps.OptimizationProblem(objective=total_energy_with_soft_pin)
        problem_with_soft_pin.fix_variables(fixed_dofs.tolist(), result.x[fixed_dofs], num_dofs=result.x.size)
        result_with_soft_pin = optimizer.solve(problem_with_soft_pin, result.x)

        print("status:", result_with_soft_pin.status.name)
        print("converged:", result_with_soft_pin.converged)
        print("iterations:", result_with_soft_pin.iterations)
        print("final objective:", result_with_soft_pin.final_objective)
        print("final gradient max norm:", result_with_soft_pin.final_gradient_max_norm)
        print("max |u|:", float(np.max(np.abs(result_with_soft_pin.x))))
        print("soft pin energy at new solved state:", soft_pin.value(result_with_soft_pin.x))

        # visualize the new deformed shape
        displacement_with_soft_pin = result_with_soft_pin.x.reshape((-1, 3))
        deformed_vertices_with_soft_pin = cubic_data.vertices + displacement_with_soft_pin
        deformed_cubic_with_soft_pin = pgo.mesh.CubicMeshData(deformed_vertices_with_soft_pin, cubic_data.elements)
        deformed_surface_with_soft_pin = surface_embedding.deform(result_with_soft_pin.x)

        soft_pin_obj = OUTPUT_DIR / "static_solve_box_hang_soft_pin_deformed.obj"
        pgo.mesh.write_obj(str(soft_pin_obj), deformed_surface_with_soft_pin)

        print("deformed bbox with soft pin:", deformed_cubic_with_soft_pin.bbox)
        print(
            "soft pin deformed surface:",
            deformed_surface_with_soft_pin.num_vertices,
            "vertices,",
            deformed_surface_with_soft_pin.num_elements,
            "triangles",
        )
        print("wrote soft pin OBJ:", soft_pin_obj)
        vis.plot_surface(
            [embedded_surface, deformed_surface_with_soft_pin],
            titles=["embedded rest surface", "embedded soft pin solve"],
            colors=["lightgray", "lightcoral"],
            show_edges=True,
            window_size=(900, 420),
        )
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "pypgo" / "examples" / "static_solve_box_hang_demo.ipynb", CELLS)


if __name__ == "__main__":
    main()
