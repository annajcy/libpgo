#!/usr/bin/env python3
"""Generate pypgo/examples/plastic_shape_match_demo.ipynb."""

from __future__ import annotations

try:
    from .notebook_builder import code, md, repo_root, write_notebook
except ImportError:
    from notebook_builder import code, md, repo_root, write_notebook


def run_demo(volume_veg=None, surface_obj=None, target_obj=None, output_dir=None):
    """Small importable entry point for the generated notebook workflow.

    The notebook itself builds a procedural dense cubic mesh, so the path
    arguments are optional placeholders for larger bunny-style experiments.
    """
    del volume_veg, surface_obj, target_obj, output_dir
    root = repo_root()
    write_notebook(root / "pypgo" / "examples" / "plastic_shape_match_demo.ipynb", CELLS)


CELLS = [
    md(
        """
        # Plastic Field Shape Matching Demo

        This notebook optimizes a volumetric plastic field so a released static
        equilibrium shape moves toward a target. The compact example here uses
        a dense cubic block and a visibly sheared cubic target; the same API is
        intended for larger bunny-style meshes.
        """
    ),
    code(
        """
        from pathlib import Path

        import numpy as np
        import pypgo as pgo
        import pypgo.fem as pf
        import pypgo.solver as ps
        import torch

        PACKAGE_ROOT = Path(pgo.__file__).resolve().parent
        OUTPUT_DIR = PACKAGE_ROOT / "examples" / "outputs"
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        """
    ),
    md(
        """
        ## 1. Build A Dense Cubic Simulation Mesh
        """
    ),
    code(
        """
        nx = ny = nz = 4

        def vertex_id(i, j, k):
            return i * (ny + 1) * (nz + 1) + j * (nz + 1) + k

        cube_vertices = np.array(
            [
                [i / nx, j / ny, k / nz]
                for i in range(nx + 1)
                for j in range(ny + 1)
                for k in range(nz + 1)
            ],
            dtype=np.float64,
        )
        cube_elements = np.array(
            [
                [
                    vertex_id(i, j, k),
                    vertex_id(i + 1, j, k),
                    vertex_id(i + 1, j + 1, k),
                    vertex_id(i, j + 1, k),
                    vertex_id(i, j, k + 1),
                    vertex_id(i + 1, j, k + 1),
                    vertex_id(i + 1, j + 1, k + 1),
                    vertex_id(i, j + 1, k + 1),
                ]
                for i in range(nx)
                for j in range(ny)
                for k in range(nz)
            ],
            dtype=np.int64,
        )

        cubic = pgo.mesh.CubicMeshData(cube_vertices, cube_elements)
        initial_plastic = np.zeros((cube_elements.shape[0], 6), dtype=np.float64)
        initial_plastic_magnitude = np.linalg.norm(initial_plastic, axis=1)
        volume = pgo.mesh.veg.VolumeMesh.create_from_single_material(
            cubic, pgo.mesh.veg.ENuMaterial(E=1e6, nu=0.45)
        )
        sim = pgo.sim.SimulationMesh.create_volumetric(volume)

        state = pf.deformation_model_state(
            sim,
            elastic=pf.StVK(),
            elastic_field=pf.ElementwiseField(),
            plastic=pf.VolumetricPlasticity(dofs=6),
            plastic_field=pf.ElementwiseField(),
        )
        energy = pf.deformation_energy(
            state,
            formulation=pf.LinearCubic(),
            options=pf.DeformationOptions(enforce_spd=False, enable_material_max_step=False),
        )
        print("plastic field:", state.plastic_field.values.shape)
        print("grid:", nx, "x", ny, "x", nz, "cubic elements")
        print("deformation dofs:", energy.num_dofs, "plastic dofs:", energy.num_plastic_dofs)
        pgo.vis.plot_volume_surface(cubic, titles=["rest cubic volume"], show_edges=True)
        """
    ),
    md(
        """
        ## 2. Define A Dense Surface And Sheared Target
        """
    ),
    code(
        """
        boundary_mask = np.any(
            np.isclose(cube_vertices, 0.0) | np.isclose(cube_vertices, 1.0),
            axis=1,
        )
        surface_vertex_ids = np.flatnonzero(boundary_mask).astype(np.int64)
        surface_lut = {int(global_id): local_id for local_id, global_id in enumerate(surface_vertex_ids)}
        surface_triangles = []

        def add_surface_quad(a, b, c, d):
            surface_triangles.append([surface_lut[int(a)], surface_lut[int(b)], surface_lut[int(c)]])
            surface_triangles.append([surface_lut[int(a)], surface_lut[int(c)], surface_lut[int(d)]])

        for i in range(nx):
            for j in range(ny):
                add_surface_quad(vertex_id(i, j, 0), vertex_id(i + 1, j, 0), vertex_id(i + 1, j + 1, 0), vertex_id(i, j + 1, 0))
                add_surface_quad(vertex_id(i, j, nz), vertex_id(i, j + 1, nz), vertex_id(i + 1, j + 1, nz), vertex_id(i + 1, j, nz))
        for i in range(nx):
            for k in range(nz):
                add_surface_quad(vertex_id(i, 0, k), vertex_id(i, 0, k + 1), vertex_id(i + 1, 0, k + 1), vertex_id(i + 1, 0, k))
                add_surface_quad(vertex_id(i, ny, k), vertex_id(i + 1, ny, k), vertex_id(i + 1, ny, k + 1), vertex_id(i, ny, k + 1))
        for j in range(ny):
            for k in range(nz):
                add_surface_quad(vertex_id(0, j, k), vertex_id(0, j + 1, k), vertex_id(0, j + 1, k + 1), vertex_id(0, j, k + 1))
                add_surface_quad(vertex_id(nx, j, k), vertex_id(nx, j, k + 1), vertex_id(nx, j + 1, k + 1), vertex_id(nx, j + 1, k))

        surface = pgo.mesh.TriMeshData(
            cube_vertices[surface_vertex_ids],
            np.asarray(surface_triangles, dtype=np.int64),
        )
        shear_strength = 0.35
        depth_shear = 0.15
        target_vertices = surface.vertices.copy()
        target_vertices[:, 0] += shear_strength * surface.vertices[:, 1]
        target_vertices[:, 2] += depth_shear * surface.vertices[:, 1]
        target_surface = pgo.mesh.TriMeshData(target_vertices, surface.elements)

        pgo.vis.plot_surface(
            [surface, target_surface],
            titles=["rest cubic surface", "sheared cubic target"],
            colors=["lightgray", "palegreen"],
            show_edges=True,
            window_size=(900, 420),
        )

        fixed_vertices = np.flatnonzero(np.isclose(cube_vertices[:, 1], 0.0)).astype(np.int64)
        fixed_dofs = (3 * fixed_vertices[:, None] + np.arange(3, dtype=np.int64)).ravel()
        fixed_values = np.zeros(fixed_dofs.size, dtype=np.float64)
        free_dofs = np.setdiff1d(np.arange(energy.num_dofs, dtype=np.int64), fixed_dofs)
        inner_optimizer = ps.NewtonOptimizer(max_iterations=10, damping=False)
        l2_weight = 1e-4
        a0 = state.plastic_field.values.ravel().copy()
        a0_torch = torch.as_tensor(a0, dtype=torch.float64)
        target_vertices_torch = torch.as_tensor(target_vertices, dtype=torch.float64)
        equilibrium_layer = pgo.torch.StaticEquilibriumLayer(
            state=state,
            energy=energy,
            fixed_dofs=fixed_dofs,
            fixed_values=fixed_values,
            surface_vertices=surface.vertices,
            surface_vertex_ids=surface_vertex_ids,
            inner_optimizer=inner_optimizer,
        )
        print("surface vertices:", surface.num_vertices, "surface triangles:", surface.num_elements)
        print("fixed bottom vertices:", fixed_vertices.size)
        """
    ),
    md(
        """
        ## 3. Evaluate The Adjoint Objective
        """
    ),
    code(
        """
        def vertex_error_stats(residual):
            vertex_errors = np.linalg.norm(residual, axis=1)
            global_error = float(np.linalg.norm(residual.ravel()))
            return {
                "global_error": global_error,
                "per_vertex_rms": global_error / np.sqrt(residual.shape[0]),
                "mean_vertex_error": float(vertex_errors.mean()),
                "max_vertex_error": float(vertex_errors.max()),
            }


        def print_error_stats(label, stats):
            print(
                label,
                "global:",
                stats["global_error"],
                "per-vertex RMS:",
                stats["per_vertex_rms"],
                "mean vertex:",
                stats["mean_vertex_error"],
                "max vertex:",
                stats["max_vertex_error"],
            )


        plastic_param = torch.tensor(a0.copy(), dtype=torch.float64, requires_grad=True)
        initial_vertices = equilibrium_layer(plastic_param)
        initial_residual = initial_vertices - target_vertices_torch
        initial_shape_loss = 0.5 * torch.sum(initial_residual ** 2)
        initial_regularization = 0.5 * l2_weight * torch.sum((plastic_param - a0_torch) ** 2)
        initial_loss = initial_shape_loss + initial_regularization
        initial_loss.backward()

        print("initial objective:", float(initial_loss.detach()))
        print_error_stats("initial shape error", vertex_error_stats(initial_residual.detach().numpy()))
        print("gradient norm:", float(torch.linalg.norm(plastic_param.grad)))

        initial_surface = pgo.mesh.TriMeshData(initial_vertices.detach().numpy(), surface.elements)
        pgo.vis.plot_surface(
            [surface, target_surface, initial_surface],
            titles=["rest", "target", "initial equilibrium"],
            colors=["lightgray", "palegreen", "lightskyblue"],
            show_edges=True,
            window_size=(1200, 420),
        )
        """
    ),
    md(
        """
        ## 4. PyTorch Outer Optimization
        """
    ),
    code(
        """
        num_outer_steps = 60
        learning_rate = 0.01
        plastic_param = torch.tensor(a0.copy(), dtype=torch.float64, requires_grad=True)
        outer_optimizer = torch.optim.Adam([plastic_param], lr=learning_rate)
        history = []
        best_value = np.inf
        best_plastic = a0.copy()

        for iteration in range(num_outer_steps):
            outer_optimizer.zero_grad()
            solved_vertices = equilibrium_layer(plastic_param)
            residual = solved_vertices - target_vertices_torch
            shape_loss = 0.5 * torch.sum(residual ** 2)
            regularization = 0.5 * l2_weight * torch.sum((plastic_param - a0_torch) ** 2)
            loss = shape_loss + regularization
            loss.backward()

            value = float(loss.detach())
            stats = vertex_error_stats(residual.detach().numpy())
            if value < best_value:
                best_value = value
                best_plastic = plastic_param.detach().numpy().copy()
            outer_optimizer.step()
            history.append((value, stats["global_error"], stats["mean_vertex_error"]))
            print(
                "torch step",
                iteration + 1,
                "objective:",
                value,
                "global error:",
                stats["global_error"],
                "mean vertex:",
                stats["mean_vertex_error"],
                "grad:",
                float(torch.linalg.norm(plastic_param.grad)),
            )

        optimized_plastic_tensor = torch.as_tensor(best_plastic, dtype=torch.float64)
        equilibrium_layer.reset_warm_start()
        optimized_vertices = equilibrium_layer(optimized_plastic_tensor).detach().numpy().copy()
        final_residual = optimized_vertices - target_vertices
        final_shape_value = 0.5 * float(np.dot(final_residual.ravel(), final_residual.ravel()))
        final_regularization = 0.5 * l2_weight * float(np.dot(best_plastic - a0, best_plastic - a0))
        final_value = final_shape_value + final_regularization

        print("final objective:", final_value)
        print_error_stats("final shape error", vertex_error_stats(final_residual))
        optimized_plastic = best_plastic.reshape(state.plastic_field.values.shape)
        plastic_delta = optimized_plastic - initial_plastic
        initial_plastic_norm = np.linalg.norm(initial_plastic, axis=1)
        optimized_plastic_norm = np.linalg.norm(optimized_plastic, axis=1)
        plastic_delta_norm = np.linalg.norm(plastic_delta, axis=1)

        print("initial plastic norm min/max:", initial_plastic_norm.min(), initial_plastic_norm.max())
        print("optimized plastic norm min/max:", optimized_plastic_norm.min(), optimized_plastic_norm.max())
        print("plastic delta norm min/max:", plastic_delta_norm.min(), plastic_delta_norm.max())
        print("optimized plastic sample rows:")
        print(optimized_plastic[: min(8, optimized_plastic.shape[0])])

        weight_path = OUTPUT_DIR / "plastic_shape_match_weights.npz"
        np.savez(
            weight_path,
            initial_plastic=initial_plastic,
            optimized_plastic=optimized_plastic,
            plastic_delta=plastic_delta,
            initial_plastic_norm=initial_plastic_norm,
            optimized_plastic_norm=optimized_plastic_norm,
            plastic_delta_norm=plastic_delta_norm,
            cube_vertices=cube_vertices,
            cube_elements=cube_elements,
            surface_vertex_ids=surface_vertex_ids,
            target_vertices=target_vertices,
            optimized_vertices=optimized_vertices,
        )
        print("saved plastic weights:", weight_path)

        optimized_surface = pgo.mesh.TriMeshData(optimized_vertices, surface.elements)
        pgo.vis.plot_surface(
            [target_surface, optimized_surface],
            titles=["target", "optimized equilibrium"],
            colors=["palegreen", "salmon"],
            show_edges=True,
            window_size=(900, 420),
        )
        """
    ),
    md(
        """
        ## 5. Inspect The Learned Plastic Field
        """
    ),
    code(
        """
        print("plastic_delta_norm mean:", plastic_delta_norm.mean())
        print("plastic_delta_norm max:", plastic_delta_norm.max())

        pgo.vis.plot_volume_surface(
            [cubic, cubic, cubic],
            titles=["initial plastic magnitude", "optimized plastic magnitude", "plastic delta magnitude"],
            scalars=[initial_plastic_norm, optimized_plastic_norm, plastic_delta_norm],
            scalar_bar_titles=["||a0||", "||a*||", "||a* - a0||"],
            show_edges=True,
            window_size=(1200, 420),
        )
        """
    ),
]


def main():
    run_demo()


if __name__ == "__main__":
    main()
