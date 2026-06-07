#!/usr/bin/env python3
"""Generate pypgo/examples/tricubic_hermite_demo.ipynb.

Run from the repository root:

    conda run -n libpgo python pypgo/examples/scripts/generate_tricubic_hermite_demo.py
"""

from __future__ import annotations

from notebook_builder import code, md, repo_root, write_notebook


CELLS = [
    md(
        """
        # Tricubic Hermite FEM — Static Deformation Demo

        This notebook demonstrates the **regular-grid tricubic Hermite** hex
        formulation (`pf.TricubicHermite()`) on a cubic box, using only the
        Python-facing `pypgo.fem` / `pypgo.energy` / `pypgo.solver` APIs.

        **What makes tricubic Hermite different from trilinear hex.**
        A trilinear hex carries one value DOF per vertex (`num_dofs = nvtx * 3`).
        A tricubic Hermite hex carries **8 Hermite modes per vertex**
        (value + the 7 first/second/third derivative modes), so
        `num_dofs = nvtx * 24`. The global DOF index is

        ```
        global = vertex * 24 + mode * 3 + coord
        mode order: [value, d/dxi, d/deta, d/dzeta,
                     d2/dxideta, d2/dxidzeta, d2/detadzeta, d3/dxidetadzeta]
        ```

        The field is **C1 within each cell**, which is the appeal of Hermite
        elements for smooth deformation and inverse design.

        **Scope of this demo (read me).**
        The Hermite element is fully wired into the deformation-energy stack:
        `value`, `gradient`, and `hessian` all work, and you can run a static
        equilibrium solve. This notebook shows exactly that — a **static
        deformation** driven by prescribed boundary displacements.

        It does **not** run a dynamic box-drop + IPC contact simulation. Mass,
        gravity, the IPC surface embedding, and the dynamic time stepper all
        currently assume a `nvtx * 3` DOF vector, while Hermite is `nvtx * 24`.
        Making the full dynamic/contact stack Hermite-native is a separate,
        larger effort (see the project's Hermite roadmap). Everything in this
        notebook stays inside the part of the stack that already supports the
        24-DOF-per-vertex layout.
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
        from pypgo.mesh import visualize as vis
        from pypgo.mesh.volume import VolumeMesh, read_veg
        """
    ),
    md(
        """
        ## 1. Load the Cubic Box

        `box.veg` is a regular 5x5x5 axis-aligned hex grid (125 vertices,
        64 cubes) — exactly the uniform-grid regime the MVP Hermite layout
        targets. `box.obj` is a display surface embedded in the volume.
        """
    ),
    code(
        """
        PACKAGE_ROOT = Path(pgo.__file__).resolve().parent
        ASSET_DIR = PACKAGE_ROOT / "examples" / "assets"
        OUTPUT_DIR = PACKAGE_ROOT / "examples" / "outputs"
        CUBIC_BOX = ASSET_DIR / "veg" / "cubic" / "box.veg"
        BOX_SURFACE = ASSET_DIR / "obj" / "box.obj"

        veg = read_veg(str(CUBIC_BOX))
        volume = VolumeMesh.from_veg_file(veg)
        cubic_data = volume.mesh_data
        rest_vertices = cubic_data.vertices
        bbox_min, bbox_max = cubic_data.bbox

        embedded_surface = pgo.mesh.read_obj(str(BOX_SURFACE))
        surface_embedding = pgo.mesh.SurfaceEmbedding(embedded_surface, volume)

        sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
        nvtx = sim_mesh.num_vertices

        print("vertices:", nvtx, "cubes:", sim_mesh.num_elements)
        print("bbox:", cubic_data.bbox)
        vis.plot_volume_surface(cubic_data, titles=["rest cubic box"], show_edges=True)
        """
    ),
    md(
        """
        ## 2. Build the Hermite (and a Trilinear) Deformation Energy

        Both formulations consume the same `SimulationMesh` and material; only
        the formulation object differs. Note the DOF counts: Hermite is
        `nvtx * 24`, trilinear is `nvtx * 3`.
        """
    ),
    code(
        """
        def make_energy(formulation):
            state = pf.deformation_model_state(
                sim_mesh,
                elastic=pf.StableNeo(),
                elastic_field=pf.ElementwiseField(),
                plastic=pf.VolumetricPlasticity(dofs=6),
                plastic_field=pf.ElementwiseField(),
            )
            return pf.deformation_energy(state, formulation=formulation)

        hermite = make_energy(pf.TricubicHermite())
        trilinear = make_energy(pf.LinearCubic())

        print("hermite   num_dofs:", hermite.num_dofs, "= nvtx * 24 =", nvtx * 24)
        print("trilinear num_dofs:", trilinear.num_dofs, "= nvtx * 3  =", nvtx * 3)
        print("hermite state_kind:", hermite.state_kind)
        """
    ),
    md(
        """
        ## 3. Patch Tests in Python

        These mirror the C++ unit tests and are the quickest way to convince
        yourself the element is correct. On a uniform axis-aligned grid the
        synthesized rest Hermite field is exactly affine, so the rest
        deformation gradient is exactly the identity.

        We need a way to map an affine deformation `x = A X + t` onto the
        Hermite DOFs: the **value** mode of each vertex maps as `A·pos + t`,
        and **every derivative** mode maps as `A·mode` (no translation).
        """
    ),
    code(
        """
        rest_flat = np.asarray(hermite.rest_position).reshape(-1)   # (nvtx*24,)
        rest_modes = rest_flat.reshape(nvtx, 8, 3)                  # per-vertex 8 mode 3-vectors

        def hermite_affine_disp(A, t):
            deformed = rest_modes @ A.T
            deformed[:, 0, :] += t                                  # translation hits value mode only
            return (deformed - rest_modes).reshape(-1)

        def trilinear_affine_disp(A, t):
            return (rest_vertices @ A.T + t - rest_vertices).reshape(-1)

        # (a) rest state -> zero energy
        u0 = np.zeros(hermite.num_dofs)
        print("rest energy:            ", hermite.value(u0))
        print("rest gradient norm:     ", float(np.linalg.norm(hermite.gradient(u0))))

        # (b) rigid translation -> zero energy
        u_tr = hermite_affine_disp(np.eye(3), np.array([0.3, -0.7, 1.1]))
        print("translation energy:     ", hermite.value(u_tr))

        # (c) rigid rotation -> zero energy (frame-invariant material)
        ax = np.array([0.3, 0.8, 0.5]); ax = ax / np.linalg.norm(ax)
        th = 0.4
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        R = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)
        u_rot = hermite_affine_disp(R, np.zeros(3))
        print("rotation energy:        ", hermite.value(u_rot))

        # (d) a non-rigid affine deformation -> same total energy as trilinear (both have constant F)
        A = np.array([[1.05, 0.03, 0.0], [0.0, 0.98, 0.02], [0.01, 0.0, 1.03]])
        t = np.array([0.05, -0.02, 0.01])
        eH = hermite.value(hermite_affine_disp(A, t))
        eT = trilinear.value(trilinear_affine_disp(A, t))
        print("affine energy  hermite: ", eH)
        print("affine energy trilinear:", eT)
        print("relative difference:    ", abs(eH - eT) / max(1.0, abs(eT)))
        """
    ),
    md(
        """
        ## 4. Static Deformation Solve — Clamp + Shear

        A static equilibrium solve minimizes the internal Hermite energy
        subject to prescribed boundary displacements. We:

        1. **Clamp the bottom face** (`y = y_min`): fix its vertex *value* DOFs
           to zero.
        2. **Shear the top face** (`y = y_max`): fix its vertex *value* DOFs to
           a horizontal `+x` displacement.

        Only the value (position) modes are constrained; the derivative modes
        are free and settle to minimize energy. Fixed DOFs are handled by
        equality bounds on the `OptimizationProblem`, not by an extra energy.
        """
    ),
    code(
        """
        y = rest_vertices[:, 1]
        bottom = np.flatnonzero(np.isclose(y, bbox_min[1]))
        top = np.flatnonzero(np.isclose(y, bbox_max[1]))
        shear = 0.20 * float(bbox_max[0] - bbox_min[0])

        def value_dofs(vertices, stride):
            # value-mode DOF indices (x,y,z) for the given vertices under a `stride`-DOF layout.
            return (stride * vertices[:, None] + np.arange(3)).ravel()

        def solve_static(energy, stride, label):
            fixed = np.concatenate([value_dofs(bottom, stride), value_dofs(top, stride)])
            x0 = np.zeros(energy.num_dofs)
            fixed_values = x0[fixed].copy()
            # top face: prescribe +x shear on its value DOFs (every 3rd entry of the top block)
            fixed_values[len(bottom) * 3::3] = shear

            problem = ps.OptimizationProblem(objective=pe.EnergySet([(energy, 1.0)]))
            problem.fix_variables(fixed.tolist(), fixed_values, num_dofs=energy.num_dofs)
            optimizer = ps.NewtonOptimizer(
                max_iterations=200, gradient_tolerance=1e-4, damping=True, line_search="backtrack")
            result = optimizer.solve(problem, x0)
            print(f"[{label}] converged={result.converged} iters={result.iterations} "
                  f"obj={result.final_objective:.6g} grad={result.final_gradient_max_norm:.3e}")
            return result

        result_h = solve_static(hermite, 24, "hermite")
        result_t = solve_static(trilinear, 3, "trilinear")
        """
    ),
    md(
        """
        ## 5. Visualize the Deformed Shapes

        The solver state is a displacement vector. For Hermite we extract the
        **value** (position) DOFs (`x[:, :3]` of the per-vertex 24-block) to get
        the physical vertex displacement; for trilinear the displacement is
        already per-vertex. The embedded `box.obj` surface is driven from those
        value DOFs through `SurfaceEmbedding`.
        """
    ),
    code(
        """
        def value_displacement(x, stride):
            return x.reshape(-1, stride)[:, :3]

        disp_h = value_displacement(result_h.x, 24)
        disp_t = value_displacement(result_t.x, 3)

        deformed_h = pgo.mesh.CubicMeshData(rest_vertices + disp_h, cubic_data.elements)
        deformed_t = pgo.mesh.CubicMeshData(rest_vertices + disp_t, cubic_data.elements)

        surf_h = surface_embedding.deform(disp_h.reshape(-1))
        surf_t = surface_embedding.deform(disp_t.reshape(-1))

        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        pgo.mesh.write_obj(str(OUTPUT_DIR / "tricubic_hermite_sheared.obj"), surf_h)

        print("rest bbox:           ", cubic_data.bbox)
        print("hermite deformed bbox:", deformed_h.bbox)
        print("max |value disp| hermite:", float(np.max(np.abs(disp_h))))
        print("max |value disp| trilinear:", float(np.max(np.abs(disp_t))))

        vis.plot_surface(
            [embedded_surface, surf_t, surf_h],
            titles=["rest", "trilinear shear", "tricubic hermite shear"],
            colors=["lightgray", "lightsteelblue", "salmon"],
            show_edges=True,
            window_size=(1200, 420),
        )
        """
    ),
    md(
        """
        ## 6. Summary & What's Next

        You built a tricubic Hermite deformation energy (`nvtx * 24` DOFs),
        verified its correctness with patch tests (rest / translation /
        rotation give zero energy; a constant deformation gradient matches the
        trilinear energy exactly), and ran a static clamp-and-shear equilibrium
        solve, comparing it to a trilinear hex on the same boundary conditions.

        **To extend this to a dynamic box-drop + IPC simulation**, the rest of
        the simulation stack has to learn the 24-DOF-per-vertex layout:

        - a Hermite **consistent-mass** matrix (24x24 per-vertex blocks),
        - **gravity / body forces** projected onto the value modes,
        - an **IPC surface embedding** that interpolates from the value DOFs and
          scatters contact gradients back to them,
        - a **dynamic time stepper** over the `nvtx * 24` state.

        Those pieces are tracked as the next phase of the Hermite roadmap.
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "pypgo" / "examples" / "tricubic_hermite_demo.ipynb", CELLS)


if __name__ == "__main__":
    main()
