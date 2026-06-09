#!/usr/bin/env python3
"""Generate examples/tricubic_hermite_demo.ipynb.

Run from the repository root:

    conda run -n libpgo python examples/scripts/generate_tricubic_hermite_demo.py
"""

from __future__ import annotations

from examples.scripts.notebook_builder import code, md, repo_root, write_notebook


CELLS = [
    md(
        """
        # Tricubic Hermite FEM — Static Deformation Demo

        This notebook demonstrates the **regular-grid tricubic Hermite** hex
        formulation (`pf.CubicTricubicHermite()`) on a cubic box, using only the
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
        `value`, `gradient`, `hessian`, mass matrices, body forces, and surface
        embedding all work through the formulation-driven DOF layout.  This
        notebook shows a **static deformation** solve driven by prescribed
        boundary displacements, plus patch tests that verify correctness.
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
        PACKAGE_ROOT = Path(pgo.__file__).resolve().parent.parent
        ASSET_DIR = PACKAGE_ROOT / "examples" / "assets"
        OUTPUT_DIR = PACKAGE_ROOT / "examples" / "outputs" / "tricubic_hermite"
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        CUBIC_BOX = ASSET_DIR / "veg" / "cubic" / "box.veg"
        BOX_SURFACE = ASSET_DIR / "obj" / "box.obj"

        veg = read_veg(str(CUBIC_BOX))
        volume = VolumeMesh.from_veg_file(veg)
        cubic_data = volume.mesh_data
        rest_vertices = cubic_data.vertices
        bbox_min, bbox_max = cubic_data.bbox

        embedded_surface = pgo.mesh.read_obj(str(BOX_SURFACE))

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
            return pf.deformation_energy(
                sim_mesh,
                elastic=pf.StableNeo(),
                elastic_field=pf.ElementwiseField(),
                plastic=pf.VolumetricPlasticity(dofs=6),
                plastic_field=pf.ElementwiseField(),
                formulation=formulation,
            )

        hermite = make_energy(pf.CubicTricubicHermite())
        trilinear = make_energy(pf.CubicLinear())

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
                max_iterations=200, gradient_tolerance=1e-4, damping=True, line_search=ps.Backtrack())
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

        The solver state is a displacement vector. The hex-vertex positions are
        the **value** DOFs (`x[:, :3]` of the per-vertex 24-block for Hermite,
        per-vertex for trilinear), which we use for the volume bbox.

        For the embedded `box.obj` surface we drive each vertex through its
        formulation's **real basis** via `surface_embedding_matrix` — Hermite
        consumes the full `nvtx*24` DOF vector (so the surface inherits the C1
        Hermite field), trilinear the `nvtx*3` vector. This is the faithful
        surface, unlike the generic linear barycentric `SurfaceEmbedding`.
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

        # Drive the embedded surface through each formulation's real basis.
        surf_verts = embedded_surface.vertices
        W_surf_h = pf.CubicTricubicHermite().surface_embedding_matrix(volume, surf_verts)
        W_surf_t = pf.CubicLinear().surface_embedding_matrix(volume, surf_verts)
        surf_h = pgo.mesh.TriMeshData(
            surf_verts + (W_surf_h @ result_h.x).reshape(-1, 3), embedded_surface.elements)
        surf_t = pgo.mesh.TriMeshData(
            surf_verts + (W_surf_t @ result_t.x).reshape(-1, 3), embedded_surface.elements)

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
        ## 5b. C1 Continuity — reconstruct the gradient field across faces

        C1 continuity means the **deformation gradient** (first derivative of
        the displacement field) is continuous across element faces.  Trilinear
        hex is only C0 — the displacement is continuous but its gradient jumps
        at element boundaries; tricubic Hermite is C1 — the gradient is
        continuous too.

        **How to actually demonstrate it.**  It is *not* enough to read the
        derivative DOF at a shared vertex: that DOF is shared by construction,
        so it is trivially single-valued and proves nothing.  The real test is
        to **reconstruct the field through each element's own basis** and ask
        whether the gradient agrees across the shared face — including at points
        *off* the vertices, where C1 is genuinely non-trivial.

        We do that with `formulation.surface_embedding_matrix(volume, points)`,
        which builds the interpolation operator `W` from simulation DOFs to the
        displacement at arbitrary interior points, **using the formulation's
        real shape functions** (Hermite consumes all `nvtx*24` DOFs, trilinear
        `nvtx*3`).  We sample a dense line that crosses several element faces at
        an *off-corner* `(y, z)`, evaluate `u(x) = W @ dofs`, differentiate to
        get `du/dx`, and look for jumps at the faces.
        """
    ),
    code(
        """
        # ── A dense probe line crossing element faces, placed off the vertex
        #    grid so we test continuity on the face interior, not just at
        #    shared vertices. ───────────────────────────────────────────────
        x_vals = rest_vertices[:, 0]
        x_min, x_max = x_vals.min(), x_vals.max()
        faces = np.unique(np.round(x_vals, 6))          # x-positions of element faces
        h = float(np.diff(faces)[0])                    # element size
        mid_y = float(rest_vertices[:, 1].mean())
        mid_z = float(rest_vertices[:, 2].mean())

        n = 600
        xs = np.linspace(x_min + 1e-4, x_max - 1e-4, n)
        y_probe = mid_y + 0.37 * h                       # deliberately off-corner
        z_probe = mid_z + 0.31 * h
        probes = np.column_stack([xs, np.full(n, y_probe), np.full(n, z_probe)])

        # ── Reconstruct the displacement field through each REAL basis. ─────
        W_h = pf.CubicTricubicHermite().surface_embedding_matrix(volume, probes)  # (3n, nvtx*24)
        W_t = pf.CubicLinear().surface_embedding_matrix(volume, probes)      # (3n, nvtx*3)
        u_h = (W_h @ result_h.x).reshape(-1, 3)
        u_t = (W_t @ result_t.x).reshape(-1, 3)

        # du/dx of the x-displacement along the probe line.
        dudx_h = np.gradient(u_h[:, 0], xs)
        dudx_t = np.gradient(u_t[:, 0], xs)

        def onesided_jump(dudx, xf):
            # du/dx is degree-2 in x within an element (cubic field), so a
            # quadratic fit on each side is exact and extrapolates cleanly to
            # the face — giving the true one-sided limit of the gradient.
            left = (xs > xf - 0.45 * h) & (xs < xf - 0.02 * h)
            right = (xs > xf + 0.02 * h) & (xs < xf + 0.45 * h)
            limL = np.polyval(np.polyfit(xs[left], dudx[left], 2), xf)
            limR = np.polyval(np.polyfit(xs[right], dudx[right], 2), xf)
            return abs(limL - limR)

        print(f"element size h = {h:.4f},  probe at y={y_probe:.3f}, z={z_probe:.3f}")
        print(f"{'x_face':>8}  {'hermite jump':>14}  {'trilinear jump':>16}")
        for xf in faces[1:-1]:
            jh = onesided_jump(dudx_h, xf)
            jt = onesided_jump(dudx_t, xf)
            print(f"{xf:+8.3f}  {jh:14.3e}  {jt:16.3e}")
        """
    ),
    code(
        """
        import matplotlib.pyplot as plt

        fig, (ax_u, ax_g) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

        # Top: the displacement itself — continuous (C0) for BOTH formulations.
        ax_u.plot(xs, u_t[:, 0], color="tab:blue", lw=1.6, label="trilinear")
        ax_u.plot(xs, u_h[:, 0], color="tab:red", lw=1.6, label="tricubic hermite")
        ax_u.set_ylabel("u_x  (displacement)")
        ax_u.set_title("Displacement is C0 for both — value continuous across faces")
        ax_u.legend(loc="best")
        ax_u.grid(True, ls=":", alpha=0.4)

        # Bottom: du/dx — the gradient. Trilinear jumps at faces (C0);
        # Hermite stays continuous (C1).
        ax_g.plot(xs, dudx_t, color="tab:blue", lw=1.6, label="trilinear (jumps → C0)")
        ax_g.plot(xs, dudx_h, color="tab:red", lw=1.6, label="hermite (smooth → C1)")
        ax_g.set_ylabel("du_x/dx  (gradient)")
        ax_g.set_xlabel("x along probe line")
        ax_g.set_title("Gradient is discontinuous for trilinear, continuous for Hermite")
        ax_g.legend(loc="best")
        ax_g.grid(True, ls=":", alpha=0.4)

        # Mark the element faces the probe line crosses.
        for xf in faces[1:-1]:
            for ax in (ax_u, ax_g):
                ax.axvline(xf, color="gray", ls="--", lw=0.8, alpha=0.6)

        fig.tight_layout()
        plt.show()
        """
    ),
    md(
        """
        **What to look for:**
        - **Top panel (`u_x`):** both curves are continuous across every face
          (gray dashed lines) — both elements are at least C0.
        - **Bottom panel (`du_x/dx`):** the trilinear gradient is piecewise and
          **jumps** at each face, while the Hermite gradient passes through
          smoothly — this is the C1 property.
        - The printed table quantifies it: the Hermite one-sided jump is at the
          numerical-zero floor (~1e-7), while the trilinear jump is orders of
          magnitude larger (~1e-2).

        This is a genuine test because the field is reconstructed through each
        element's actual basis and sampled on the face *interior*, rather than
        reading a shared nodal DOF.
        """
    ),
    md(
        """
        ## 6. Summary

        You built a tricubic Hermite deformation energy (`nvtx * 24` DOFs),
        verified its correctness with patch tests (rest / translation /
        rotation give zero energy; a constant deformation gradient matches the
        trilinear energy exactly), and ran a static clamp-and-shear equilibrium
        solve, comparing it to a trilinear hex on the same boundary conditions.

        **Next:** full dynamic box-drop + IPC contact simulation with Hermite,
        and general curvilinear meshes (Phase 3 — inverse design transform).
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "examples" / "tricubic_hermite_demo.ipynb", CELLS)


if __name__ == "__main__":
    main()
