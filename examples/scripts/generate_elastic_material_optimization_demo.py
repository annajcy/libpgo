#!/usr/bin/env python3
"""Generate examples/elastic_material_optimization_demo.ipynb.

This tutorial is the elastic-parameter counterpart of the plastic shape-match
pipeline: a material field is optimized through an implicitly differentiated
static equilibrium solve.  The public PyTorch entry point is
``pypgo.fem.ElasticStaticEquilibriumLayer``.

Run from the repository root:

    conda run -n libpgo python examples/scripts/generate_elastic_material_optimization_demo.py

Then execute top-to-bottom to validate:

    conda run -n libpgo python -m jupyter nbconvert --to notebook --execute \
        examples/elastic_material_optimization_demo.ipynb \
        --output /tmp/pypgo_elastic_material_optimization_executed.ipynb \
        --ExecutePreprocessor.timeout=600
"""

from __future__ import annotations

try:
    from .notebook_builder import code, md, repo_root, write_notebook
except ImportError:
    try:
        from notebook_builder import code, md, repo_root, write_notebook
    except ImportError:
        from examples.scripts.notebook_builder import code, md, repo_root, write_notebook


def run_demo(output_dir=None):
    """Small importable entry point for the generated notebook workflow."""
    del output_dir
    root = repo_root()
    write_notebook(root / "examples" / "elastic_material_optimization_demo.ipynb", CELLS)


CELLS = [
    md(
        r"""
        # Elastic-Field Shape Matching — Differentiating Through Equilibrium
        """
    ),
    md(
        r"""
        ## Outline

        1. Problem statement — optimize elastic parameters, not displacement
        2. Adjoint path — the elastic Jacobian $\partial^2E/\partial u\,\partial b$
           and the load-gradient correction for self-weight gravity
        3. Setup — imports and output directory
        4. Build a Koiter shell grid with an elastic field
        5. Add self-weight gravity, generate a sagging target, and fix boundary DOFs
        6. One forward + one gradient sanity check
        7. The outer optimization loop (Adam)
        8. Inspect the optimized elastic field
        9. Recap and pitfalls
        """
    ),
    md(
        r"""
        ## 1. Problem statement — optimize elastic parameters, not displacement

        We clamp the top edge of a shell, apply downward gravity, and let the
        sheet relax to static equilibrium.  The design variable is the elastic
        parameter field

        $$
        \mathbf b =
        (E_m,\nu_m,E_b,\nu_b,h)_{e=1}^{n_\mathrm{tri}},
        $$

        one five-channel Koiter StVK row per triangle.  For each candidate
        $\mathbf b$, the inner problem solves

        $$
        \mathbf u^\star(\mathbf b)
        =\arg\min_{\mathbf u:\,\mathbf u_c=0}
        \left(E_\mathrm{shell}(\mathbf u,\mathbf b)-\mathbf f_g^\top\mathbf u\right),
        $$

        and the outer problem matches the gravity-sagged surface to a target while
        regularizing the elastic field toward its initial value:

        $$
        L(\mathbf b)=
        \tfrac12\lVert \mathbf x_\mathrm{surf}(\mathbf b)-\hat{\mathbf x}\rVert^2
        +\tfrac{\mu_\mathrm{reg}}2
        \left\lVert\frac{\mathbf b-\mathbf b_0}{\mathbf s}\right\rVert^2 .
        $$
        """
    ),
    md(
        r"""
        ## 2. Adjoint path — the elastic Jacobian

        At equilibrium the free displacement DOFs satisfy
        $\mathbf g_f(\mathbf u^\star,\mathbf b)=0$.  Differentiating this equation
        with respect to elastic parameters gives

        $$
        \mathbf H_{ff}\frac{d\mathbf u_f^\star}{d\mathbf b}
        +\mathbf J^b_f=0,\qquad
        \mathbf J^b=\frac{\partial^2 E}{\partial \mathbf u\,\partial\mathbf b}.
        $$

        When the external load depends on $\mathbf b$ (self-weight gravity
        with a thickness-coupled mass field), the stationarity condition picks
        up a load-force term:

        $$
        \mathbf J^b=\frac{\partial^2 E}{\partial \mathbf u\,\partial\mathbf b}
        -\frac{\partial\mathbf f_g}{\partial\mathbf b},
        $$

        where $\mathbf f_g$ is the gravity force vector assembled by
        `SelfWeightGravity`.  In code, `energy.elastic_jacobian(u)` returns only
        the second-mixed Hessian $\partial^2E/\partial u\,\partial b$; the
        layer's backward pass adds the
        `-external_load.force_jacobian(u)`
        contribution automatically when `external_load` is provided.

        For a shape residual upstream gradient $\mathbf q=\partial L/\partial u$,
        solve one adjoint system

        $$
        \mathbf H_{ff}\lambda_f=\mathbf q_f,
        $$

        then contract

        $$
        \nabla_{\mathbf b}L_\mathrm{shape}
        =-(\mathbf J^b)^\top\lambda .
        $$

        In pypgo this is the elastic counterpart of the plastic layer:
        `ElasticStaticEquilibriumLayer.backward` calls
        `energy.elastic_jacobian(u)`, while `PlasticStaticEquilibriumLayer`
        calls `energy.plastic_jacobian(u)`.
        """
    ),
    md(
        r"""
        ## 3. Setup — imports and output directory

        The differentiable equilibrium layer currently expects CPU `float64`
        tensors.  We keep the mesh small so the notebook runs quickly.
        """
    ),
    code(
        """
        from pathlib import Path

        import matplotlib.pyplot as plt
        import numpy as np
        import pypgo as pgo
        import pypgo.energy as pe
        import pypgo.fem as pf
        import pypgo.solver as ps
        import torch

        PACKAGE_ROOT = Path(pgo.__file__).resolve().parent.parent
        OUTPUT_DIR = PACKAGE_ROOT / "examples" / "outputs" / "elastic_material_optimization"
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

        np.set_printoptions(precision=5, suppress=True)
        torch.set_default_dtype(torch.float64)
        """
    ),
    md(
        r"""
        ## 4. Build a Koiter shell grid with an elastic field

        The volumetric `StableNeo` / `StVK` paths in this build take elastic
        constants from the simulation mesh material and expose zero elastic
        optimization channels.  Koiter shells already expose the intended elastic
        field: five channels per triangle.  We therefore use a square shell grid
        and an `ElementwiseField`, mirroring the plastic demo's elementwise design
        space.
        """
    ),
    code(
        """
        nx = ny = 5

        def vertex_id(i, j):
            return i * (ny + 1) + j

        vertices = np.array(
            [[i / nx, j / ny, 0.0] for i in range(nx + 1) for j in range(ny + 1)],
            dtype=np.float64,
        )
        triangles = []
        for i in range(nx):
            for j in range(ny):
                triangles.append([vertex_id(i, j), vertex_id(i + 1, j), vertex_id(i + 1, j + 1)])
                triangles.append([vertex_id(i, j), vertex_id(i + 1, j + 1), vertex_id(i, j + 1)])
        triangles = np.asarray(triangles, dtype=np.int64)
        surface = pgo.mesh.TriMeshData(vertices, triangles)

        shell_mat = pf.KoiterStVKShellMaterial(
            thickness=1.0e-3,
            E_membrane=2.0e4,
            nu_membrane=0.35,
        )
        sim = pf.SimulationMesh.create_shell(surface, shell_mat)

        base_elastic_row = np.array([2.0e4, 0.35, 1.0e4, 0.25, 1.0e-3], dtype=np.float64)
        initial_elastic = np.tile(base_elastic_row, (triangles.shape[0], 1))

        centers = vertices[triangles].mean(axis=1)
        plastic_values = np.ones((triangles.shape[0], 1), dtype=np.float64)

        energy = pf.deformation_energy(
            sim,
            elastic=pf.KoiterStVK(),
            elastic_field=pf.ElementwiseField(values=initial_elastic),
            plastic=pf.ShellPlasticity(dofs=1),
            plastic_field=pf.ElementwiseField(values=plastic_values),
            formulation=pf.KoiterShell(),
            options=pf.DeformationOptions(enforce_spd=False, enable_material_max_step=False),
        )

        print("shell grid:", nx, "x", ny, "->", surface.num_vertices, "vertices,", surface.num_elements, "triangles")
        print("elastic field shape:", energy.elastic_field.values.shape,
              "| elastic dofs:", energy.num_elastic_dofs)
        print("plastic field is rest-state and fixed, shape:", energy.plastic_field.values.shape)
        pgo.mesh.plot_surface(surface, titles=["rest shell grid"], colors=["lightgray"], show_edges=True)
        """
    ),
    md(
        r"""
        ## 5. Add gravity, generate a sagging target, and fix boundary DOFs

        We clamp the top edge ($y=1$), apply downward gravity, and build a
        synthetic target by solving the same gravity-loaded system with a hidden
        softer/thinner elastic field.  In your own fitting run, replace only
        `target_vertices`; the rest of the pipeline is the same.

        A `ShellDensityElasticThickness` mass field reads the live thickness from
        elastic channel 4, and `SelfWeightGravity` converts it to nodal forces
        $\mathbf f_g = \rho h A g$.  The load updates every time the elastic
        parameters change, so the adjoint gains an additional
        $-(\partial\mathbf f_g/\partial\mathbf b)^\top\bm\lambda$ term.
        """
    ),
    code(
        """
        shear_strength = 0.08
        sag_strength = 20.0
        gravity_accel = np.array([0.0, 0.0, -sag_strength], dtype=np.float64)

        # Physically correct self-weight: f_g = rho * h * area/3 per corner
        # vertex, with h read live from elastic channel 4.  rho chosen so
        # rho*h0 = 1 kg/m^2 at the initial thickness.
        mass_field = pf.ShellDensityElasticThickness(
            density=1000.0, parameter_field=energy.elastic_field, channel=4)
        external_load = pf.SelfWeightGravity(
            formulation=pf.KoiterShell(), sim_mesh=sim,
            mass_field=mass_field, acceleration=gravity_accel)
        gravity_force = external_load.force()
        objective = energy  # the layer / target solve add the load dynamically

        def _objective_at_current_b():
            return pe.EnergySet([(energy, 1.0), (pe.LinearEnergy(-external_load.force()), 1.0)])

        fixed_vertices = np.flatnonzero(np.isclose(vertices[:, 1], 1.0)).astype(np.int64)
        fixed_dofs = (3 * fixed_vertices[:, None] + np.arange(3, dtype=np.int64)).ravel()
        fixed_values = np.zeros(fixed_dofs.size, dtype=np.float64)
        free_dofs = np.setdiff1d(np.arange(energy.num_dofs, dtype=np.int64), fixed_dofs)

        def solve_surface_for_elastic(elastic_values):
            energy.set_elastic_values(elastic_values)
            problem = ps.OptimizationProblem(objective=_objective_at_current_b())
            problem.fix_variables(fixed_dofs.tolist(), fixed_values, num_dofs=energy.num_dofs)
            result = ps.NewtonOptimizer(
                max_iterations=80,
                gradient_tolerance=1e-9,
                damping=True,
                line_search=ps.Backtrack(),
            ).solve(problem, energy.zero_state())
            if not result.converged:
                print("target solve warning:", result.status.name, "grad:", result.final_gradient_max_norm)
            return vertices + result.x.reshape((-1, 3))

        # Hidden target material: softer (lower E_membrane) near the free
        # lower middle.  Only E_membrane is varied; all other channels
        # (nu_m, E_bending, nu_b, thickness) stay at their initial values.
        # For a thin shell (h ≪ L) under self-weight, membrane stiffness
        # is the only elastic parameter independently identifiable from shape.
        centers = vertices[triangles].mean(axis=1)
        distance_from_clamp = 1.0 - centers[:, 1]
        center_band = np.exp(-((centers[:, 0] - 0.5) / 0.75) ** 2)
        softness = (distance_from_clamp ** 0.8) * center_band
        target_elastic = initial_elastic.copy()
        target_elastic[:, 0] *= 1.0 - 0.98 * softness
        target_vertices = solve_surface_for_elastic(target_elastic)
        target_vertices[:, 2] += shear_strength * (1.0 - vertices[:, 1]) * np.sin(2 * np.pi * vertices[:, 0])
        target_surface = pgo.mesh.TriMeshData(target_vertices, triangles)
        energy.set_elastic_values(initial_elastic)

        equilibrium_layer = pgo.fem.ElasticStaticEquilibriumLayer(
            energy=energy,
            objective_energy=_objective_at_current_b(),
            external_load=external_load,
            fixed_dofs=fixed_dofs,
            fixed_values=fixed_values,
            surface_vertices=vertices,
            surface_vertex_ids=np.arange(vertices.shape[0], dtype=np.int64),
            inner_optimizer=ps.NewtonOptimizer(
                max_iterations=80,
                gradient_tolerance=1e-9,
                damping=True,
                line_search=ps.Backtrack(),
            ),
        )

        b0 = energy.elastic_field.values.ravel().copy()
        b0_torch = torch.as_tensor(b0, dtype=torch.float64)
        target_vertices_torch = torch.as_tensor(target_vertices, dtype=torch.float64)

        print("gravity force norm:", float(np.linalg.norm(gravity_force)))
        print("fixed top-edge vertices:", fixed_vertices.size,
              "| free dofs:", free_dofs.size, "of", energy.num_dofs)
        pgo.mesh.plot_surface(
            [surface, target_surface],
            titles=["rest shell", "target gravity sag"],
            colors=["lightgray", "palegreen"],
            show_edges=True,
            window_size=(900, 420),
        )
        """
    ),
    md(
        r"""
        ## 6. One forward + one gradient sanity check

        This cell exercises the full PyTorch operator once: forward equilibrium,
        shape loss, explicit elastic regularizer, and adjoint backward through
        `energy.elastic_jacobian`.
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
                f"{label}: global={stats['global_error']:.6f}  "
                f"per-vertex RMS={stats['per_vertex_rms']:.6f}  "
                f"mean={stats['mean_vertex_error']:.6f}  "
                f"max={stats['max_vertex_error']:.6f}"
            )

        l2_weight = 2.0e-4
        """
    ),
    md(
        r"""
        ## 7. The outer optimization loop (NN-parameterized E_membrane)

        Instead of independent per-element parameters, a small neural network
        maps element centres $(x,y) \to E_\mathrm{membrane}$.  The network has
        only 30–40 weights, which (a) makes the inverse problem well-determined
        and (b) implicitly produces a smooth spatial stiffness field without
        hand-tuned regularisation.

        The loop is:

        1. `em_net(xy)` predicts per-element $E_m$.
        2. `equilibrium_layer(elastic_param)` solves self-weight equilibrium.
        3. Shape loss $+$ small weight decay.
        4. `loss.backward()` → adjoint gradient.
        5. Adam updates the network weights.
        """
    ),
    code(
        """
        xy = torch.as_tensor(centers[:, :2], dtype=torch.float64)
        b0_torch = torch.as_tensor(b0.ravel(), dtype=torch.float64)
        target_torch = torch.as_tensor(target_vertices, dtype=torch.float64)

        class EmNet(torch.nn.Module):
            \"\"\"Tiny MLP: (x, y) -> E_membrane. Tanh activations produce
            a naturally smooth field without explicit regularisation.\"\"\"
            def __init__(self):
                super().__init__()
                self.net = torch.nn.Sequential(
                    torch.nn.Linear(2, 8), torch.nn.Tanh(),
                    torch.nn.Linear(8, 8), torch.nn.Tanh(),
                    torch.nn.Linear(8, 1),
                )
                with torch.no_grad():
                    self.net[-1].bias.zero_()
                    self.net[-1].weight.zero_()

            def forward(self, xy):
                return self.net(xy).squeeze(-1) * 8000.0 + 12000.0

        em_net = EmNet().to(torch.float64)
        outer_optimizer = torch.optim.Adam(em_net.parameters(), lr=0.05)
        history = []
        best_value = np.inf
        best_state = {k: v.detach().clone() for k, v in em_net.state_dict().items()}
        best_displacement = energy.zero_state()

        def build_param(Em):
            \"\"\"Expand scalar Em_50 into full (250,) parameter vector.\"\"\"
            p = b0_torch.clone()
            p[0::5] = Em
            return p

        # -- initial forward --------------------------------------------------
        Em0 = em_net(xy)
        param0 = build_param(Em0)
        initial_vertices = equilibrium_layer(param0)
        initial_residual = initial_vertices - target_torch
        initial_loss = (0.5 * torch.sum(initial_residual ** 2)
                        + l2_weight * sum((p ** 2).sum() for p in em_net.parameters()))
        # -- sanity check: forward pass at b0, no backward yet --------
        elastic_jacobian = energy.elastic_jacobian(equilibrium_layer.last_equilibrium_displacement)
        print("initial objective L(b0):", float(initial_loss.detach()))
        print_error_stats("initial shape error", vertex_error_stats(initial_residual.detach().numpy()))
        print("elastic_jacobian shape:", elastic_jacobian.shape)
        print(f"initial Em range:     [{float(Em0.min()):.0f}, {float(Em0.max()):.0f}]")

        initial_surface = pgo.mesh.TriMeshData(initial_vertices.detach().numpy(), triangles)
        print("rest z range:   ", (float(vertices[:, 2].min()), float(vertices[:, 2].max())))
        print("initial z range:", (float(initial_surface.vertices[:, 2].min()), float(initial_surface.vertices[:, 2].max())))
        print("target z range: ", (float(target_vertices[:, 2].min()), float(target_vertices[:, 2].max())))
        pgo.mesh.plot_surface(
            [surface, target_surface, initial_surface],
            titles=["rest", "target", "initial equilibrium (b = b0)"],
            colors=["lightgray", "palegreen", "lightskyblue"],
            show_edges=True,
            window_size=(1200, 420),
        )

        # -- optimization loop ------------------------------------------------
        num_outer_steps = 300
        for iteration in range(num_outer_steps):
            outer_optimizer.zero_grad()
            Em = em_net(xy)
            solved = equilibrium_layer(build_param(Em))
            residual = solved - target_torch
            shape_loss = 0.5 * torch.sum(residual ** 2)
            wd = l2_weight * sum((p ** 2).sum() for p in em_net.parameters())
            # Em0 is the initial network output, computed once before the loop and
            # therefore tied to a graph that loss.backward() frees on the first
            # iteration. The anchor baseline is a constant, so detach it — otherwise
            # the next iteration raises "backward through the graph a second time".
            mean_anchor = 0.5 * 1e-4 * (Em.mean() - Em0.detach().mean()) ** 2
            loss = shape_loss + wd + mean_anchor
            loss.backward()

            value = float(loss.detach())
            stats = vertex_error_stats(residual.detach().numpy())
            if value < best_value:
                best_value = value
                best_state = {k: v.detach().clone() for k, v in em_net.state_dict().items()}
                best_displacement = equilibrium_layer.last_equilibrium_displacement
            outer_optimizer.step()
            history.append((value, stats["global_error"], stats["mean_vertex_error"]))
            if iteration < 5 or (iteration + 1) % 40 == 0:
                with torch.no_grad():
                    print(f"  step {iteration+1:3d}  loss={value:.6f}  "
                          f"global_err={stats['global_error']:.6f}  "
                          f"Em=[{float(Em.min()):.0f}, {float(Em.max()):.0f}]")

        print(f"\\nbest objective over {num_outer_steps} steps: {best_value:.6f}")
        """
    ),
    md(
        r"""
        ### Evaluate the best design and save the result

        Reset the warm start, re-solve at the best elastic field, and save the
        optimized field plus geometry for downstream inspection.
        """
    ),
    code(
        """
        em_net.load_state_dict(best_state)
        with torch.no_grad():
            Em_best = em_net(xy)
            best_param = build_param(Em_best)
        best_elastic = best_param.detach().numpy()
        optimized_elastic_tensor = torch.as_tensor(best_elastic, dtype=torch.float64)
        equilibrium_layer.reset_warm_start(best_displacement)
        optimized_vertices = equilibrium_layer(optimized_elastic_tensor).detach().numpy().copy()
        final_residual = optimized_vertices - target_vertices
        final_shape_value = 0.5 * float(np.dot(final_residual.ravel(), final_residual.ravel()))
        final_value = final_shape_value  # weight decay already included in best_loss

        print("final objective:", final_value, "(shape loss)")
        print_error_stats("final shape error", vertex_error_stats(final_residual))

        optimized_elastic = best_elastic.reshape(energy.elastic_field.values.shape)
        elastic_delta = optimized_elastic - initial_elastic
        E_membrane_delta = elastic_delta[:, 0]

        print("optimized vs prescribed (target) E_membrane — first 12 elements:")
        target_Em = target_elastic[:, 0]
        optimized_Em = optimized_elastic[:, 0]
        print(f"{'elem':>4s}  {'target':>8s}  {'optimized':>10s}  {'delta':>10s}")
        n_show = min(12, optimized_elastic.shape[0])
        for i in range(n_show):
            print(f"{i:4d}  {target_Em[i]:8.1f}  {optimized_Em[i]:10.1f}  {optimized_Em[i] - target_Em[i]:+10.1f}")
        print()
        Em_diff = optimized_Em - target_Em
        Em_relerr = np.abs(Em_diff) / np.maximum(np.abs(target_Em), 1.0)
        corr = np.corrcoef(target_Em, optimized_Em)[0, 1]
        print("E_membrane recovery:")
        print(f"  target:               min={target_Em.min():.0f}  max={target_Em.max():.0f}  "
              f"mean={target_Em.mean():.0f}")
        print(f"  optimized:            min={optimized_Em.min():.0f}  max={optimized_Em.max():.0f}  "
              f"mean={optimized_Em.mean():.0f}")
        print(f"  correlation r:        {corr:.4f}")
        print(f"  max |delta|:          {np.abs(Em_diff).max():.0f}")
        print(f"  mean relative error:  {Em_relerr.mean():.4f}")
        print(f"  max relative error:   {Em_relerr.max():.4f}")
        print()
        print("fixed channels (unchanged from initial):  "
              f"E_b={optimized_elastic[0,2]:.0f}  "
              f"nu_m={optimized_elastic[0,1]:.2f}  "
              f"nu_b={optimized_elastic[0,3]:.2f}  "
              f"h={optimized_elastic[0,4]:.4f}")

        weight_path = OUTPUT_DIR / "elastic_shape_match_weights.npz"
        np.savez(
            weight_path,
            initial_elastic=initial_elastic,
            optimized_elastic=optimized_elastic,
            elastic_delta=elastic_delta,
            E_membrane_delta=E_membrane_delta,
            vertices=vertices,
            triangles=triangles,
            target_vertices=target_vertices,
            optimized_vertices=optimized_vertices,
            plastic_values=plastic_values,
            gravity_force=gravity_force,
            target_elastic=target_elastic,
        )
        print("saved elastic weights ->", weight_path)

        optimized_surface = pgo.mesh.TriMeshData(optimized_vertices, triangles)
        pgo.mesh.plot_surface(
            [target_surface, optimized_surface],
            titles=["target", "optimized equilibrium"],
            colors=["palegreen", "salmon"],
            show_edges=True,
            window_size=(900, 420),
        )
        """
    ),
    md(
        r"""
        ### Convergence history
        """
    ),
    code(
        """
        objective_hist = np.array([h[0] for h in history])
        global_hist = np.array([h[1] for h in history])
        steps = np.arange(1, len(history) + 1)
        best_step = int(np.argmin(objective_hist)) + 1

        fig, ax1 = plt.subplots(figsize=(8, 4.5))
        ax1.semilogy(steps, objective_hist, color="tab:blue", marker="o", ms=3, lw=1.5,
                     label="objective L(b)")
        ax1.axvline(best_step, color="gray", ls="--", lw=1.0, label=f"best @ step {best_step}")
        ax1.set_xlabel("Adam iteration")
        ax1.set_ylabel("objective  L(b)   (log scale)", color="tab:blue")
        ax1.tick_params(axis="y", labelcolor="tab:blue")
        ax1.grid(True, which="both", ls=":", alpha=0.4)

        ax2 = ax1.twinx()
        ax2.plot(steps, global_hist, color="tab:red", marker="s", ms=3, lw=1.5,
                 label="global shape error")
        ax2.set_ylabel("global shape error  ||r||", color="tab:red")
        ax2.tick_params(axis="y", labelcolor="tab:red")

        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=9)
        ax1.set_title("Elastic shape-matching convergence")
        fig.tight_layout()
        plt.show()
        """
    ),
    md(
        r"""
        ## 8. Inspect the optimized elastic field

        Only E_membrane is plotted — the remaining four channels are fixed and
        unchanged throughout the optimization.
        """
    ),
    code(
        """
        def plot_element_scalar(ax, values, title, cmap="viridis"):
            triang = ax.tripcolor(
                vertices[:, 0],
                vertices[:, 1],
                triangles,
                facecolors=values,
                edgecolors="k",
                linewidth=0.25,
                cmap=cmap,
                shading="flat",
            )
            ax.set_aspect("equal")
            ax.set_title(title)
            ax.set_xlabel("x")
            ax.set_ylabel("y")
            plt.colorbar(triang, ax=ax, shrink=0.8)

        fig, ax = plt.subplots(1, 1, figsize=(5, 4))
        plot_element_scalar(ax, E_membrane_delta, "delta E_membrane", cmap="coolwarm")
        fig.tight_layout()
        plt.show()
        """
    ),
    md(
        r"""
        ## 9. Recap and pitfalls

        - `ElasticStaticEquilibriumLayer` is the elastic analogue of
          `PlasticStaticEquilibriumLayer`.
        - The backward pass uses `energy.elastic_jacobian(u)`:
          $\partial^2E/\partial u\,\partial b$.
        - When `external_load` is provided (self-weight gravity), the layer's
          backward pass automatically adds
          $-\lambda^\top(\partial\mathbf f_g/\partial\mathbf b)$ to the
          parameter gradient.
        - **Only E_membrane is optimised here.**  For a thin Koiter shell
          ($h \ll L$), the bending stiffness scales as $h^3$ and is typically
          six orders of magnitude smaller than the membrane stiffness ($h$).
          Under self-weight gravity, the thickness channel is also
          self-cancelling: doubling $h$ makes the shell twice as stiff but also
          twice as heavy.  The remaining channels ($\nu_m, E_b, \nu_b, h$) are
          fixed at their initial values — the gradient simply has no independent
          information about them from the equilibrium shape.
        - Bounds matter: unconstrained elastic constants can become negative or
          leave the stable range of Poisson ratios.
        """
    ),
]


if __name__ == "__main__":
    run_demo()
