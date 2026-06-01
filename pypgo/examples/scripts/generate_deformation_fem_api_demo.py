#!/usr/bin/env python3
"""Generate pypgo/examples/deformation_fem_api_demo.ipynb.

Uses real assets from pypgo/examples/assets/veg/{tet,cubic}/ and
pypgo/examples/assets/obj/shell.obj.

Run from the repository root:

    conda run -n libpgo python pypgo/examples/scripts/generate_deformation_fem_api_demo.py
"""

from __future__ import annotations

from notebook_builder import code, md, repo_root, write_notebook

CELLS = [
    md(
        """
        # pypgo.fem & pypgo.energy — Deformation FEM API Demo

        This tutorial demonstrates how to build deformation energies for FEM
        simulations using `pypgo.fem` and evaluate them through
        `pypgo.energy`, with **real mesh assets** (tet, cubic, shell).

        **Audience:** users building FEM simulations with tet, hex, or shell
        elements in pypgo.

        **Prerequisites:** familiarity with `pypgo.energy` basics (see
        `energy_api_demo.ipynb`) and with NumPy.

        **Learning goals:**

        1. Load `.veg` / `.obj` assets and create `SimulationMesh` objects.
        2. Choose a formulation (`TetP1`, `LinearCubic`, `KoiterShell`).
        3. Choose elastic and plastic material laws.
        4. Build a `DeformationEnergy` via `pypgo.fem.deformation_energy()`.
        5. Evaluate energy, gradient, Hessian at displacement states.
        6. Compose with `EnergySet` for multi-term objective functions.
        7. Understand `state_kind` and `rest_position` conventions.
        """
    ),
    md(
        """
        ## Outline

        1. Setup — imports and asset discovery
        2. Tet P1 energy from `bunny.veg` (711 vertices, 436 elements)
        3. Cubic hex trilinear energy from `box.veg` (125 vertices, 64 elements)
        4. Shell Koiter energy from `shell.obj` (surface mesh)
        5. Evaluate energy, gradient, Hessian at zero state
        6. Compose with EnergySet + VertexAttachment
        7. Lifetime and ownership
        8. Exercise
        """
    ),
    code(
        """
        from pathlib import Path

        import numpy as np
        import pypgo as pgo
        import pypgo.fem as pf
        import pypgo.energy as pe
        from pypgo.mesh.veg import VolumeMesh, read_veg
        """
    ),
    md(
        """
        ## 1. Setup — imports and asset discovery

        Assets live under `pypgo/examples/assets/`:
        - `veg/tet/*.veg` — tetrahedral meshes with ENu material payloads
        - `veg/cubic/*.veg` — hexahedral (cubic) meshes with ENu material payloads
        - `obj/*.obj` — triangle surface meshes (for shell)

        All `.veg` files carry ENu material, so `StableNeo` / `StVK` /
        `LinearElastic` laws are all valid.
        """
    ),
    code(
        """
        def _repo_root() -> Path:
            for p in Path.cwd().resolve().parents:
                if (p / "setup.py").exists() and (p / "pypgo").exists():
                    return p
            # fallback: look for pypgo/examples/assets relative to this file
            return Path.cwd().resolve()

        ROOT = _repo_root()
        ASSETS = ROOT / "pypgo" / "examples" / "assets"
        TET_VEG = ASSETS / "veg" / "tet"
        CUBIC_VEG = ASSETS / "veg" / "cubic"
        OBJ_DIR = ASSETS / "obj"

        print("Repo root:", ROOT)
        print()
        print("Tet .veg assets:  ", sorted(f.name for f in TET_VEG.glob("*.veg")))
        print("Cubic .veg assets:", sorted(f.name for f in CUBIC_VEG.glob("*.veg")))
        print("OBJ assets:       ", sorted(f.name for f in OBJ_DIR.glob("*.obj")))
        """
    ),
    md(
        """
        ## 2. Tet P1 energy from `bunny.veg`

        We load a real tetrahedral mesh — `bunny.veg` (711 vertices, 436
        tets, ENu material) — convert it to a `VolumeMesh`, then to a
        solver-ready `SimulationMesh`, and finally build a
        `DeformationEnergy` with the `TetP1` formulation.

        **Steps:** `.veg` → `VegFile` → `VolumeMesh` → `SimulationMesh` →
        `deformation_energy()`
        """
    ),
    code(
        """
        # Load bunny.veg tet mesh
        bunny_path = str(TET_VEG / "bunny.veg")
        veg = read_veg(bunny_path)

        # Convert to VolumeMesh (extracts tet geometry + ENu material)
        bunny_volume = VolumeMesh.from_veg_file(veg)
        print(f"VolumeMesh: {bunny_volume.num_vertices} vertices, "
              f"{bunny_volume.num_elements} elements")
        print(f"mesh_type: {bunny_volume.mesh_type}")
        """
    ),
    code(
        """
        # Create solver-ready simulation mesh
        bunny_mesh = pgo.sim.SimulationMesh.create_volumetric(bunny_volume)
        print(f"mesh_type:      {bunny_mesh.mesh_type}")
        print(f"num_vertices:   {bunny_mesh.num_vertices}")
        print(f"num_elements:   {bunny_mesh.num_elements}")
        print(f"expected DOFs:  {bunny_mesh.num_vertices * 3}")
        """
    ),
    code(
        """
        # Build Tet P1 deformation energy
        energy_tet = pf.deformation_energy(
            bunny_mesh,
            formulation=pf.TetP1(),
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        print(type(energy_tet).__name__)
        print(f"num_dofs:     {energy_tet.num_dofs}")
        print(f"state_kind:   {energy_tet.state_kind}")
        print(f"num_vertices: {energy_tet.num_vertices}")
        print(repr(energy_tet))
        """
    ),
    md(
        """
        Tet meshes can omit the `formulation` argument — `TetP1()` is the
        default.  But **cubic** and **shell** meshes **must** pass it
        explicitly (shown below).
        """
    ),
    code(
        """
        # Also try with StVK and different plastic DOFs
        energy_tet_stvk = pf.deformation_energy(
            bunny_mesh,
            elastic=pf.StVK(),
            plastic=pf.VolumetricPlasticity(dofs=3),
        )
        print(f"StVK + dof3: {energy_tet_stvk.num_dofs} DOFs, "
              f"state_kind={energy_tet_stvk.state_kind}")
        """
    ),
    md(
        """
        ## 3. Cubic hex trilinear energy from `box.veg`

        Cubic (hex) meshes use 8-node trilinear elements.  The formulation
        `LinearCubic()` is **required** — the factory will raise
        `ValueError` if you omit it.

        We use `box.veg` (125 vertices, 64 hex elements) from the cubic
        asset directory.
        """
    ),
    code(
        """
        # Load box.veg cubic mesh
        box_path = str(CUBIC_VEG / "box.veg")
        box_veg = read_veg(box_path)
        box_volume = VolumeMesh.from_veg_file(box_veg)
        print(f"VolumeMesh: {box_volume.num_vertices} vertices, "
              f"{box_volume.num_elements} elements")
        print(f"mesh_type: {box_volume.mesh_type}")
        """
    ),
    code(
        """
        box_mesh = pgo.sim.SimulationMesh.create_volumetric(box_volume)
        print(f"mesh_type:    {box_mesh.mesh_type}")
        print(f"num_vertices: {box_mesh.num_vertices}")
        print(f"num_elements: {box_mesh.num_elements}")
        """
    ),
    code(
        """
        # REQUIRED: explicit LinearCubic() formulation
        energy_cubic = pf.deformation_energy(
            box_mesh,
            formulation=pf.LinearCubic(),
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        print(f"num_dofs:   {energy_cubic.num_dofs}")
        print(f"state_kind: {energy_cubic.state_kind}")
        print(f"rest_position shape: {energy_cubic.rest_position.shape}")
        """
    ),
    code(
        """
        # Omitting the formulation for a cubic mesh → ValueError
        try:
            pf.deformation_energy(
                box_mesh,
                elastic=pf.StableNeo(),
                plastic=pf.VolumetricPlasticity(dofs=6),
            )
        except ValueError as e:
            print(f"Error (expected): {e}")
        """
    ),
    md(
        """
        ## 4. Shell Koiter energy from `shell.obj`

        Shell meshes use the Koiter thin-shell formulation.  We load
        `shell.obj` (a triangulated surface), create a `TriMeshData`,
        assign a `KoiterStVKShellMaterial`, and build the energy.

        Shells require **shell-specific** elastic/plastic wrappers:
        `KoiterStVK()` and `ShellPlasticity(dofs=1)`.
        """
    ),
    code(
        """
        # Load shell.obj → TriMeshData
        shell_path = str(OBJ_DIR / "shell.obj")
        shell_surface = pgo.mesh.read_obj(shell_path)
        print(f"Surface: {shell_surface.num_vertices} vertices, "
              f"{shell_surface.num_elements} triangles")

        # Assign shell material (membrane + bending stiffness)
        shell_mat = pgo.sim.KoiterStVKShellMaterial(
            thickness=0.001,
            E_membrane=2e6,
            nu_membrane=0.35,
        )
        shell_mesh = pgo.sim.SimulationMesh.create_shell(shell_surface, shell_mat)
        print(f"mesh_type:    {shell_mesh.mesh_type}")
        print(f"num_vertices: {shell_mesh.num_vertices}")
        """
    ),
    code(
        """
        # Build shell deformation energy
        energy_shell = pf.deformation_energy(
            shell_mesh,
            formulation=pf.KoiterShell(),
            elastic=pf.KoiterStVK(),
            plastic=pf.ShellPlasticity(dofs=1),
        )
        print(f"num_dofs:     {energy_shell.num_dofs}")
        print(f"state_kind:   {energy_shell.state_kind}")
        print(repr(energy_shell))
        """
    ),
    md(
        """
        ## 5. Evaluate energy, gradient, Hessian at zero state

        All three energy types share the same evaluation protocol.
        `state_kind == "displacement"` means the state vector `u` is a
        displacement from `rest_position`.
        """
    ),
    code(
        """
        # ---- Tet (bunny) ----
        u_tet = energy_tet.zero_state()
        print("=== Tet (bunny) ===")
        print(f"zero_state:     shape={u_tet.shape}, all_zero={np.all(u_tet == 0.0)}")
        print(f"energy(u=0):    {energy_tet.value(u_tet):.6e}")
        g_tet = energy_tet.gradient(u_tet)
        print(f"gradient norm:  {np.linalg.norm(g_tet):.6e}")
        H_tet = energy_tet.hessian(u_tet)
        print(f"Hessian:        shape={H_tet.shape}, nnz={H_tet.nnz}")
        """,
    ),
    code(
        """
        # Energy increases under a perturbation (tet)
        u_pert = u_tet.copy()
        u_pert[3] = 0.01   # perturb a vertex x-displacement
        print(f"energy(u=0):              {energy_tet.value(u_tet):.6e}")
        print(f"energy(perturbed):        {energy_tet.value(u_pert):.6e}")
        print(f"increase:                 {energy_tet.value(u_pert) - energy_tet.value(u_tet):.6e}")
        """,
    ),
    code(
        """
        # ---- Cubic (box) ----
        u_cubic = energy_cubic.zero_state()
        print("=== Cubic (box) ===")
        print(f"energy(u=0):    {energy_cubic.value(u_cubic):.6e}")
        g_cubic = energy_cubic.gradient(u_cubic)
        print(f"gradient norm:  {np.linalg.norm(g_cubic):.6e}")
        H_cubic = energy_cubic.hessian(u_cubic)
        print(f"Hessian:        shape={H_cubic.shape}, nnz={H_cubic.nnz}")
        """,
    ),
    code(
        """
        # ---- Shell ----
        u_shell = energy_shell.zero_state()
        print("=== Shell ===")
        print(f"energy(u=0):    {energy_shell.value(u_shell):.6e}")
        g_shell = energy_shell.gradient(u_shell)
        print(f"gradient norm:  {np.linalg.norm(g_shell):.6e}")
        H_shell = energy_shell.hessian(u_shell)
        print(f"Hessian:        shape={H_shell.shape}, nnz={H_shell.nnz}")
        """,
    ),
    md(
        """
        ### Rest position and state convention

        Each `DeformationEnergy` exposes the undeformed vertex positions
        as a `(num_vertices, 3)` ndarray.
        """
    ),
    code(
        """
        for label, e in [("Tet (bunny)", energy_tet),
                          ("Cubic (box)", energy_cubic),
                          ("Shell",      energy_shell)]:
            rp = e.rest_position
            print(f"{label:15s} rest_pos={rp.shape}, "
                  f"vertices={e.num_vertices}, "
                  f"state_kind={e.state_kind}, "
                  f"isinstance(PotentialEnergy)={isinstance(e, pe.PotentialEnergy)}")
        """,
    ),
    md(
        """
        ## 6. Compose with EnergySet + VertexAttachment

        `DeformationEnergy` is fully compatible with `EnergySet`.  Here we
        combine the tet deformation energy with an external gravity-like
        force and a soft pin constraint on selected vertices.
        """
    ),
    code(
        """
        # ---- External force (gravity-like, z-direction) ----
        n = energy_tet.num_dofs
        gravity = np.zeros(n, dtype=np.float64)
        gravity[2::3] = -9.81
        force = pe.LinearEnergy(gravity)

        # ---- Pin a few vertices (simulate fixed boundary) ----
        # Pin vertices 0, 1, 2 to their rest positions
        pin_vtx = np.array([0, 1, 2], dtype=np.int64)
        rp = energy_tet.rest_position
        pin_targets = rp[pin_vtx].ravel()  # flatten to (m*3,)

        pin = pe.VertexAttachment(
            sim_mesh=bunny_mesh,
            vertex_indices=pin_vtx,
            target_positions=pin_targets,
            coeff=1e6,
        )
        print(f"Pin constraint: {pin.num_dofs} DOFs, state_kind={pin.state_kind}")
        """,
    ),
    code(
        """
        # ---- Combine in EnergySet ----
        total = pe.EnergySet([
            (energy_tet,  1.0),    # deformation
            (force,      -1.0),    # external work
            (pin,         1.0),    # soft pin constraint
        ])
        print(repr(total))
        """,
    ),
    code(
        """
        # Evaluate total energy at zero displacement
        u = total.zero_state()
        print(f"total energy(u=0): {total.value(u):.6e}")

        # Perturb — energy increases due to all three terms
        u_pert = u.copy()
        u_pert[3:6] = 0.005
        print(f"total energy(perturbed): {total.value(u_pert):.6e}")

        # Adjust weights at runtime
        total.set_weight(1, 0.0)   # disable gravity
        print(f"gravity disabled:        {total.value(u_pert):.6e}")
        total.set_weight(1, -1.0)  # re-enable
        print(f"gravity re-enabled:      {total.value(u_pert):.6e}")
        """,
    ),
    md(
        """
        ## 7. Lifetime and ownership

        The C++ layer owns the mesh and energy data via `shared_ptr`.
        You can delete the Python `SimulationMesh` wrapper — the energy
        still works.
        """
    ),
    code(
        """
        import gc

        # Build energy from a temporary mesh
        tmp_veg = read_veg(str(TET_VEG / "box.veg"))
        tmp_vol = VolumeMesh.from_veg_file(tmp_veg)
        tmp_sim = pgo.sim.SimulationMesh.create_volumetric(tmp_vol)

        e = pf.deformation_energy(
            tmp_sim,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        u0 = e.zero_state()
        val_before = e.value(u0)

        # Delete Python mesh wrappers
        del tmp_sim, tmp_vol, tmp_veg
        gc.collect()

        val_after = e.value(u0)
        print(f"value before mesh deletion: {val_before:.10e}")
        print(f"value after mesh deletion:  {val_after:.10e}")
        print(f"difference:                 {abs(val_before - val_after):.2e}")
        assert abs(val_before - val_after) < 1e-14
        print("✓ energy survives mesh deletion")
        """,
    ),
    md(
        """
        ### Two independent energies from the same mesh

        The same `SimulationMesh` can be used to build multiple energies
        — it's borrowed (not consumed) by the factory.
        """
    ),
    code(
        """
        e1 = pf.deformation_energy(
            bunny_mesh,
            elastic=pf.StableNeo(),
            plastic=pf.VolumetricPlasticity(dofs=6),
        )
        e2 = pf.deformation_energy(
            bunny_mesh,
            elastic=pf.StVK(),
            plastic=pf.VolumetricPlasticity(dofs=3),
        )
        u = e1.zero_state()
        print(f"e1 (StableNeo, dof6):  {e1.value(u):.6e}")
        print(f"e2 (StVK, dof3):       {e2.value(u):.6e}")
        print(f"both use same mesh ✓")
        """,
    ),
    md(
        """
        ## 8. Exercise

        Use the tet `box.veg` asset:
        1. Build a `DeformationEnergy` with `StVK` material and `dofs=3`
           plastic.
        2. Verify the Hessian at rest is symmetric (check `|H - H^T| < 1e-8`).
        3. Add a `VertexAttachment` that pins the first 5 vertices.
        4. Compose them in an `EnergySet`.
        5. Disable the pin constraint weight → energy drops.
        """
    ),
    code(
        """
        # Your solution here
        box_tet_path = str(TET_VEG / "box.veg")
        box_tet_veg = read_veg(box_tet_path)
        box_tet_vol = VolumeMesh.from_veg_file(box_tet_veg)
        box_tet_sim = pgo.sim.SimulationMesh.create_volumetric(box_tet_vol)

        e = pf.deformation_energy(
            box_tet_sim,
            elastic=pf.StVK(),
            plastic=pf.VolumetricPlasticity(dofs=3),
        )
        u = e.zero_state()
        print(f"energy at rest: {e.value(u):.6e}")

        # Check Hessian symmetry
        H = e.hessian(u)
        rows, cols, vals = H.to_coo()
        H_dense = np.zeros(H.shape, dtype=np.float64)
        for r, c, v in zip(rows, cols, vals):
            H_dense[r, c] = v
        sym_err = np.max(np.abs(H_dense - H_dense.T))
        print(f"Hessian symmetry error: {sym_err:.2e}")
        assert sym_err < 1e-8, "Hessian must be symmetric"
        print("✓ Hessian is symmetric")

        # Pin first 5 vertices
        rp = e.rest_position
        pin_vtx = np.arange(5, dtype=np.int64)
        pin = pe.VertexAttachment(
            sim_mesh=box_tet_sim,
            vertex_indices=pin_vtx,
            target_positions=rp[pin_vtx].ravel(),
            coeff=1e6,
        )

        total = pe.EnergySet([(e, 1.0), (pin, 1.0)])
        print(f"total energy (with pin): {total.value(u):.6e}")

        total.set_weight(1, 0.0)  # disable pin
        print(f"total energy (no pin):  {total.value(u):.6e}")
        """,
    ),
    md(
        """
        ## Available formulations, materials, and plastics

        | Type | Formulation | Nodes | DOFs | Example |
        |---|---|---|---|---|
        | Tet | `pf.TetP1()` (default) | 4 | 12 | `veg/tet/*.veg` |
        | Cubic | `pf.LinearCubic()` (required) | 8 | 24 | `veg/cubic/*.veg` |
        | Shell | `pf.KoiterShell()` (required) | 6 | 18 | `obj/shell.obj` |

        | Elastic Law | Wrapper | Valid With |
        |---|---|---|
        | Stable Neo-Hookean | `pf.StableNeo()` | ENu payload |
        | StVK | `pf.StVK()` | ENu payload |
        | StVK Volume | `pf.StVKVolume()` | ENu payload |
        | Linear | `pf.LinearElastic()` | ENu payload |
        | Mooney-Rivlin | `pf.MooneyRivlin()` | MooneyRivlin payload |
        | Koiter StVK | `pf.KoiterStVK()` | Shell ENuh payload |

        | Plastic | Wrapper | Valid With |
        |---|---|---|
        | Volumetric (6 dof) | `pf.VolumetricPlasticity(dofs=6)` | Tet, Cubic |
        | Volumetric (3 dof) | `pf.VolumetricPlasticity(dofs=3)` | Tet, Cubic |
        | Volumetric (0 dof) | `pf.VolumetricPlasticity(dofs=0)` | Tet, Cubic |
        | Shell (1 dof) | `pf.ShellPlasticity(dofs=1)` | Shell |
        | Shell (0 dof) | `pf.ShellPlasticity(dofs=0)` | Shell |

        **Pitfall:** Cubic and shell meshes must pass an explicit
        formulation.  Missing formulation → `ValueError`.

        **Pitfall:** Deformation energy state is always displacement.
        Passing absolute positions will give wrong results.

        **Note:** `VolumeMesh.from_veg_file()` auto-detects the element
        type from the `.veg` file.  Make sure you load from the correct
        `veg/tet/` or `veg/cubic/` subdirectory so the mesh type matches
        your formulation.

        **Extension:** Combine `DeformationEnergy` with
        `VertexAttachment` and `EnergySet` for full static/dynamic
        IPC simulations.

        **Heavier assets:** `dragon.veg` (1247 vertices) and
        `box-with-sphere.veg` are available in both `veg/tet/` and
        `veg/cubic/` for experimentation.
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "pypgo" / "examples" / "deformation_fem_api_demo.ipynb", CELLS)


if __name__ == "__main__":
    main()
