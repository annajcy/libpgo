#!/usr/bin/env python3
"""Generate examples/energy_api_demo.ipynb.

Run from the repository root:

    conda run -n libpgo python examples/scripts/generate_energy_api_demo.py
"""

from __future__ import annotations

from examples.scripts.notebook_builder import code, md, repo_root, write_notebook


CELLS = [
    md(
        """
        # pypgo.energy API Demo

        This tutorial demonstrates every public energy type in the `pypgo.energy`
        module: `LinearEnergy`, `QuadraticEnergy`, `VertexAttachment`, and
        `EnergySet`.  FEM deformation energies built by `pypgo.fem` are
        also `PotentialEnergy` instances and are covered in
        `deformation_fem_api_demo.ipynb`.

        **Audience:** users building energy-based models (FEM, IPC, optimization)
        with pypgo.

        **Prerequisites:** NumPy basics and familiarity with the concept of
        potential energy.

        **Learning goals:**

        1. Construct `LinearEnergy` and `QuadraticEnergy` from NumPy arrays.
        2. Pin vertices with `VertexAttachment`.
        3. Combine energies with `EnergySet` and adjust weights.
        4. Evaluate energy, gradient, and Hessian through the unified API.
        5. Understand `state_kind` and `zero_state` conventions.
        """
    ),
    md(
        """
        ## Outline

        1. Import and setup
        2. LinearEnergy — b^T x
        3. QuadraticEnergy — 1/2 x^T A x + b^T x
        4. VertexAttachment — soft pin constraints
        5. EnergySet — weighted sum of energies
        6. state_kind and zero_state conventions
        7. Composite Hessian and max_step
        8. Lifetime and ownership
        9. Exercise
        """
    ),
    code(
        """
        import numpy as np
        import pypgo.energy as pe
        """
    ),
    md(
        """
        ## 1. Import and setup

        All energy types live in `pypgo.energy`.  They share the same evaluation
        methods — `value(x)`, `gradient(x)`, `hessian(x)`, `max_step(x, dx)`,
        `zero_state()` — and the same properties `num_dofs`, `dofs`, `state_kind`.
        """
    ),
    code(
        """
        # Quick sanity: what's available?
        for name in sorted(dir(pe)):
            if not name.startswith("_"):
                print(name)
        """
    ),
    md(
        """
        ## 2. LinearEnergy — b^T x

        The simplest energy term.  The coefficient vector `b` is **copied** into
        C++ owned storage, so you can mutate or delete the Python array after
        construction.
        """
    ),
    code(
        """
        b = np.array([1.0, -2.0, 3.0, 4.0], dtype=np.float64)
        lin = pe.LinearEnergy(b)

        print("num_dofs:", lin.num_dofs)
        print("state_kind:", lin.state_kind)
        print(repr(lin))
        """
    ),
    code(
        """
        x = np.array([1.0, 0.5, -1.0, 0.0], dtype=np.float64)

        e = lin.value(x)
        g = lin.gradient(x)
        H = lin.hessian(x)

        print(f"value  = {e:.6f}  (expected b·x = 1*1 + -2*0.5 + 3*-1 + 4*0 = {1*1 + (-2)*0.5 + 3*(-1) + 4*0})")
        print(f"gradient = {g}  (always equals b)")
        print("hessian (dense):")
        print(H.to_dense())
        print(f"(nnz={H.nnz} — linear energy has no curvature)")
        """
    ),
    code(
        """
        # Ownership: delete b — energy still works
        b_original = np.array([10.0, 20.0, 30.0], dtype=np.float64)
        owned = pe.LinearEnergy(b_original)
        del b_original
        x = np.ones(3, dtype=np.float64)
        print("value after b deleted:", owned.value(x))
        """
    ),
    md(
        """
        ## 3. QuadraticEnergy — 1/2 x^T A x + b^T x

        The `A` matrix can be provided as a **5-tuple** `(rows, cols,
        row_indices, col_indices, values)` or as a `PySparseMatrix`.  `b` is
        optional.
        """
    ),
    code(
        """
        # 5-tuple COO input: A = diag(2, 3, 4)
        q = pe.QuadraticEnergy(
            (3, 3,                              # shape
             [0, 1, 2],                         # row_indices
             [0, 1, 2],                         # col_indices
             np.array([2.0, 3.0, 4.0], dtype=np.float64)),  # values
        )
        print("num_dofs:", q.num_dofs)
        print("state_kind:", q.state_kind)
        print(repr(q))
        """
    ),
    code(
        """
        x = np.ones(3, dtype=np.float64)

        # value = 1/2 * (2·1² + 3·1² + 4·1²) = 4.5
        print("value:", q.value(x))

        # gradient = A @ x = [2, 3, 4]
        print("gradient:", q.gradient(x))

        # Hessian = A = diag(2, 3, 4)
        H = q.hessian(x)
        rows, cols, vals = H.to_coo()
        print("hessian coo values:", vals)
        """
    ),
    md(
        """
        ### Non-diagonal Hessian

        A matrix with off-diagonal entries produces a non-trivial Hessian
        pattern visible in `to_dense()`.
        """
    ),
    code(
        """
        # A = [[2, 1, 0],
        #      [1, 3, 1],
        #      [0, 1, 4]]
        A_coo = (3, 3,
                 [0, 0, 1, 1, 1, 2, 2],     # row_indices
                 [0, 1, 0, 1, 2, 1, 2],     # col_indices
                 [2.0, 1.0, 1.0, 3.0, 1.0, 1.0, 4.0])
        q_nd = pe.QuadraticEnergy(A_coo)

        H = q_nd.hessian(np.ones(3))
        print("Hessian (dense):")
        print(H.to_dense())
        print(f"\\nshape={H.shape}, nnz={H.nnz}")
        """
    ),
    code(
        """
        # With linear term: 1/2 x^T A x + b^T x
        qb = pe.QuadraticEnergy(
            (2, 2,
             [0, 1],
             [0, 1],
             np.array([2.0, 0.0], dtype=np.float64)),
            b=np.array([1.0, 3.0], dtype=np.float64),
        )
        x = np.array([2.0, 0.0], dtype=np.float64)
        # value = 0.5 * 2 * 4 + 1*2 + 3*0 = 4 + 2 = 6.0
        print("value with b:", qb.value(x))
        g = qb.gradient(x)
        # gradient = A@x + b = [4, 0] + [1, 3] = [5, 3]
        print("gradient with b:", g)
        """
    ),
    md(
        """
        ### Dense ndarray input

        You can also pass a 2-D NumPy array directly — zeros are automatically
        dropped during COO conversion.
        """
    ),
    code(
        """
        # Dense ndarray input — zeros are dropped automatically
        A_dense = np.diag([2.0, 3.0, 4.0])
        q_dense = pe.QuadraticEnergy(A_dense)

        x = np.ones(3, dtype=np.float64)
        print("value from dense:", q_dense.value(x))
        print("gradient from dense:", q_dense.gradient(x))

        # COO tuple still works too
        q_coo = pe.QuadraticEnergy(
            (2, 2,
             [0, 1],             # row_indices (list of int)
             [0, 1],             # col_indices (list of int)
             [10.0, 20.0]),      # values (plain list)
        )
        print("value from coo:", q_coo.value(np.array([1.0, 0.0])))
        """
    ),
    md(
        """
        ## 4. VertexAttachment — soft pin constraints

        `VertexAttachment` pins selected vertices to target positions with a
        quadratic penalty: $\\text{coeff}\\,\\|u_i - \\text{target}_i\\|^2$.  It is a **displacement**
        energy (`state_kind == "displacement"`) — it assumes `x` is a displacement
        from the rest configuration.
        """
    ),
    code(
        """
        # Pin vertex 0 to (0,0,0) and vertex 1 to (1,0,0) in a 3-vertex system
        n_dofs = 9  # 3 vertices × 3 dof
        koff = (n_dofs, n_dofs,
                list(range(n_dofs)),
                list(range(n_dofs)),
                [1.0] * n_dofs)

        pin = pe.VertexAttachment(
            koff=koff,
            vertex_indices=np.array([0, 1], dtype=np.int64),
            target_positions=np.array(
                [0.0, 0.0, 0.0,   # target for vertex 0
                 1.0, 0.0, 0.0],  # target for vertex 1
                dtype=np.float64,
            ),
            coeff=1000.0,
        )
        print("num_dofs:", pin.num_dofs)
        print("state_kind:", pin.state_kind)
        """
    ),
    code(
        """
        # At zero displacement, vertices 0 and 1 are at rest (0,0,0) and (1,0,0)?
        # Wait — if is_displacement=True (default), then absolute position =
        # rest + displacement.  Our rest_positions are all-zero here, so the
        # targets are absolute positions in this simplified setup.
        u = np.zeros(n_dofs, dtype=np.float64)
        print("energy at zero disp:", pin.value(u))

        # Move vertex 0 away from its target
        u_pert = u.copy()
        u_pert[0] = 0.5   # displace vertex 0 x by 0.5
        print("energy after perturbation:", pin.value(u_pert))
        print("energy increase:", pin.value(u_pert) - pin.value(u))
        """
    ),
    code(
        """
        # Gradient points toward the targets
        g = pin.gradient(u)
        # For vertex 0 pinned to (0,0,0) at coeff 1000:
        # g[0:3] = coeff * (rest + u - target) = coeff * (0 + 0 - 0) = 0
        # For vertex 1 pinned to (1,0,0):
        # g[3:6] = coeff * (rest + u - target) = 1000 * (0 + 0 - 1, 0, 0) = (-1000, 0, 0)
        print("gradient at pinned vertices:\\n", g.reshape(-1, 3))
        """
    ),
    md(
        """
        ## 5. EnergySet — weighted sum of energies

        `EnergySet` composes multiple energy terms into a single energy:
        `total = Σ weight_i · energy_i`.  Construction is one-shot — no
        separate `add`/`init` step.
        """
    ),
    code(
        """
        # Create individual energies
        A_mat = (3, 3,
                 [0, 1, 2],
                 [0, 1, 2],
                 np.array([100.0, 200.0, 300.0], dtype=np.float64))
        elastic = pe.QuadraticEnergy(A_mat)

        force = pe.LinearEnergy(np.array([0.0, -9.81, 0.0], dtype=np.float64))

        # Combine with weights
        total = pe.EnergySet([
            (elastic,   1.0),   # elastic energy × 1
            (force,    -1.0),   # − b^T u  (external work)
        ])
        print(repr(total))
        """
    ),
    code(
        """
        u = np.array([0.01, -0.02, 0.0], dtype=np.float64)

        print("total value:     ", total.value(u))
        print("elastic only:    ", elastic.value(u))
        print("force only:      ", force.value(u))
        """
    ),
    code(
        """
        # Adjust weights dynamically
        total.set_weight(1, 0.0)  # disable external force
        print("after disabling force:", total.value(u))

        total.set_weight(1, -1.0)  # re-enable
        print("after re-enabling:    ", total.value(u))
        """
    ),
    md(
        """
        ## 6. state_kind and zero_state conventions

        `state_kind` is `"displacement"` for FEM/contact/attachment energies and
        `"generic"` for linear/quadratic energies.  `EnergySet` composes them:
        all-children-displacement → `"displacement"`, otherwise `"generic"`.
        """
    ),
    code(
        """
        n = 3
        koff = (n, n, list(range(n)), list(range(n)), [1.0] * n)
        pin = pe.VertexAttachment(
            koff=koff,
            vertex_indices=np.array([0], dtype=np.int64),
            target_positions=np.zeros(3, dtype=np.float64),
        )
        lin = pe.LinearEnergy(np.ones(n, dtype=np.float64))

        print("pin state_kind:  ", pin.state_kind)
        print("lin state_kind:  ", lin.state_kind)

        es_pin = pe.EnergySet([(pin, 1.0)])
        es_mix = pe.EnergySet([(pin, 1.0), (lin, 1.0)])

        print("pin-only set:    ", es_pin.state_kind)
        print("mixed set:       ", es_mix.state_kind)
        """
    ),
    code(
        """
        # zero_state() always returns a zero array of the right dtype
        for e in [pin, lin, elastic, total]:
            z = e.zero_state()
            print(f"{type(e).__name__:20s}  zero_state: shape={z.shape}, dtype={z.dtype}")
        """
    ),
    md(
        """
        ## 7. Composite Hessian and max_step

        The Hessian of an `EnergySet` is assembled automatically from its
        children.  `max_step` delegates to each child's barrier (if any) and
        returns the most restrictive limit.
        """
    ),
    code(
        """
        # Hessian of combined energy — returns a SparseMatrix
        H = total.hessian(u)
        print(f"total Hessian: {H.shape[0]}×{H.shape[1]}, nnz={H.nnz}")

        # COO export
        rows, cols, vals = H.to_coo()
        print("coo values:", vals)

        # Dense export
        H_dense = H.to_dense()
        print("\\ndense Hessian:\\n", H_dense)
        """
    ),
    code(
        """
        # max_step returns a StepConstraint with alpha + clamp info
        dx = np.ones_like(u)
        ms = total.max_step(u, dx)
        print(f"alpha:             {ms.alpha}")
        print(f"clamped:           {ms.clamped}")
        print(f"source:            {ms.source}")
        """
    ),
    md(
        """
        ## 8. Lifetime and ownership

        Every energy type **owns** its data in C++.  You can delete the Python
        wrapper of a child energy after adding it to an `EnergySet` — the set
        keeps the C++ object alive through `shared_ptr`.
        """
    ),
    code(
        """
        import gc

        q = pe.QuadraticEnergy(
            (2, 2, [0, 1], [0, 1], np.array([5.0, 5.0], dtype=np.float64)),
        )
        es = pe.EnergySet([(q, 1.0)])
        del q
        gc.collect()

        x = np.ones(2, dtype=np.float64)
        print("value after child deletion:", es.value(x))
        """
    ),
    md(
        """
        ## 9. Exercise

        Build a simple 2-DOF system:
        1. A quadratic energy with `A = diag(10, 20)`.
        2. A linear energy with `b = [1, 0]`.
        3. Combine them in an `EnergySet` with weights `(1.0, -1.0)`.
        4. Evaluate at `x = [1, 1]` and verify the gradient manually.
        5. Temporarily set the linear term weight to 0 and re-evaluate.
        """
    ),
    code(
        """
        # Your solution here
        q = pe.QuadraticEnergy(
            (2, 2, [0, 1], [0, 1], np.array([10.0, 20.0], dtype=np.float64)),
        )
        lin = pe.LinearEnergy(np.array([1.0, 0.0], dtype=np.float64))
        es = pe.EnergySet([(q, 1.0), (lin, -1.0)])

        x = np.array([1.0, 1.0], dtype=np.float64)

        # value = 0.5*(10*1 + 20*1) - 1*1 = 15 - 1 = 14
        print("value:", es.value(x))
        # gradient: A@x = [10, 20]; -b = [-1, 0]; total = [9, 20]
        print("gradient:", es.gradient(x))

        es.set_weight(1, 0.0)
        print("value without linear term:", es.value(x))
        print("gradient without linear term:", es.gradient(x))
        """
    ),
    md(
        """
        ## Pitfalls and extensions

        **Pitfall:** all energies in an `EnergySet` must have the same
        `num_dofs`.  A `ValueError` is raised if they differ.

        **Pitfall:** `VertexAttachment` `is_displacement=True` (default) means
        the state `x` is interpreted as a displacement from rest.  Set
        `is_displacement=False` if your state is absolute positions.

        **Pitfall:** `QuadraticEnergy` accepts 5-tuple COO input `(rows, cols,
        row_indices, col_indices, values)` — 4-tuples are rejected with a
        clear error.

        **Extension:** for FEM deformation energies (tet, hex, shell), see
        `deformation_fem_api_demo.ipynb`.  The `pypgo.fem` module builds
        `DeformationEnergy` objects that are full `PotentialEnergy` instances
        with `state_kind == "displacement"` and can be combined in
        `EnergySet` alongside `LinearEnergy`, `QuadraticEnergy`, and
        `VertexAttachment`.

        **Extension:** `DeformationEnergy` exposes `rest_position` — a
        `(num_vertices, 3)` ndarray of undeformed vertex coordinates.
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "examples" / "energy_api_demo.ipynb", CELLS)


if __name__ == "__main__":
    main()
