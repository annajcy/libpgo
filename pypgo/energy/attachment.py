"""Soft attachment energies (vertex pin constraints)."""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo._utils import float_vector, int_vector
from pypgo.sparse import as_coo
from pypgo.energy.base import PotentialEnergy


class VertexAttachment(PotentialEnergy):
    """Soft pin constraint on selected vertices: coef * ||u_i - target_i||^2.

    The Hessian sparsity template is taken from the simulation asset.
    A fallback sparse coo input is also accepted.

    Parameters
    ----------
    asset : SimulationAsset, optional
        Provides numDofs and Koff sparsity template.
    koff : PySparseMatrix or 5-tuple, optional
        Fallback Hessian sparsity template if asset is not given.
    vertex_indices : ndarray (m,) int64
        Vertex indices to constrain.
    target_positions : ndarray (m*3,) float64
        Flat target positions (x0, y0, z0, x1, y1, z1, ...).
    coeff : float
        Penalty stiffness coefficient.
    is_displacement : bool
        True if the state is displacement (default True).
    """

    def __init__(self, *,
                 asset=None,
                 koff=None,
                 vertex_indices,
                 target_positions,
                 coeff=1e6,
                 is_displacement=True):
        vtx = int_vector("vertex_indices", vertex_indices)
        tgt = float_vector("target_positions", target_positions)
        if len(tgt) != len(vtx) * 3:
            raise ValueError(
                f"target_positions length ({len(tgt)}) must be 3 * "
                f"len(vertex_indices) ({len(vtx) * 3})"
            )

        if asset is not None:
            nDofs = asset.num_vertices * 3
            rows = nDofs
            cols = nDofs
            kri = list(range(nDofs))
            kci = list(range(nDofs))
            kvals = [1.0] * nDofs
            rest_positions = np.zeros(nDofs, dtype=np.float64)
        elif koff is not None:
            rows, cols, kri, kci, kvals = as_coo(koff)
            nDofs = rows
            rest_positions = np.zeros(nDofs, dtype=np.float64)
        else:
            raise ValueError("Either asset or koff must be provided")

        handle = _core._create_vertex_attachment(
            nDofs, rows, cols, kri, kci, list(kvals),
            rest_positions,
            vtx,
            tgt,
            float(coeff),
            bool(is_displacement),
        )
        super().__init__(handle)
        object.__setattr__(self, "_num_target_dofs", len(tgt))

    def set_targets(self, target_positions):
        """Update target positions."""
        tgt = float_vector("target_positions", target_positions)
        if len(tgt) != self._num_target_dofs:
            raise ValueError(
                f"target_positions length ({len(tgt)}) must match existing "
                f"target length ({self._num_target_dofs})"
            )
        self._handle.set_target_positions(tgt)

    def __repr__(self) -> str:
        return f"VertexAttachment({self.num_dofs} DOFs)"


class EmbeddedVertexAttachment(PotentialEnergy):
    """Soft pin on EMBEDDED vertices: coeff * ||(W u)_i||^2 (held at rest).

    ``W`` maps simulation DOFs to embedded-vertex displacement DOFs (3 per
    vertex) — e.g. a formulation's surface embedding matrix. The constraint
    therefore acts on physical points rather than simulation DOFs, which makes
    it formulation-independent: for a tricubic Hermite mesh the derivative
    DOFs participate only through the interpolation, no DOF clamping involved.

    Internally this is a constant-Hessian quadratic energy
    ``E(u) = 1/2 u^T (2*coeff*Ws^T Ws) u`` where ``Ws`` are the rows of ``W``
    belonging to the selected vertices. The coeff convention matches
    :class:`VertexAttachment` (``E = coeff * ||.||^2``).

    Parameters
    ----------
    embedding : SparseMatrix or None
        Mapping from simulation DOFs to embedded displacement DOFs (3m x n).
        ``None`` means identity (the embedded vertices ARE the simulation
        vertices, 3 DOFs each) — then ``num_dofs`` is required.
    vertex_indices : ndarray (k,) int
        Indices into the embedded vertex set to pin at rest.
    coeff : float
        Penalty stiffness coefficient.
    num_dofs : int, optional
        Number of simulation DOFs; required when ``embedding`` is None,
        otherwise inferred from the embedding's column count.
    """

    def __init__(self, *, embedding=None, vertex_indices, coeff=1e5,
                 num_dofs=None):
        from pypgo.energy.algebraic import QuadraticEnergy

        idx = np.unique(int_vector("vertex_indices", vertex_indices))
        if idx.size == 0:
            raise ValueError("vertex_indices must be non-empty")
        c2 = 2.0 * float(coeff)

        if embedding is None:
            if num_dofs is None:
                raise ValueError("num_dofs is required when embedding is None")
            n = int(num_dofs)
            dof = (idx[:, None] * 3 + np.arange(3, dtype=np.int64)).ravel()
            if dof[-1] >= n:
                raise ValueError("vertex_indices out of range for num_dofs")
            ui, uj = dof, dof
            vals = np.full(dof.size, c2, dtype=np.float64)
        else:
            n = int(embedding.shape[1])
            if num_dofs is not None and int(num_dofs) != n:
                raise ValueError(
                    f"num_dofs ({num_dofs}) conflicts with embedding columns ({n})")
            if 3 * int(idx[-1]) + 2 >= embedding.shape[0]:
                raise ValueError("vertex_indices out of range for embedding rows")
            rows, cols, wvals = embedding.to_coo()
            wanted = np.isin(rows // 3, idx)
            rows_s = rows[wanted]
            cols_s = cols[wanted].astype(np.int64)
            vals_s = wvals[wanted]
            order = np.argsort(rows_s, kind="stable")
            rows_s, cols_s, vals_s = rows_s[order], cols_s[order], vals_s[order]

            out_i, out_j, out_v = [], [], []
            row_starts = np.flatnonzero(np.r_[True, rows_s[1:] != rows_s[:-1]])
            bounds = np.r_[row_starts, rows_s.size]
            for s, e in zip(bounds[:-1], bounds[1:]):
                ci, cv = cols_s[s:e], vals_s[s:e]
                gi, gj = np.meshgrid(ci, ci, indexing="ij")
                out_i.append(gi.ravel())
                out_j.append(gj.ravel())
                out_v.append((c2 * np.outer(cv, cv)).ravel())
            oi = np.concatenate(out_i)
            oj = np.concatenate(out_j)
            ov = np.concatenate(out_v)
            key = oi * np.int64(n) + oj
            uniq, inv = np.unique(key, return_inverse=True)
            merged = np.zeros(uniq.size, dtype=np.float64)
            np.add.at(merged, inv, ov)
            ui = (uniq // n).astype(np.int64)
            uj = (uniq % n).astype(np.int64)
            vals = merged

        quad = QuadraticEnergy((n, n, ui.tolist(), uj.tolist(), vals.tolist()))
        super().__init__(quad._handle)
        object.__setattr__(self, "_num_embedded", int(idx.size))

    def __repr__(self) -> str:
        return (f"EmbeddedVertexAttachment({self.num_dofs} DOFs, "
                f"{self._num_embedded} embedded vertices)")
