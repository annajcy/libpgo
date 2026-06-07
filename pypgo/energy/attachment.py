"""Soft attachment energies (vertex pin constraints)."""

from __future__ import annotations

import numpy as np

import pypgo._core as _core
from pypgo._utils import float_vector, int_vector
from pypgo.sparse import as_coo
from pypgo.energy.base import PotentialEnergy


class VertexAttachment(PotentialEnergy):
    """Soft pin constraint on selected vertices: coef * ||u_i - target_i||^2.

    The Hessian sparsity template is taken from the sim mesh (Koff).
    A fallback sparse coo input is also accepted.

    Parameters
    ----------
    sim_mesh : SimulationMesh, optional
        Provides numDofs and Koff sparsity template.
    koff : PySparseMatrix or 5-tuple, optional
        Fallback Hessian sparsity template if sim_mesh is not given.
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
                 sim_mesh=None,
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

        if sim_mesh is not None:
            nDofs = sim_mesh.num_vertices * 3
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
            raise ValueError("Either sim_mesh or koff must be provided")

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
