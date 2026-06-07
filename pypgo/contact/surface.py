"""Contact surface descriptor and per-contact-vertex embedding metadata."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import pypgo._core as _core
from pypgo._utils import vertex_array
from pypgo.sparse import as_sparse_handle


@dataclass(frozen=True)
class ContactVertexEmbedding:
    """Optional per-contact-vertex embedding metadata."""

    indices: np.ndarray
    weights: np.ndarray

    def __post_init__(self):
        indices = np.asarray(self.indices, dtype=np.int64)
        weights = np.asarray(self.weights, dtype=np.float64)
        if indices.ndim != 2 or weights.ndim != 2:
            raise ValueError("indices and weights must be 2-D arrays")
        if indices.shape != weights.shape:
            raise ValueError("indices and weights must have the same shape")
        if indices.shape[1] <= 0:
            raise ValueError("embedding arity must be positive")
        if indices.size and np.min(indices) < 0:
            raise ValueError("indices must contain non-negative vertex indices")
        if not np.all(np.isfinite(weights)):
            raise ValueError("weights must be finite")
        row_sums = weights.sum(axis=1)
        if row_sums.size and not np.allclose(row_sums, 1.0):
            raise ValueError("each embedding weight row must sum to 1")
        object.__setattr__(self, "indices", np.ascontiguousarray(indices))
        object.__setattr__(self, "weights", np.ascontiguousarray(weights))

    @property
    def embedding_arity(self) -> int:
        return int(self.indices.shape[1])


@dataclass(frozen=True)
class ContactSurface:
    """Contact surface with a surface-sized identity simulation map."""

    _handle: object
    rest_vertices: np.ndarray
    vertex_embedding: ContactVertexEmbedding | None = None

    @staticmethod
    def identity(rest_vertices, *, vertex_embedding: ContactVertexEmbedding | None = None) -> "ContactSurface":
        vertices = vertex_array("rest_vertices", rest_vertices)
        if vertex_embedding is not None and not isinstance(vertex_embedding, ContactVertexEmbedding):
            raise TypeError("vertex_embedding must be a ContactVertexEmbedding")
        return ContactSurface(
            _core._create_contact_surface_identity(vertices),
            vertices.copy(),
            vertex_embedding,
        )

    @staticmethod
    def embedded(
        rest_vertices,
        surface_from_simulation,
        *,
        vertex_embedding: ContactVertexEmbedding | None = None,
    ) -> "ContactSurface":
        """Create a contact surface whose displacement is interpolated from simulation DOFs."""
        vertices = vertex_array("rest_vertices", rest_vertices)
        if vertex_embedding is not None and not isinstance(vertex_embedding, ContactVertexEmbedding):
            raise TypeError("vertex_embedding must be a ContactVertexEmbedding")
        return ContactSurface(
            _core._create_contact_surface_embedded(
                vertices,
                as_sparse_handle(surface_from_simulation, name="surface_from_simulation"),
            ),
            vertices.copy(),
            vertex_embedding,
        )

    @staticmethod
    def from_surface_embedding(
        embedding,
        *,
        vertex_embedding: ContactVertexEmbedding | None = None,
    ) -> "ContactSurface":
        """Create a contact surface from ``pypgo.mesh.SurfaceEmbedding``."""
        try:
            rest_surface = embedding.rest_surface
            interpolation_matrix = embedding.interpolation_matrix
        except AttributeError as exc:
            raise TypeError("embedding must be a pypgo.mesh.SurfaceEmbedding") from exc
        return ContactSurface.embedded(
            rest_surface.vertices,
            interpolation_matrix,
            vertex_embedding=vertex_embedding,
        )

    @property
    def num_surface_vertices(self) -> int:
        return self._handle.num_surface_vertices

    @property
    def num_surface_dofs(self) -> int:
        return self._handle.num_surface_dofs

    @property
    def num_simulation_dofs(self) -> int:
        return self._handle.num_simulation_dofs
