"""Geometry facades, embeddings, and algorithms."""

from pypgo.mesh.geo.core import (
    BarycentricEmbedding,
    CubicMeshGeo,
    SurfaceEmbedding,
    TetMeshGeo,
    TriMeshGeo,
    surface_to_volume_interpolation_matrix,
)
from pypgo.mesh.geo.algorithms import (
    connected_components_by_edge,
    connected_components_by_vertex,
    filter_small_components,
    get_outer_component,
    minimum_bounding_sphere,
    split_components,
    triangle_component_ids,
)

__all__ = [
    "BarycentricEmbedding",
    "CubicMeshGeo",
    "SurfaceEmbedding",
    "TetMeshGeo",
    "TriMeshGeo",
    "connected_components_by_edge",
    "connected_components_by_vertex",
    "filter_small_components",
    "get_outer_component",
    "minimum_bounding_sphere",
    "split_components",
    "surface_to_volume_interpolation_matrix",
    "triangle_component_ids",
]
