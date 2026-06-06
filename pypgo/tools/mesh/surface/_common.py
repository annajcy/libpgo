"""Shared helpers for surface mesh CLI tools."""

from __future__ import annotations

import argparse
import json

import numpy as np

import pypgo.mesh as _mesh


def add_surface_io_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("-i", "--input-mesh", required=True, help="input OBJ surface")
    parser.add_argument("-o", "--output-mesh", required=True, help="output OBJ surface")


def read_surface(path: str) -> _mesh.TriMeshData:
    return _mesh.read_obj(path)


def write_surface(path: str, surface: _mesh.TriMeshData) -> None:
    _mesh.write_obj(path, surface)


def write_json(path: str, payload: dict) -> None:
    with open(path, "w") as f:
        f.write(json.dumps(payload, indent=2) + "\n")


def average_triangle_edge_length(surface: _mesh.TriMeshData) -> float:
    vertices = surface.vertices
    triangles = surface.elements
    if triangles.size == 0:
        raise ValueError("cannot compute average edge length for a mesh with no triangles")
    tri_vertices = vertices[triangles]
    lengths = []
    for i, j in ((0, 1), (1, 2), (2, 0)):
        lengths.append(np.linalg.norm(tri_vertices[:, i] - tri_vertices[:, j], axis=1))
    return float(np.concatenate(lengths).mean())
