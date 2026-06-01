"""PyVista-based visualization helpers for pypgo mesh types.

Optional dependency — install with: pip install -e .[examples]
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from pypgo.mesh import CubicMeshData, TetMeshData, TriMeshData

if TYPE_CHECKING:
    import pyvista as pv

_INSTALL_HINT = "Install visualization dependencies with: pip install -e .[examples]"

try:
    import pyvista as _pv
except ModuleNotFoundError:
    _pv = None


def _require_pyvista() -> bool:
    if _pv is None:
        print(f"Skipping PyVista view. {_INSTALL_HINT}")
        return False
    return True


def to_pyvista_surface(surface_data: TriMeshData) -> "pv.PolyData":
    """Convert a TriMeshData to a PyVista PolyData surface."""
    if not isinstance(surface_data, TriMeshData):
        raise TypeError(f"surface_data must be a TriMeshData, got {type(surface_data).__name__}")
    if _pv is None:
        raise RuntimeError(_INSTALL_HINT)

    faces = np.column_stack(
        [
            np.full(surface_data.num_elements, 3, dtype=np.int64),
            surface_data.elements,
        ]
    ).ravel()
    return _pv.PolyData(surface_data.vertices, faces)


def to_pyvista_volume(volume_data: TetMeshData | CubicMeshData) -> "pv.UnstructuredGrid":
    """Convert a TetMeshData or CubicMeshData to a PyVista UnstructuredGrid."""
    if _pv is None:
        raise RuntimeError(_INSTALL_HINT)
    if not isinstance(volume_data, (TetMeshData, CubicMeshData)):
        raise TypeError(
            f"volume_data must be a TetMeshData or CubicMeshData, got {type(volume_data).__name__}"
        )

    elements = volume_data.elements
    width = elements.shape[1]
    if width == 4:
        cell_type = _pv.CellType.TETRA
    elif width == 8:
        cell_type = _pv.CellType.HEXAHEDRON
    else:
        raise ValueError(f"Unsupported volume element width: {width}")

    cells = np.column_stack(
        [
            np.full(volume_data.num_elements, width, dtype=np.int64),
            elements,
        ]
    ).ravel()
    cell_types = np.full(volume_data.num_elements, cell_type, dtype=np.uint8)
    return _pv.UnstructuredGrid(cells, cell_types, volume_data.vertices)


def plot_surface(
    meshes,
    *,
    titles=None,
    show_edges: bool = True,
    colors=None,
    window_size: tuple[int, int] = (900, 360),
):
    """Render one or more TriMeshData objects side-by-side with PyVista."""
    if not _require_pyvista():
        return None

    if isinstance(meshes, TriMeshData):
        meshes = [meshes]
    meshes = list(meshes)
    titles = titles or [None] * len(meshes)
    colors = colors or ["lightgray"] * len(meshes)

    plotter = _pv.Plotter(shape=(1, len(meshes)), window_size=window_size)
    for index, mesh in enumerate(meshes):
        if len(meshes) > 1:
            plotter.subplot(0, index)
        plotter.add_mesh(
            to_pyvista_surface(mesh),
            color=colors[index % len(colors)],
            show_edges=show_edges,
            smooth_shading=False,
        )
        if titles[index]:
            plotter.add_text(titles[index], position="upper_left", font_size=10)
        plotter.view_isometric()
        plotter.camera.zoom(1.2)
    return plotter.show()


def plot_volume_surface(
    meshes,
    *,
    titles=None,
    show_edges: bool = True,
    colors=None,
    window_size: tuple[int, int] = (900, 360),
):
    """Render the surface of one or more volume meshes side-by-side with PyVista."""
    if not _require_pyvista():
        return None

    if isinstance(meshes, (TetMeshData, CubicMeshData)):
        meshes = [meshes]
    meshes = list(meshes)
    titles = titles or [None] * len(meshes)
    colors = colors or ["lightsteelblue"] * len(meshes)

    plotter = _pv.Plotter(shape=(1, len(meshes)), window_size=window_size)
    for index, mesh in enumerate(meshes):
        if len(meshes) > 1:
            plotter.subplot(0, index)
        plotter.add_mesh(
            to_pyvista_volume(mesh).extract_surface(),
            color=colors[index % len(colors)],
            show_edges=show_edges,
            smooth_shading=False,
        )
        if titles[index]:
            plotter.add_text(titles[index], position="upper_left", font_size=10)
        plotter.view_isometric()
        plotter.camera.zoom(1.2)
    return plotter.show()
