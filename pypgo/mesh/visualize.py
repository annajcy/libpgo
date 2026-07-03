"""PyVista-based visualization helpers for pypgo mesh types."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pyvista as _pv

from pypgo.mesh.data import CubicMeshData, TetMeshData, TriMeshData

_DEFAULT_BACKEND = "jupyter"
_VALID_BACKENDS = {"jupyter", "static", "none"}
_PYVISTA_BACKENDS = {
    "jupyter": "trame",
    "static": "static",
    "none": "none",
}
_backend = _DEFAULT_BACKEND


def _normalize_backend(backend: str) -> str:
    backend = str(backend).lower()
    if backend not in _VALID_BACKENDS:
        valid = ", ".join(sorted(_VALID_BACKENDS))
        raise ValueError(f"backend must be one of: {valid}")
    return backend


def set_backend(backend: str) -> None:
    """Set the default visualization backend used when a plot call omits one."""
    global _backend
    _backend = _normalize_backend(backend)


def reset_backend() -> None:
    """Reset pypgo visualization defaults to the interactive Jupyter backend."""
    set_backend(_DEFAULT_BACKEND)


def get_backend() -> str:
    """Return the current default pypgo visualization backend for plot calls."""
    return _backend


def _show_plotter(plotter, *, backend: str | None):
    effective_backend = _backend if backend is None else _normalize_backend(backend)
    return plotter.show(jupyter_backend=_PYVISTA_BACKENDS[effective_backend])


def _extract_volume_surface(volume_grid):
    return volume_grid.extract_surface(algorithm="dataset_surface")


def _normalize_scalar_inputs(scalars, count: int):
    if scalars is None:
        return [None] * count
    if isinstance(scalars, np.ndarray):
        if count != 1:
            raise ValueError("scalars must be a list with one array per mesh")
        return [scalars]
    if isinstance(scalars, (list, tuple)):
        if count == 1:
            return [np.asarray(scalars, dtype=np.float64)]
        if len(scalars) != count:
            raise ValueError("scalars must have one entry per mesh")
        return [None if values is None else np.asarray(values, dtype=np.float64) for values in scalars]
    if count != 1:
        raise ValueError("scalars must be a list with one array per mesh")
    return [np.asarray(scalars, dtype=np.float64)]


def _normalize_points(points) -> np.ndarray:
    point_array = np.asarray(points, dtype=np.float64)
    if point_array.ndim != 2 or point_array.shape[1] != 3:
        raise ValueError(f"points must have shape (n, 3), got {point_array.shape}")
    return np.ascontiguousarray(point_array, dtype=np.float64)


def to_pyvista_surface(surface_data: TriMeshData) -> _pv.PolyData:
    """Convert a TriMeshData to a PyVista PolyData surface."""
    if not isinstance(surface_data, TriMeshData):
        raise TypeError(f"surface_data must be a TriMeshData, got {type(surface_data).__name__}")

    faces = np.column_stack(
        [
            np.full(surface_data.num_elements, 3, dtype=np.int64),
            surface_data.elements,
        ]
    ).ravel()
    return _pv.PolyData(surface_data.vertices, faces)


def to_pyvista_volume(volume_data: TetMeshData | CubicMeshData) -> _pv.UnstructuredGrid:
    """Convert a TetMeshData or CubicMeshData to a PyVista UnstructuredGrid."""
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
    backend: str | None = None,
):
    """Render one or more TriMeshData objects side-by-side with PyVista.

    ``backend`` overrides the module default for this call only. Use
    ``"jupyter"`` for interactive notebook views or ``"static"`` for images.
    """
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
    return _show_plotter(plotter, backend=backend)


def plot_volume_surface(
    meshes,
    *,
    titles=None,
    show_edges: bool = True,
    colors=None,
    scalars=None,
    scalar_bar_titles=None,
    clims=None,
    window_size: tuple[int, int] = (900, 360),
    backend: str | None = None,
):
    """Render the surface of one or more volume meshes side-by-side with PyVista.

    ``backend`` overrides the module default for this call only. Use
    ``"jupyter"`` for interactive notebook views or ``"static"`` for images.

    ``clims`` optionally fixes the color range per subplot: pass a list with one
    ``(lo, hi)`` tuple (or ``None`` for autoscale) per mesh. Use this to make
    panels share a comparable scale, or to keep a small-magnitude field from
    being washed out next to a large one. Default ``None`` autoscales each panel.
    """
    if isinstance(meshes, (TetMeshData, CubicMeshData)):
        meshes = [meshes]
    meshes = list(meshes)
    titles = titles or [None] * len(meshes)
    colors = colors or ["lightsteelblue"] * len(meshes)
    scalar_arrays = _normalize_scalar_inputs(scalars, len(meshes))
    scalar_bar_titles = scalar_bar_titles or [None] * len(meshes)
    if clims is None:
        clims = [None] * len(meshes)
    elif len(clims) != len(meshes):
        raise ValueError("clims must have one (lo, hi) entry (or None) per mesh")

    plotter = _pv.Plotter(shape=(1, len(meshes)), window_size=window_size)
    for index, mesh in enumerate(meshes):
        if len(meshes) > 1:
            plotter.subplot(0, index)
        grid = to_pyvista_volume(mesh)
        scalar_array = scalar_arrays[index]
        if scalar_array is None:
            plotter.add_mesh(
                _extract_volume_surface(grid),
                color=colors[index % len(colors)],
                show_edges=show_edges,
                smooth_shading=False,
            )
        else:
            if scalar_array.ndim != 1 or scalar_array.size != mesh.num_elements:
                raise ValueError(
                    f"scalars[{index}] must have shape ({mesh.num_elements},), got {scalar_array.shape}"
                )
            name = f"cell_scalars_{index}"
            grid.cell_data[name] = scalar_array
            plotter.add_mesh(
                grid,
                scalars=name,
                preference="cell",
                clim=clims[index],
                show_edges=show_edges,
                smooth_shading=False,
                scalar_bar_args={"title": scalar_bar_titles[index] or ""},
            )
        if titles[index]:
            plotter.add_text(titles[index], position="upper_left", font_size=10)
        plotter.view_isometric()
        plotter.camera.zoom(1.2)
    return _show_plotter(plotter, backend=backend)


def plot_points_on_mesh(
    mesh,
    points,
    *,
    title=None,
    mesh_color="lightgray",
    mesh_opacity: float = 0.3,
    point_color="red",
    point_size: float = 10,
    render_points_as_spheres: bool = True,
    show_edges: bool = False,
    window_size: tuple[int, int] = (900, 650),
    backend: str | None = None,
):
    """Render a mesh with an overlaid point cloud.

    ``mesh`` may be a :class:`TriMeshData`, :class:`TetMeshData`, or
    :class:`CubicMeshData`. Volume meshes are displayed through their extracted
    surface. ``points`` must be an ``(n, 3)`` array in the same coordinate
    system as the mesh.
    """
    point_array = _normalize_points(points)
    if isinstance(mesh, TriMeshData):
        display_mesh = to_pyvista_surface(mesh)
    elif isinstance(mesh, (TetMeshData, CubicMeshData)):
        display_mesh = _extract_volume_surface(to_pyvista_volume(mesh))
    else:
        raise TypeError(
            "mesh must be a TriMeshData, TetMeshData, or CubicMeshData, "
            f"got {type(mesh).__name__}"
        )

    plotter = _pv.Plotter(window_size=window_size)
    plotter.add_mesh(
        display_mesh,
        color=mesh_color,
        opacity=float(mesh_opacity),
        show_edges=show_edges,
        smooth_shading=False,
    )
    plotter.add_points(
        point_array,
        color=point_color,
        point_size=float(point_size),
        render_points_as_spheres=render_points_as_spheres,
    )
    if title:
        plotter.add_text(title, position="upper_left", font_size=10)
    plotter.view_isometric()
    plotter.camera.zoom(1.2)
    return _show_plotter(plotter, backend=backend)


def write_points_obj(path, points) -> None:
    """Write an ``(n, 3)`` point cloud as OBJ vertex records."""
    point_array = _normalize_points(points)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as file:
        for point in point_array:
            file.write(f"v {point[0]} {point[1]} {point[2]}\n")
