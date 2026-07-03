import numpy as np
import pytest

from pypgo.mesh.data import TetMeshData, TriMeshData
from pypgo.mesh import visualize as vis


def teardown_function():
    vis.reset_backend()


def test_vis_defaults_to_interactive_jupyter_backend():
    assert vis.get_backend() == "jupyter"


def test_vis_can_explicitly_switch_to_static_backend():
    vis.set_backend("static")
    assert vis.get_backend() == "static"

    vis.reset_backend()
    assert vis.get_backend() == "jupyter"


def test_vis_rejects_unknown_backend():
    with pytest.raises(ValueError):
        vis.set_backend("browser")


def test_explicit_plot_backend_overrides_default_without_mutating_it():
    class FakePlotter:
        def __init__(self):
            self.seen_backend = None

        def show(self, *, jupyter_backend):
            self.seen_backend = jupyter_backend
            return jupyter_backend

    vis.set_backend("static")

    plotter = FakePlotter()
    assert vis._show_plotter(plotter, backend="jupyter") == "trame"
    assert plotter.seen_backend == "trame"
    assert vis.get_backend() == "static"


def test_extract_volume_surface_pins_pyvista_algorithm_default():
    class FakeGrid:
        def __init__(self):
            self.algorithm = None

        def extract_surface(self, *, algorithm):
            self.algorithm = algorithm
            return "surface"

    grid = FakeGrid()
    assert vis._extract_volume_surface(grid) == "surface"
    assert grid.algorithm == "dataset_surface"


def test_plot_points_on_surface_mesh_adds_mesh_and_points(monkeypatch):
    calls = {}

    class FakeCamera:
        def zoom(self, value):
            calls["zoom"] = value

    class FakePlotter:
        def __init__(self, *, window_size):
            calls["window_size"] = window_size
            self.camera = FakeCamera()

        def add_mesh(self, mesh, **kwargs):
            calls["mesh"] = mesh
            calls["mesh_kwargs"] = kwargs

        def add_points(self, points, **kwargs):
            calls["points"] = points
            calls["point_kwargs"] = kwargs

        def add_text(self, text, **kwargs):
            calls["text"] = text
            calls["text_kwargs"] = kwargs

        def view_isometric(self):
            calls["view_isometric"] = True

        def show(self, *, jupyter_backend):
            calls["backend"] = jupyter_backend
            return "shown"

    surface = TriMeshData(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
        np.array([[0, 1, 2]], dtype=np.int64),
    )
    points = np.array([[0.25, 0.25, 0.0], [0.5, 0.25, 0.0]])

    monkeypatch.setattr(vis._pv, "Plotter", FakePlotter)
    monkeypatch.setattr(vis, "to_pyvista_surface", lambda mesh: "surface-polydata")

    result = vis.plot_points_on_mesh(
        surface,
        points,
        title="fixed vertices",
        mesh_opacity=0.2,
        point_color="red",
        point_size=12,
        backend="static",
    )

    assert result == "shown"
    assert calls["window_size"] == (900, 650)
    assert calls["mesh"] == "surface-polydata"
    assert calls["mesh_kwargs"]["opacity"] == 0.2
    assert calls["mesh_kwargs"]["show_edges"] is False
    assert np.allclose(calls["points"], points)
    assert calls["point_kwargs"]["color"] == "red"
    assert calls["point_kwargs"]["point_size"] == 12
    assert calls["point_kwargs"]["render_points_as_spheres"] is True
    assert calls["text"] == "fixed vertices"
    assert calls["view_isometric"] is True
    assert calls["zoom"] == 1.2
    assert calls["backend"] == "static"


def test_plot_points_on_volume_mesh_extracts_surface(monkeypatch):
    calls = {}

    class FakeCamera:
        def zoom(self, value):
            pass

    class FakePlotter:
        def __init__(self, *, window_size):
            self.camera = FakeCamera()

        def add_mesh(self, mesh, **kwargs):
            calls["mesh"] = mesh

        def add_points(self, points, **kwargs):
            calls["points"] = points

        def view_isometric(self):
            pass

        def show(self, *, jupyter_backend):
            return "shown"

    volume = TetMeshData(
        np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        ),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )

    monkeypatch.setattr(vis._pv, "Plotter", FakePlotter)
    monkeypatch.setattr(vis, "to_pyvista_volume", lambda mesh: "volume-grid")
    monkeypatch.setattr(vis, "_extract_volume_surface", lambda grid: f"surface({grid})")

    vis.plot_points_on_mesh(volume, np.array([[0.0, 0.0, 0.0]]), backend="none")

    assert calls["mesh"] == "surface(volume-grid)"


def test_plot_points_on_mesh_rejects_non_xyz_points():
    surface = TriMeshData(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
        np.array([[0, 1, 2]], dtype=np.int64),
    )

    with pytest.raises(ValueError, match="points must have shape"):
        vis.plot_points_on_mesh(surface, np.array([0.0, 1.0, 2.0]))


def test_write_points_obj_writes_vertices(tmp_path):
    path = tmp_path / "points.obj"

    vis.write_points_obj(
        path,
        np.array([[0.0, 1.0, 2.0], [3.5, 4.25, 5.125]], dtype=np.float64),
    )

    assert path.read_text() == "v 0.0 1.0 2.0\nv 3.5 4.25 5.125\n"
