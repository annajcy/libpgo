import pytest

from pypgo import vis


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
