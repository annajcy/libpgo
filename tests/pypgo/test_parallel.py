import pytest

import pypgo as pgo
from pypgo import implicit


def teardown_function():
    pgo.parallel.set_num_threads(None)


def test_parallel_module_controls_default_num_threads():
    assert pgo.parallel.get_num_threads() is None

    pgo.parallel.set_num_threads(4)
    assert pgo.parallel.get_num_threads() == 4

    pgo.parallel.set_num_threads(None)
    assert pgo.parallel.get_num_threads() is None


def test_thread_limit_context_restores_previous_value():
    pgo.parallel.set_num_threads(3)

    with pgo.parallel.thread_limit(1):
        assert pgo.parallel.get_num_threads() == 1

    assert pgo.parallel.get_num_threads() == 3


def test_invalid_num_threads_rejected():
    with pytest.raises(ValueError):
        pgo.parallel.set_num_threads(0)
    with pytest.raises(ValueError):
        pgo.parallel.set_num_threads(-2)
    with pytest.raises(ValueError):
        pgo.parallel.thread_limit(0)


def test_sample_to_grid_accepts_auto_num_threads():
    spec = implicit.GridSpec([-1, -1, -1], [1, 1, 1], resolution=4)
    grid = implicit.SphereField([0, 0, 0], 1.0).sample_to_grid(spec, num_threads=None)
    assert grid.values.shape == (4, 4, 4)
