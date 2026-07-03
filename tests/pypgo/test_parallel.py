import os

import pytest

import pypgo as pgo
from pypgo import implicit


def teardown_function():
    pgo.parallel.set_num_threads(None)
    pgo.parallel.set_cpu_affinity_limit(None)


def test_parallel_module_controls_thread_limit():
    assert pgo.parallel.get_num_threads() is None

    pgo.parallel.set_num_threads(4)
    assert pgo.parallel.get_num_threads() == 4

    pgo.parallel.set_num_threads(None)
    assert pgo.parallel.get_num_threads() is None


def test_runtime_info_reports_active_native_limits():
    pgo.parallel.set_num_threads(4)
    info = pgo.parallel.runtime_info()

    assert info["thread_limit"] == 4
    assert info["tbb_max_allowed_parallelism"] in (None, 4)
    assert info["openmp_max_threads"] in (None, 4)
    assert info["mkl_max_threads"] is None or info["mkl_max_threads"] >= 1


def test_thread_limit_context_restores_previous_value():
    pgo.parallel.set_num_threads(3)

    with pgo.parallel.thread_limit(1):
        assert pgo.parallel.get_num_threads() == 1

    assert pgo.parallel.get_num_threads() == 3


def test_nested_thread_limits_restore_each_process_wide_value():
    pgo.parallel.set_num_threads(4)

    with pgo.parallel.thread_limit(2):
        assert pgo.parallel.get_num_threads() == 2
        with pgo.parallel.thread_limit(1):
            assert pgo.parallel.get_num_threads() == 1
        assert pgo.parallel.get_num_threads() == 2

    assert pgo.parallel.get_num_threads() == 4


def test_invalid_num_threads_rejected():
    with pytest.raises(ValueError):
        pgo.parallel.set_num_threads(0)
    with pytest.raises(ValueError):
        pgo.parallel.set_num_threads(-2)
    with pytest.raises(ValueError):
        pgo.parallel.thread_limit(0)


def test_cpu_affinity_limit_context_restores_previous_value():
    if not pgo.parallel.supports_cpu_affinity_limit():
        pytest.skip("CPU affinity limit is not supported on this platform")

    before = os.sched_getaffinity(0)
    with pgo.parallel.cpu_affinity_limit(1):
        assert pgo.parallel.get_cpu_affinity_limit() == 1
        assert len(os.sched_getaffinity(0)) == 1

    assert pgo.parallel.get_cpu_affinity_limit() is None
    assert os.sched_getaffinity(0) == before


def test_sample_to_grid_respects_process_wide_limit():
    spec = implicit.GridSpec([-1, -1, -1], [1, 1, 1], resolution=4)
    grid = implicit.SphereField([0, 0, 0], 1.0).sample_to_grid(spec)
    assert grid.values.shape == (4, 4, 4)
