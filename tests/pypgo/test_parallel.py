import pytest

import pypgo as pgo
from pypgo import implicit


def teardown_function():
    pgo.parallel.set_worker_limit(None)
    pgo.parallel.set_cpu_affinity_limit(None)


def test_parallel_module_controls_worker_limit():
    assert pgo.parallel.get_worker_limit() is None

    pgo.parallel.set_worker_limit(4)
    assert pgo.parallel.get_worker_limit() == 4

    pgo.parallel.set_worker_limit(None)
    assert pgo.parallel.get_worker_limit() is None


def test_runtime_info_reports_active_native_limits():
    pgo.parallel.set_worker_limit(4)
    info = pgo.parallel.runtime_info()

    assert info["worker_limit"] == 4
    assert info["tbb_max_allowed_parallelism"] in (None, 4)
    if pgo.parallel.supports_cpu_affinity_limit():
        assert info["cpu_affinity_limit"] is None
        assert info["current_cpu_affinity_cpus"] >= 1
    else:
        assert "cpu_affinity_limit" not in info
        assert "current_cpu_affinity_cpus" not in info
    assert "eigen_num_threads" not in info
    assert "openmp_max_threads" not in info
    assert "openblas_num_threads" not in info
    assert "mkl_max_threads" not in info
    assert "mkl_effective_thread_limit" not in info
    assert "mkl_pardiso_max_threads" not in info


def test_worker_limit_context_restores_previous_value():
    pgo.parallel.set_worker_limit(3)

    with pgo.parallel.worker_limit(1):
        assert pgo.parallel.get_worker_limit() == 1

    assert pgo.parallel.get_worker_limit() == 3


def test_nested_worker_limits_restore_each_process_wide_value():
    pgo.parallel.set_worker_limit(4)

    with pgo.parallel.worker_limit(2):
        assert pgo.parallel.get_worker_limit() == 2
        with pgo.parallel.worker_limit(1):
            assert pgo.parallel.get_worker_limit() == 1
        assert pgo.parallel.get_worker_limit() == 2

    assert pgo.parallel.get_worker_limit() == 4


def test_invalid_num_workers_rejected():
    with pytest.raises(ValueError):
        pgo.parallel.set_worker_limit(0)
    with pytest.raises(ValueError):
        pgo.parallel.set_worker_limit(-2)
    with pytest.raises(ValueError):
        pgo.parallel.worker_limit(0)


def test_cpu_affinity_limit_context_restores_previous_value():
    if not pgo.parallel.supports_cpu_affinity_limit():
        pytest.skip("CPU affinity limit is not supported on this platform")

    before = pgo.parallel.runtime_info()["current_cpu_affinity_cpus"]
    with pgo.parallel.cpu_affinity_limit(1):
        assert pgo.parallel.get_cpu_affinity_limit() == 1
        assert pgo.parallel.runtime_info()["current_cpu_affinity_cpus"] == 1

    assert pgo.parallel.get_cpu_affinity_limit() is None
    assert pgo.parallel.runtime_info()["current_cpu_affinity_cpus"] == before


def test_sample_to_grid_respects_process_wide_limit():
    spec = implicit.GridSpec([-1, -1, -1], [1, 1, 1], resolution=4)
    grid = implicit.SphereField([0, 0, 0], 1.0).sample_to_grid(spec)
    assert grid.values.shape == (4, 4, 4)
