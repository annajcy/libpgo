from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import textwrap

import pypgo as pgo


ROOT = Path(__file__).resolve().parents[2]


def run_parallel_scenario(source: str) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(
        [str(ROOT), env.get("PYTHONPATH", "")]
    ).rstrip(os.pathsep)
    return subprocess.run(
        [sys.executable, "-c", textwrap.dedent(source)],
        cwd=ROOT,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )


def assert_scenario(source: str) -> None:
    result = run_parallel_scenario(source)
    assert result.returncode == 0, result.stdout + result.stderr


def test_import_and_queries_do_not_initialize_runtime():
    assert_scenario(
        """
        from dataclasses import FrozenInstanceError
        import pypgo
        from pypgo import parallel

        info = parallel.runtime_info()
        assert not info.initialized
        assert info.max_concurrency is None
        assert info.current_total_participants == 0
        assert parallel.default_concurrency() >= 1
        assert not parallel.runtime_info().initialized
        assert info.__slots__ == tuple(info.__dataclass_fields__)
        try:
            info.initialized = True
        except FrozenInstanceError:
            pass
        else:
            raise AssertionError("RuntimeInfo must be frozen")
        """
    )


def test_explicit_initialize_is_idempotent_and_conflicts_are_runtime_errors():
    assert_scenario(
        """
        from pypgo import parallel

        first = parallel.initialize(max_concurrency=2)
        second = parallel.initialize(max_concurrency=2)
        assert first.initialized and second.initialized
        assert first.max_concurrency == second.max_concurrency == 2
        assert not first.using_default_concurrency
        try:
            parallel.initialize(max_concurrency=3)
        except RuntimeError as error:
            message = str(error)
            assert "2" in message and "3" in message
        else:
            raise AssertionError("conflicting initialization must fail")
        """
    )


def test_default_initialize_records_first_configuration_source():
    assert_scenario(
        """
        from pypgo import parallel

        candidate = parallel.default_concurrency()
        info = parallel.initialize()
        assert info.initialized
        assert info.using_default_concurrency
        assert info.max_concurrency == candidate
        assert info.effective_tbb_max_allowed_parallelism >= 1
        assert info.tbb_worker_ceiling == info.effective_tbb_max_allowed_parallelism - 1
        assert parallel.initialize(max_concurrency=candidate).using_default_concurrency
        """
    )


def test_parallel_algorithm_lazy_initializes_singleton_and_late_conflict_fails():
    assert_scenario(
        """
        from pypgo import implicit, parallel

        assert not parallel.runtime_info().initialized
        spec = implicit.GridSpec([-1, -1, -1], [1, 1, 1], resolution=4)
        grid = implicit.SphereField([0, 0, 0], 1.0).sample_to_grid(spec)
        assert grid.values.shape == (4, 4, 4)
        info = parallel.runtime_info()
        assert info.initialized
        requested = 2 if info.max_concurrency == 1 else 1
        try:
            parallel.initialize(max_concurrency=requested)
        except RuntimeError:
            pass
        else:
            raise AssertionError("late conflicting initialization must fail")
        """
    )


def test_old_python_and_native_controls_are_removed():
    old_public = {
        "set_worker_limit",
        "get_worker_limit",
        "worker_limit",
        "supports_cpu_affinity_limit",
        "set_cpu_affinity_limit",
        "get_cpu_affinity_limit",
        "cpu_affinity_limit",
    }
    assert old_public.isdisjoint(dir(pgo.parallel))

    old_native = {
        "_parallel_get_worker_limit",
        "_parallel_set_worker_limit",
        "_parallel_reset_worker_limit",
        "_parallel_supports_cpu_affinity_limit",
        "_parallel_get_cpu_affinity_limit",
        "_parallel_set_cpu_affinity_limit",
        "_parallel_reset_cpu_affinity_limit",
    }
    assert old_native.isdisjoint(dir(pgo._core))


def test_invalid_max_concurrency_rejected_without_initializing():
    assert_scenario(
        """
        from pypgo import parallel

        for value in (0, -1):
            try:
                parallel.initialize(max_concurrency=value)
            except ValueError:
                pass
            else:
                raise AssertionError("non-positive concurrency must fail")
        for value in (True, 1.5, "2"):
            try:
                parallel.initialize(max_concurrency=value)
            except TypeError:
                pass
            else:
                raise AssertionError("non-integer concurrency must fail")
        assert not parallel.runtime_info().initialized
        """
    )
