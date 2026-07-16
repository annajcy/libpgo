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


def test_invalid_global_control_concurrency_is_rejected():
    assert_scenario(
        """
        from pypgo import parallel

        for value in (0, -1):
            try:
                parallel.GlobalTbbControl(value)
            except ValueError:
                pass
            else:
                raise AssertionError("non-positive concurrency must fail")
        for value in (True, 1.5, "2"):
            try:
                parallel.GlobalTbbControl(value)
            except TypeError:
                pass
            else:
                raise AssertionError("non-integer concurrency must fail")
        """
    )


def test_runtime_diagnostics_are_removed():
    assert "GlobalTbbControl" in dir(pgo.parallel)
    assert "initialize" not in dir(pgo.parallel)
    assert {"RuntimeInfo", "runtime_info", "default_concurrency"}.isdisjoint(dir(pgo.parallel))
    assert {"_parallel_runtime_info", "_parallel_default_concurrency"}.isdisjoint(dir(pgo._core))


def test_global_tbb_control_has_explicit_python_lifetime():
    assert_scenario(
        """
        from pypgo import parallel

        with parallel.GlobalTbbControl(2) as control:
            assert control is not None
        control.close()
        for value in (0, -1):
            try:
                parallel.GlobalTbbControl(value)
            except ValueError:
                pass
            else:
                raise AssertionError("non-positive concurrency must fail")
        """
    )


def test_set_threading_policy_accepts_mkl_reset_value():
    assert_scenario(
        """
        from pypgo import parallel

        parallel.set_threading_policy(mkl_local_thread_budget=0)
        parallel.set_threading_policy(
            accelerate=parallel.AccelerateThreading.SINGLE
        )
        for value in (-1, -4):
            try:
                parallel.set_threading_policy(mkl_local_thread_budget=value)
            except ValueError:
                pass
            else:
                raise AssertionError("negative MKL budget must fail")
        """
    )


def test_arena_threading_executor_runs_callable_and_propagates_errors():
    assert_scenario(
        """
        from pypgo import parallel

        executor = parallel.ArenaThreadingExecutor(2)
        assert executor.execute(lambda: 42) == 42

        marker = []
        assert executor.execute(lambda: marker.append("ran")) is None
        assert marker == ["ran"]

        try:
            executor.execute(lambda: 1 / 0)
        except ZeroDivisionError:
            pass
        else:
            raise AssertionError("Python exceptions must cross execute()")
        """
    )
