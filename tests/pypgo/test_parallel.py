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


def test_initialize_is_repeatable_and_returns_effective_concurrency():
    assert_scenario(
        """
        from pypgo import parallel

        first = parallel.initialize(max_concurrency=2)
        second = parallel.initialize(max_concurrency=4)
        restored = parallel.initialize()
        assert isinstance(first, int) and 1 <= first <= 2
        assert isinstance(second, int) and 1 <= second <= 4
        assert isinstance(restored, int) and restored >= 1
        """
    )


def test_invalid_max_concurrency_is_rejected():
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
        """
    )


def test_runtime_diagnostics_are_removed():
    assert "initialize" in dir(pgo.parallel)
    assert {"RuntimeInfo", "runtime_info", "default_concurrency"}.isdisjoint(dir(pgo.parallel))
    assert {"_parallel_runtime_info", "_parallel_default_concurrency"}.isdisjoint(dir(pgo._core))
