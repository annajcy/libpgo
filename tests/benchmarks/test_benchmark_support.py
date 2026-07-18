from __future__ import annotations

import sys
from pathlib import Path


BENCHMARKS_ROOT = Path(__file__).resolve().parents[2] / "benchmarks"
if str(BENCHMARKS_ROOT) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS_ROOT))

from benchmark_support.google_benchmark import (  # noqa: E402
    exact_filter,
    integer_counter,
    list_cases,
    run_case,
)
from benchmark_support.statistics import (  # noqa: E402
    bootstrap_median_ci,
    median_absolute_deviation,
    percentile,
)
from benchmark_support.python_worker import (  # noqa: E402
    run_json_worker,
    worker_environment,
)
from benchmark_support.process import parse_key_value_marker  # noqa: E402
from benchmark_support.validation import (  # noqa: E402
    require_nonnegative,
    require_positive,
)


def test_google_benchmark_fixture_lists_and_runs_one_case(tmp_path: Path) -> None:
    executable = tmp_path / "fake_google_benchmark.py"
    executable.write_text(
        "#!/usr/bin/env python3\n"
        "import json\n"
        "import sys\n"
        "if '--benchmark_list_tests' in sys.argv:\n"
        "    print('Fake/Case')\n"
        "    raise SystemExit(0)\n"
        "output = next(arg.split('=', 1)[1] for arg in sys.argv if arg.startswith('--benchmark_out='))\n"
        "with open(output, 'w') as stream:\n"
        "    json.dump({'benchmarks': [{'name': 'Fake/Case', 'real_time': 2500, 'time_unit': 'us'}]}, stream)\n"
    )
    executable.chmod(0o755)

    assert list_cases(executable) == ["Fake/Case"]
    assert run_case(executable, "Fake/Case", tmp_path / "result.json", "1x", 0.0) == {
        "name": "Fake/Case",
        "real_time": 2500,
        "time_unit": "us",
        "wall_seconds": 0.0025,
    }


def test_google_benchmark_counter_and_statistics_helpers() -> None:
    assert exact_filter("prefix+suffix") == r"^prefix\+suffix$"
    assert integer_counter({"work": 4.0}, "work") == 4
    assert percentile([1.0, 3.0], 0.5) == 2.0
    assert median_absolute_deviation([1.0, 2.0, 3.0]) == 1.0

    import random

    interval = bootstrap_median_ci([1.0, 2.0, 3.0], random.Random(17), 20)
    assert interval[0] <= interval[1]


def test_marker_and_isolated_python_worker_fixtures(tmp_path: Path) -> None:
    assert parse_key_value_marker(
        "noise\nRESULT name=case iterations=4 elapsed=0.25\n",
        "RESULT",
        string_fields=frozenset({"name"}),
        integer_fields=frozenset({"iterations"}),
        float_fields=frozenset({"elapsed"}),
    ) == {"name": "case", "iterations": 4, "elapsed": 0.25}

    worker = tmp_path / "worker.py"
    worker.write_text(
        "#!/usr/bin/env python3\n"
        "import json\n"
        "print('RESULT=' + json.dumps({'answer': 42}))\n"
    )
    worker.chmod(0o755)
    environment = worker_environment(tmp_path, {"BENCHMARK_FIXTURE": "enabled"})
    assert environment["BENCHMARK_FIXTURE"] == "enabled"
    assert run_json_worker(
        [worker], marker="RESULT=", cwd=tmp_path, environment=environment
    ) == {"answer": 42}


def test_integer_argument_validation() -> None:
    require_positive(1, "--value")
    require_nonnegative(0, "--value")

    import pytest

    with pytest.raises(ValueError, match="--value must be positive"):
        require_positive(0, "--value")
    with pytest.raises(ValueError, match="--value must be nonnegative"):
        require_nonnegative(-1, "--value")
