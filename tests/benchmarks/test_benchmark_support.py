from __future__ import annotations

import sys
from pathlib import Path


BENCHMARKS_ROOT = Path(__file__).resolve().parents[2] / "benchmarks"
if str(BENCHMARKS_ROOT) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS_ROOT))

from benchmark_support.cpp_probe import (  # noqa: E402
    duration_seconds,
    integer_counter,
    run_cpp_probe,
)
from benchmark_support.artifact import JsonArtifact, runner_manifest  # noqa: E402
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
from benchmark_support.warmup import run_workload_warmup  # noqa: E402


def test_cpp_probe_fixture_runs_and_normalizes_one_case(tmp_path: Path) -> None:
    executable = tmp_path / "fake_cpp_probe.py"
    executable.write_text(
        "#!/usr/bin/env python3\n"
        "import sys\n"
        "value = lambda prefix: next(arg.split('=', 1)[1] for arg in sys.argv if arg.startswith(prefix))\n"
        "print('FAKE_RESULT label=case count=7 score=2.5 '"
        "+ 'configured_warmup_seconds=' + value('--warmup-seconds=') + ' '"
        "+ 'configured_warmup_min_operations=' + value('--warmup-min-operations=') + ' '"
        "+ 'actual_warmup_seconds=0.001 actual_warmup_operations=10 '"
        "+ 'configured_measurement_min_seconds=' + value('--measurement-min-seconds=') + ' '"
        "+ 'measurement_operations=2 measurement_wall_seconds=0.02')\n"
    )
    executable.chmod(0o755)

    assert run_cpp_probe(
        executable,
        ["--case=fixture"],
        marker="FAKE_RESULT",
        min_time="10ms",
        warmup_seconds=0.0,
        warmup_min_operations=10,
        string_fields=frozenset({"label"}),
        integer_fields=frozenset({"count"}),
        float_fields=frozenset({"score"}),
    ) == {
        "label": "case",
        "count": 7,
        "score": 2.5,
        "configured_warmup_seconds": 0.0,
        "configured_warmup_min_operations": 10,
        "actual_warmup_seconds": 0.001,
        "actual_warmup_operations": 10,
        "configured_measurement_min_seconds": 0.01,
        "measurement_operations": 2,
        "measurement_wall_seconds": 0.02,
        "wall_seconds": 0.01,
    }


def test_cpp_probe_counter_duration_and_statistics_helpers() -> None:
    assert duration_seconds("250ms") == 0.25
    assert integer_counter({"work": 4.0}, "work") == 4
    assert percentile([1.0, 3.0], 0.5) == 2.0
    assert median_absolute_deviation([1.0, 2.0, 3.0]) == 1.0

    import random

    interval = bootstrap_median_ci([1.0, 2.0, 3.0], random.Random(17), 20)
    assert interval[0] <= interval[1]


def test_workload_warmup_runs_exact_fixed_operation_count() -> None:
    completed = 0

    def operation() -> int:
        nonlocal completed
        completed += 1
        return completed

    last, result = run_workload_warmup(
        operation, minimum_seconds=0.0, minimum_operations=10
    )

    assert last == 10
    assert completed == 10
    assert result.completed_operations == 10


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


def test_json_artifact_checkpoints_completion_and_failure(tmp_path: Path) -> None:
    import json

    import pytest

    completed_path = tmp_path / "completed.json"
    completed = JsonArtifact(completed_path, scheduled_units=2)
    completed.set_active("case-a")
    completed.checkpoint({"records": [{"case": "a"}]}, completed_units=1)
    checkpoint = json.loads(completed_path.read_text())
    assert checkpoint["artifact_state"]["state"] == "running"
    assert checkpoint["artifact_state"]["active_unit"] == "case-a"
    assert checkpoint["artifact_state"]["completed_units"] == 1

    completed.complete({"records": [{"case": "a"}, {"case": "b"}]})
    final = json.loads(completed_path.read_text())
    assert final["artifact_state"]["state"] == "complete"
    assert final["artifact_state"]["complete"] is True
    assert final["artifact_state"]["completed_units"] == 2

    failed_path = tmp_path / "failed.json"
    failed = JsonArtifact(failed_path, scheduled_units=2)
    payload = {"records": []}
    failed.set_active("case-b")
    with pytest.raises(RuntimeError, match="worker failed"):
        with failed.capture_failures(lambda: payload):
            raise RuntimeError("worker failed")
    failure = json.loads(failed_path.read_text())
    assert failure["artifact_state"]["state"] == "failed"
    assert failure["artifact_state"]["active_unit"] == "case-b"
    assert failure["artifact_state"]["failure"]["type"] == "RuntimeError"
    assert list(tmp_path.glob(".*.tmp")) == []


def test_runner_manifest_records_reproducibility_fields(tmp_path: Path) -> None:
    import hashlib

    script = tmp_path / "runner.py"
    script.write_text("print('benchmark')\n")

    manifest = runner_manifest(script)

    assert manifest["script"] == str(script.resolve())
    assert manifest["script_sha256"] == hashlib.sha256(script.read_bytes()).hexdigest()
    assert manifest["command"][0] == sys.executable
    assert manifest["python"]["executable"] == sys.executable
    assert manifest["git"] is None
