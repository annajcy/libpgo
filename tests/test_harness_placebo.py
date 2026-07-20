from __future__ import annotations

import sys
from pathlib import Path

import pytest


BENCHMARKS_ROOT = Path(__file__).resolve().parents[1] / "benchmarks"
if str(BENCHMARKS_ROOT) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS_ROOT))

from run_harness_placebo import (  # noqa: E402
    analyze_records,
    equivalence_failures,
    validate_identical_results,
)


def benchmark_row(seconds: float) -> dict[str, float]:
    return {
        "wall_seconds": seconds,
        "matrix_n": 128,
        "requested_concurrency": 8,
        "effective_global_concurrency": 8,
        "configured_arena_concurrency": 8,
        "configured_mkl_local_budget": 1,
        "observed_arena_concurrency": 8,
        "checksum": 123.0,
    }


def guard(throughput: float) -> dict[str, object]:
    return {"status": "stable", "probes": [{"iterations_per_second": throughput}]}


def test_identical_placebo_records_pass_equivalence_gate() -> None:
    records = []
    for repetition in range(16):
        order = ["placebo_a", "placebo_b"]
        if repetition % 2:
            order.reverse()
        records.append(
            {
                "order": order,
                "measurements": {
                    "placebo_a": benchmark_row(1.0),
                    "placebo_b": benchmark_row(1.0),
                },
                "guards": {
                    "placebo_a": guard(100.0),
                    "placebo_b": guard(100.0),
                },
            }
        )

    summaries = analyze_records(records, seed=17, bootstrap_samples=200)
    assert equivalence_failures(
        summaries, median_tolerance=0.02, ci_tolerance=0.05
    ) == []


def test_placebo_gate_detects_label_and_position_bias() -> None:
    records = [
        {
            "order": ["placebo_a", "placebo_b"],
            "measurements": {
                "placebo_a": benchmark_row(1.0),
                "placebo_b": benchmark_row(1.1),
            },
            "guards": {
                "placebo_a": guard(100.0),
                "placebo_b": guard(110.0),
            },
        }
        for _ in range(8)
    ]
    summaries = analyze_records(records, seed=17, bootstrap_samples=100)
    failures = equivalence_failures(
        summaries, median_tolerance=0.02, ci_tolerance=0.05
    )
    assert any("workload_placebo_b_over_a median" in failure for failure in failures)
    assert any("guard_second_over_first median" in failure for failure in failures)


def test_placebo_correctness_validation_rejects_counter_mismatch() -> None:
    measurements = {
        "placebo_a": benchmark_row(1.0),
        "placebo_b": benchmark_row(1.0),
    }
    measurements["placebo_b"]["configured_mkl_local_budget"] = 8

    with pytest.raises(RuntimeError, match="configured_mkl_local_budget"):
        validate_identical_results(measurements, checksum_relative_tolerance=1e-10)
