#!/usr/bin/env python3
"""Focused tests for lifecycle benchmark summary classification."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from typing import Any


sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_arena_executor_lifecycle_benchmark as subject  # noqa: E402


def record(
    outer_tasks: int, ratio: float, *, retired_after_measurement: int = 0
) -> dict[str, Any]:
    reuse_ns = 100.0
    recreate_ns = reuse_ns * ratio
    measurements = {
        "reuse": {
            "amortized_wall_nanoseconds_per_inner_call": reuse_ns,
            "amortized_cpu_nanoseconds_per_inner_call": reuse_ns,
            "retired_after_measurement": 0,
        },
        "recreate": {
            "amortized_wall_nanoseconds_per_inner_call": recreate_ns,
            "amortized_cpu_nanoseconds_per_inner_call": recreate_ns,
            "retired_after_measurement": retired_after_measurement,
        },
    }
    return {
        "outer_tasks": outer_tasks,
        "measurements": measurements,
        "recreate_over_reuse": ratio,
        "extra_amortized_wall_ns_per_inner_call": recreate_ns - reuse_ns,
    }


class SummaryClassificationTest(unittest.TestCase):
    def test_classifies_both_directions_equivalence_and_overlap(self) -> None:
        records = [
            record(1, 1.20, retired_after_measurement=1),
            record(1, 1.21),
            record(2, 0.99),
            record(2, 1.01),
            record(3, 0.80),
            record(3, 0.81),
            record(4, 0.90),
            record(4, 1.10),
        ]

        summary = subject.summarize(
            records,
            seed=7,
            bootstrap_samples=500,
            equivalence_band=0.05,
        )
        by_tasks = {row["outer_tasks"]: row for row in summary}

        self.assertEqual(
            by_tasks[1]["evidence"],
            "recreate_slower_beyond_equivalence_band",
        )
        self.assertTrue(by_tasks[1]["retirement_backlog_observed"])
        self.assertEqual(by_tasks[2]["evidence"], "equivalent_within_band")
        self.assertEqual(
            by_tasks[3]["evidence"],
            "recreate_faster_beyond_equivalence_band",
        )
        self.assertEqual(by_tasks[4]["evidence"], "inconclusive")


if __name__ == "__main__":
    unittest.main()
