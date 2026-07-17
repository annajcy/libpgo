from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path

import pytest


RUNNER_PATH = (
    Path(__file__).resolve().parents[1]
    / "benchmarks/eigen_mkl_budget_sweep/run_eigen_mkl_budget_sweep.py"
)
SPEC = importlib.util.spec_from_file_location(
    "eigen_mkl_budget_sweep_runner", RUNNER_PATH
)
assert SPEC is not None and SPEC.loader is not None
budget_sweep = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(budget_sweep)


def test_default_arena_concurrencies_cover_powers_of_two_and_limit() -> None:
    assert budget_sweep.default_arena_concurrencies(8) == [1, 2, 4, 8]
    assert budget_sweep.default_arena_concurrencies(6) == [1, 2, 4, 6]


def test_legacy_single_arena_option_remains_supported() -> None:
    args = argparse.Namespace(
        concurrency=8,
        arena_concurrency=4,
        arena_concurrencies=None,
        matrix_n=1024,
        warmup_iterations=3,
        profile_iterations=50,
        timing_repetitions=1,
        profile_repetitions=1,
        mkl_local_thread_budgets=[0, 1],
        outer_tasks=[1],
        profile_outer_tasks=[1],
    )
    budget_sweep.validate_args(args)
    assert args.arena_concurrencies == [4]


def test_surface_jobs_cover_every_arena_budget_pair() -> None:
    args = argparse.Namespace(
        arena_concurrencies=[1, 2, 4, 8],
        mkl_local_thread_budgets=[0, 1, 8],
    )
    jobs = budget_sweep.make_jobs(args, outer_tasks_values=[1], repetitions=3, seed=7)
    assert len(jobs) == 3 * 4 * 3
    for repetition in range(1, 4):
        pairs = {
            (job["arena"], job["budget"])
            for job in jobs
            if job["repetition"] == repetition
        }
        assert pairs == {
            (arena, budget) for arena in [1, 2, 4, 8] for budget in [0, 1, 8]
        }


def test_timing_summary_keeps_arena_as_a_surface_dimension() -> None:
    records = []
    for arena, wall_seconds in [(1, 2.0), (8, 1.0)]:
        records.append(
            {
                "result": {
                    "configured_arena_concurrency": arena,
                    "configured_mkl_local_budget": 1,
                    "outer_tasks": 1,
                    "wall_seconds": wall_seconds,
                    "process_cpu_seconds": wall_seconds,
                    "observed_mkl_max_threads_min": 1,
                    "observed_mkl_max_threads_max": 1,
                }
            }
        )
    summary = budget_sweep.summarize_timing(records)
    assert [entry["configured_arena_concurrency"] for entry in summary] == [1, 8]
    assert [entry["median_wall_seconds"] for entry in summary] == [2.0, 1.0]


def test_arena_surface_rejects_width_larger_than_global_control() -> None:
    args = argparse.Namespace(
        concurrency=4,
        arena_concurrency=None,
        arena_concurrencies=[8],
        matrix_n=1024,
        warmup_iterations=3,
        profile_iterations=50,
        timing_repetitions=1,
        profile_repetitions=1,
        mkl_local_thread_budgets=[0],
        outer_tasks=[1],
        profile_outer_tasks=[1],
    )
    with pytest.raises(ValueError, match="cannot exceed --concurrency"):
        budget_sweep.validate_args(args)
