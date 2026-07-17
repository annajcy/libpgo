from __future__ import annotations

import argparse
import sys
from pathlib import Path


BENCHMARKS_ROOT = Path(__file__).resolve().parents[1] / "benchmarks"
if str(BENCHMARKS_ROOT) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS_ROOT))

import host_preconditioning  # noqa: E402
from host_preconditioning import (  # noqa: E402
    _run_burn_round,
    balanced_order,
    guard_host_condition,
    precondition_host,
)


def test_balanced_order_covers_every_policy_position() -> None:
    policies = ["a", "b", "c", "d"]
    orders = [
        balanced_order(
            policies,
            repetition=repetition,
            seed=17,
            block_key="workload",
        )
        for repetition in range(len(policies))
    ]
    for position in range(len(policies)):
        assert {order[position] for order in orders} == set(policies)


def test_skipped_preconditioning_is_recorded() -> None:
    args = argparse.Namespace(
        host_preheat_seconds=10.0,
        host_preheat_workers=None,
        host_probe_seconds=1.0,
        host_required_stable_probes=3,
        host_max_probes=6,
        host_stability_tolerance=0.02,
        host_drift_tolerance=0.05,
        host_abort_drift_tolerance=0.15,
        skip_host_preconditioning=True,
    )
    result = precondition_host(args, workers=2)
    assert result["enabled"] is False
    assert result["status"] == "skipped"


def test_burn_round_runs_requested_workers() -> None:
    result = _run_burn_round(0.02, 2)
    assert result["workers"] == 2
    assert result["iterations"] > 0
    assert result["iterations_per_second"] > 0


def test_block_guard_compares_against_initial_baseline(monkeypatch) -> None:
    args = argparse.Namespace(
        host_probe_seconds=1.0,
        host_required_stable_probes=2,
        host_stability_tolerance=0.02,
        host_drift_tolerance=0.05,
        host_abort_drift_tolerance=0.15,
    )
    preconditioning = {
        "enabled": True,
        "dry_run": False,
        "workers": 2,
        "probes": [
            {"iterations_per_second": 100.0},
            {"iterations_per_second": 100.0},
        ],
    }
    monkeypatch.setattr(
        host_preconditioning,
        "_run_burn_round",
        lambda duration, workers: {"iterations_per_second": 101.0},
    )
    check = guard_host_condition(args, preconditioning, label="block")
    assert check["status"] == "stable"
    assert check["relative_deviation"] < 0.011
    assert preconditioning["block_checks"] == [check]


def test_block_guard_records_moderate_drift(monkeypatch) -> None:
    args = argparse.Namespace(
        host_probe_seconds=1.0,
        host_required_stable_probes=2,
        host_stability_tolerance=0.02,
        host_drift_tolerance=0.05,
        host_abort_drift_tolerance=0.15,
    )
    preconditioning = {
        "enabled": True,
        "dry_run": False,
        "workers": 2,
        "probes": [
            {"iterations_per_second": 100.0},
            {"iterations_per_second": 100.0},
        ],
    }
    monkeypatch.setattr(
        host_preconditioning,
        "_run_burn_round",
        lambda duration, workers: {"iterations_per_second": 110.0},
    )
    check = guard_host_condition(args, preconditioning, label="block")
    assert check["status"] == "drifted"
    assert len(check["probes"]) == 2
