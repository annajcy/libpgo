from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pytest


BENCHMARKS_ROOT = Path(__file__).resolve().parents[1] / "benchmarks"
if str(BENCHMARKS_ROOT) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS_ROOT))

from benchmark_support import conditioning, host  # noqa: E402
from benchmark_support.conditioning import (  # noqa: E402
    guard_host_condition,
    precondition_host,
    run_burn_round,
)
from benchmark_support.host import configure_cpu_placement, parse_cpu_list  # noqa: E402
from benchmark_support.schedule import (  # noqa: E402
    balanced_order,
    order_configuration,
    order_cycle_length,
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


@pytest.mark.parametrize("policy_count", [4, 5, 6])
def test_balanced_order_balances_first_order_carryover(policy_count: int) -> None:
    policies = [f"p{index}" for index in range(policy_count)]
    cycle_length = policy_count if policy_count % 2 == 0 else 2 * policy_count
    orders = [
        balanced_order(
            policies,
            repetition=repetition,
            seed=17,
            block_key="workload",
        )
        for repetition in range(cycle_length)
    ]
    transitions = {
        (left, right): 0
        for left in policies
        for right in policies
        if left != right
    }
    for order in orders:
        for left, right in zip(order, order[1:]):
            transitions[(left, right)] += 1

    position_counts = {
        (position, policy): sum(order[position] == policy for order in orders)
        for position in range(policy_count)
        for policy in policies
    }
    assert min(position_counts.values()) == max(position_counts.values())
    assert min(transitions.values()) == max(transitions.values())


@pytest.mark.parametrize(
    ("value_count", "expected"), [(1, 1), (2, 2), (3, 6), (4, 4), (5, 10)]
)
def test_order_cycle_length(value_count: int, expected: int) -> None:
    assert order_cycle_length(value_count) == expected


def test_order_configuration_requires_complete_combined_cycle() -> None:
    with pytest.raises(ValueError, match="order_cycle_length=12"):
        order_configuration(
            10,
            [("arenas", [1, 2, 4, 8]), ("budgets", [0, 1, 2, 4, 8, 16])],
        )

    configuration = order_configuration(
        24,
        [("arenas", [1, 2, 4, 8]), ("budgets", [0, 1, 2, 4, 8, 16])],
    )
    assert configuration["order_cycle_length"] == 12
    assert configuration["completed_order_cycles"] == 2
    assert configuration["strictly_balanced"] is True


def test_incomplete_order_cycle_requires_explicit_diagnostic_override() -> None:
    configuration = order_configuration(
        7,
        [("policies", ["a", "b", "c"])],
        allow_incomplete=True,
    )
    assert configuration["order_cycle_length"] == 6
    assert configuration["strictly_balanced"] is False


def test_skipped_preconditioning_is_recorded(monkeypatch) -> None:
    monkeypatch.setattr(host, "available_cpu_ids", lambda: [0, 1])
    monkeypatch.setattr(
        host,
        "cpu_topology",
        lambda cpu: {"cpu": cpu, "core": cpu, "socket": 0, "numa_node": 0},
    )
    args = argparse.Namespace(
        host_preheat_seconds=10.0,
        host_preheat_workers=None,
        host_probe_seconds=1.0,
        host_required_stable_probes=3,
        host_max_probes=6,
        host_stability_tolerance=0.02,
        host_drift_tolerance=0.05,
        host_abort_drift_tolerance=0.50,
        skip_host_preconditioning=True,
    )
    result = precondition_host(args, workers=2)
    assert result["enabled"] is False
    assert result["strict_timing_comparability"] is False
    assert result["status"] == "skipped"


def test_parse_cpu_list_expands_ranges_and_removes_duplicates() -> None:
    assert parse_cpu_list("21-23,22,27") == [21, 22, 23, 27]
    with pytest.raises(ValueError, match="descending CPU range"):
        parse_cpu_list("23-21")


def test_explicit_cpu_list_is_applied_and_verified(monkeypatch) -> None:
    applied: list[set[int]] = []
    monkeypatch.setattr(
        host.os,
        "sched_setaffinity",
        lambda pid, cpus: applied.append(set(cpus)),
        raising=False,
    )
    monkeypatch.setattr(
        host, "available_cpu_ids", lambda: [21, 22]
    )
    monkeypatch.setattr(
        host,
        "cpu_topology",
        lambda cpu: {"cpu": cpu, "core": cpu, "socket": 0, "numa_node": 0},
    )
    args = argparse.Namespace(
        cpu_list=[21, 22], numa_node=0, allow_cross_numa=False
    )

    placement = configure_cpu_placement(
        args, expected_cpu_count=2, dry_run=False
    )

    assert applied == [{21, 22}]
    assert placement["actual_cpu_list"] == [21, 22]
    assert placement["numa_nodes"] == [0]
    assert placement["affinity_enforced"] is True


def test_inherited_affinity_must_match_benchmark_concurrency(monkeypatch) -> None:
    monkeypatch.setattr(
        host, "available_cpu_ids", lambda: [21, 22, 23]
    )
    args = argparse.Namespace(cpu_list=None, numa_node=None, allow_cross_numa=False)

    with pytest.raises(RuntimeError, match="requires exactly 2 affinity CPUs"):
        configure_cpu_placement(args, expected_cpu_count=2, dry_run=False)


def test_cross_numa_affinity_is_rejected_by_default(monkeypatch) -> None:
    monkeypatch.setattr(
        host, "available_cpu_ids", lambda: [21, 22]
    )
    monkeypatch.setattr(
        host,
        "cpu_topology",
        lambda cpu: {
            "cpu": cpu,
            "core": cpu,
            "socket": cpu - 21,
            "numa_node": cpu - 21,
        },
    )
    args = argparse.Namespace(cpu_list=None, numa_node=None, allow_cross_numa=False)

    with pytest.raises(RuntimeError, match="span NUMA nodes"):
        configure_cpu_placement(args, expected_cpu_count=2, dry_run=False)


def test_burn_round_runs_requested_workers() -> None:
    result = run_burn_round(0.02, 2)
    assert result["workers"] == 2
    assert result["iterations"] > 0
    assert result["iterations_per_second"] > 0


def test_block_guard_compares_against_initial_baseline(monkeypatch) -> None:
    args = argparse.Namespace(
        host_probe_seconds=1.0,
        host_required_stable_probes=2,
        host_stability_tolerance=0.02,
        host_drift_tolerance=0.05,
        host_abort_drift_tolerance=0.50,
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
        conditioning,
        "run_burn_round",
        lambda duration, workers: {"iterations_per_second": 101.0},
    )
    check = guard_host_condition(args, preconditioning, label="block")
    assert check["status"] == "stable"
    assert check["relative_deviation"] < 0.011
    assert preconditioning["block_checks"] == [check]


def test_block_guard_rejects_moderate_drift_by_default(monkeypatch) -> None:
    args = argparse.Namespace(
        host_probe_seconds=1.0,
        host_required_stable_probes=2,
        host_stability_tolerance=0.02,
        host_drift_tolerance=0.05,
        host_abort_drift_tolerance=0.50,
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
        conditioning,
        "run_burn_round",
        lambda duration, workers: {"iterations_per_second": 110.0},
    )
    with pytest.raises(RuntimeError, match="allowed limit 5.00%"):
        guard_host_condition(args, preconditioning, label="block")
    assert preconditioning["block_checks"][0]["status"] == "drifted"
    assert len(preconditioning["block_checks"][0]["probes"]) == 2


def test_block_guard_can_explicitly_record_moderate_drift(monkeypatch) -> None:
    args = argparse.Namespace(
        host_probe_seconds=1.0,
        host_required_stable_probes=2,
        host_stability_tolerance=0.02,
        host_drift_tolerance=0.05,
        host_abort_drift_tolerance=0.50,
        allow_host_drift=True,
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
        conditioning,
        "run_burn_round",
        lambda duration, workers: {"iterations_per_second": 110.0},
    )
    check = guard_host_condition(args, preconditioning, label="block")
    assert check["status"] == "drifted"
    assert preconditioning["strict_timing_comparability"] is False
