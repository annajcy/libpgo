from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pytest


BENCHMARKS_ROOT = Path(__file__).resolve().parents[1] / "benchmarks"
if str(BENCHMARKS_ROOT) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS_ROOT))
from benchmark_support import harness, host  # noqa: E402
from benchmark_support.harness import prepare_benchmark_host  # noqa: E402
from benchmark_support.host import configure_cpu_placement, parse_cpu_list  # noqa: E402
from benchmark_support.schedule import (  # noqa: E402
    balanced_order,
    order_configuration,
    order_cycle_length,
)
from benchmark_support.warmup import (  # noqa: E402
    analyze_warmup_calibration,
    calibration_order,
    parse_warmup_candidates,
    run_workload_warmup,
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


def test_prepare_host_records_provenance_without_proxy_workload(monkeypatch) -> None:
    placement = {"actual_cpu_list": [0, 1]}
    monkeypatch.setattr(
        harness,
        "configure_cpu_placement",
        lambda args, expected_cpu_count, dry_run: placement,
    )
    monkeypatch.setattr(
        harness,
        "host_state_snapshot",
        lambda cpu_ids: {"cpu_ids": cpu_ids},
    )
    result = prepare_benchmark_host(argparse.Namespace(), workers=2)
    assert result["protocol"] == "workload_specific_warmup"
    assert result["cpu_placement"] is placement
    assert result["state_before"] == {"cpu_ids": [0, 1]}
    assert "preheat" not in result
    assert "probes" not in result


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
    monkeypatch.setattr(host, "available_cpu_ids", lambda: [21, 22])
    monkeypatch.setattr(
        host,
        "cpu_topology",
        lambda cpu: {"cpu": cpu, "core": cpu, "socket": 0, "numa_node": 0},
    )
    args = argparse.Namespace(cpu_list=[21, 22], numa_node=0, allow_cross_numa=False)
    placement = configure_cpu_placement(args, expected_cpu_count=2, dry_run=False)
    assert applied == [{21, 22}]
    assert placement["actual_cpu_list"] == [21, 22]
    assert placement["numa_nodes"] == [0]
    assert placement["affinity_enforced"] is True


def test_inherited_affinity_must_match_benchmark_concurrency(monkeypatch) -> None:
    monkeypatch.setattr(host, "available_cpu_ids", lambda: [21, 22, 23])
    args = argparse.Namespace(cpu_list=None, numa_node=None, allow_cross_numa=False)
    with pytest.raises(RuntimeError, match="requires exactly 2 affinity CPUs"):
        configure_cpu_placement(args, expected_cpu_count=2, dry_run=False)


def test_cross_numa_affinity_is_rejected_by_default(monkeypatch) -> None:
    monkeypatch.setattr(host, "available_cpu_ids", lambda: [21, 22])
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


def test_workload_warmup_requires_both_time_and_operation_bounds() -> None:
    now = [0.0]

    def operation() -> int:
        now[0] += 0.4
        return round(now[0] * 10)

    last, result = run_workload_warmup(
        operation,
        minimum_seconds=1.0,
        minimum_operations=2,
        clock=lambda: now[0],
    )
    assert last == 12
    assert result.completed_operations == 3
    assert result.elapsed_seconds == pytest.approx(1.2)


def test_calibration_order_is_complete_and_counterbalanced() -> None:
    candidates = (0.0, 0.25, 0.5, 1.0, 2.0, 4.0)
    schedule, order = calibration_order(["case"], candidates, 6, seed=17)
    assert len(schedule) == 36
    assert order["strictly_balanced"] is True
    for position in range(len(candidates)):
        assert {schedule[repetition * 6 + position][2] for repetition in range(6)} == set(candidates)


def test_calibration_selects_smallest_equivalent_t_2t_4t_plateau() -> None:
    candidates = (0.25, 0.5, 1.0, 2.0)
    timings = {0.25: 1.10, 0.5: 1.0, 1.0: 1.005, 2.0: 0.998}
    records = [
        {
            "subject": "case",
            "repetition": repetition,
            "warmup_seconds": candidate,
            "measurement_seconds": timing,
        }
        for repetition in range(6)
        for candidate, timing in timings.items()
    ]
    summary = analyze_warmup_calibration(
        records,
        candidates,
        seed=17,
        bootstrap_samples=100,
    )
    assert summary[0]["selected_warmup_seconds"] == 0.5
    assert summary[0]["status"] == "plateau_found"


def test_parse_warmup_candidates_rejects_duplicates() -> None:
    assert parse_warmup_candidates("0,0.25,1") == (0.0, 0.25, 1.0)
    with pytest.raises(argparse.ArgumentTypeError, match="duplicates"):
        parse_warmup_candidates("0,1,1")
