"""Deterministic counterbalanced ordering for benchmark cases."""

from __future__ import annotations

import argparse
import hashlib
import math
from typing import Any, Sequence, TypeVar


_T = TypeVar("_T")
ORDER_DESIGN = "williams_first_order_carryover"


def add_schedule_arguments(group: argparse._ArgumentGroup) -> None:
    group.add_argument(
        "--allow-incomplete-order-cycle",
        action="store_true",
        help=(
            "Permit a diagnostic run whose repetitions or truncation do not form "
            "complete counterbalanced order cycles."
        ),
    )


def balanced_order(
    values: Sequence[_T],
    *,
    repetition: int,
    seed: int,
    block_key: str,
) -> list[_T]:
    """Return a deterministic first-order carry-over-balanced policy order."""
    if not values:
        return []
    digest = hashlib.sha256(f"{seed}:{block_key}".encode()).digest()
    keyed = sorted(
        values,
        key=lambda value: hashlib.sha256(digest + repr(value).encode()).digest(),
    )
    count = len(keyed)
    if count == 1:
        return keyed

    base_indices = [0]
    for position in range(1, count):
        if position % 2:
            base_indices.append((position + 1) // 2)
        else:
            base_indices.append(count - position // 2)

    if count % 2:
        cycle_length = 2 * count
        within_cycle = repetition % cycle_length
        row = within_cycle % count
        reverse = within_cycle >= count
    else:
        row = repetition % count
        reverse = False

    order = [keyed[(index + row) % count] for index in base_indices]
    if reverse:
        order.reverse()
    return order


def order_cycle_length(value_count: int) -> int:
    """Return the number of repetitions in one complete Williams cycle."""
    if value_count <= 0:
        raise ValueError("counterbalanced order requires at least one value")
    if value_count == 1:
        return 1
    return value_count if value_count % 2 == 0 else 2 * value_count


def order_configuration(
    repetitions: int,
    groups: Sequence[tuple[str, Sequence[Any]]],
    *,
    allow_incomplete: bool = False,
    schedule_truncated: bool = False,
) -> dict[str, Any]:
    """Validate and describe the complete counterbalanced scheduling cycle."""
    if repetitions <= 0:
        raise ValueError("counterbalanced repetitions must be positive")
    group_records = []
    cycle_lengths = []
    for name, values in groups:
        cycle_length = order_cycle_length(len(values))
        cycle_lengths.append(cycle_length)
        group_records.append(
            {
                "name": name,
                "value_count": len(values),
                "order_cycle_length": cycle_length,
            }
        )
    if not cycle_lengths:
        raise ValueError("counterbalanced order requires at least one value group")

    combined_cycle_length = math.lcm(*cycle_lengths)
    complete = repetitions % combined_cycle_length == 0 and not schedule_truncated
    if not complete and not allow_incomplete:
        reasons = []
        if repetitions % combined_cycle_length:
            reasons.append(
                f"{repetitions} repetitions is not a multiple of "
                f"order_cycle_length={combined_cycle_length}"
            )
        if schedule_truncated:
            reasons.append("the requested case limit truncates a repetition block")
        raise ValueError(
            "; ".join(reasons)
            + ". Use a complete cycle or pass --allow-incomplete-order-cycle "
            "for a diagnostic run."
        )

    return {
        "design": ORDER_DESIGN,
        "repetitions": repetitions,
        "order_cycle_length": combined_cycle_length,
        "completed_order_cycles": repetitions // combined_cycle_length,
        "strictly_balanced": complete,
        "schedule_truncated": schedule_truncated,
        "groups": group_records,
    }
