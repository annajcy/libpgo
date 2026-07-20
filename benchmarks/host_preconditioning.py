#!/usr/bin/env python3
"""Compatibility CLI for the split benchmark harness support modules."""

from __future__ import annotations

import argparse
import json

from benchmark_support.conditioning import (
    add_benchmark_harness_arguments,
    guard_host_condition,
    precondition_host,
    run_burn_round,
    validate_conditioning_arguments,
)
from benchmark_support.host import (
    available_cpu_ids,
    configure_cpu_placement,
    cpu_topology,
    host_state_snapshot,
    parse_cpu_list,
)
from benchmark_support.schedule import (
    balanced_order,
    order_configuration,
    order_cycle_length,
)


__all__ = [
    "add_host_preconditioning_arguments",
    "balanced_order",
    "guard_host_condition",
    "host_state_snapshot",
    "order_configuration",
    "order_cycle_length",
    "parse_cpu_list",
    "precondition_host",
    "validate_host_preconditioning_arguments",
]


# Preserve the old import names for external diagnostic scripts while runners
# migrate to the responsibility-specific modules.
add_host_preconditioning_arguments = add_benchmark_harness_arguments
validate_host_preconditioning_arguments = validate_conditioning_arguments
_available_cpu_ids = available_cpu_ids
_configure_cpu_placement = configure_cpu_placement
_cpu_topology = cpu_topology
_run_burn_round = run_burn_round


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workers", type=int, default=8)
    add_benchmark_harness_arguments(parser)
    args = parser.parse_args()
    if args.workers <= 0:
        raise SystemExit("--workers must be positive")
    print(json.dumps(precondition_host(args, workers=args.workers), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
