"""Workload-specific warmup execution and offline plateau calibration."""

from __future__ import annotations

import argparse
import math
import random
import statistics
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any, TypeVar

from benchmark_support.schedule import balanced_order, order_configuration
from benchmark_support.statistics import bootstrap_median_ci, median_absolute_deviation


_T = TypeVar("_T")
DEFAULT_CALIBRATION_CANDIDATES = (0.0, 0.25, 0.5, 1.0, 2.0, 4.0)


@dataclass(frozen=True)
class WarmupResult:
    elapsed_seconds: float
    completed_operations: int


def parse_warmup_candidates(value: str) -> tuple[float, ...]:
    try:
        candidates = tuple(float(item.strip()) for item in value.split(","))
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "warmup candidates must be comma-separated seconds"
        ) from error
    if not candidates or any(
        not math.isfinite(item) or item < 0 for item in candidates
    ):
        raise argparse.ArgumentTypeError(
            "warmup candidates must be finite nonnegative seconds"
        )
    if len(set(candidates)) != len(candidates):
        raise argparse.ArgumentTypeError(
            "warmup candidates must not contain duplicates"
        )
    return tuple(sorted(candidates))


def add_workload_warmup_arguments(
    parser: argparse.ArgumentParser,
    *,
    default_seconds: float,
    default_min_operations: int,
) -> None:
    group = parser.add_argument_group("workload-specific warmup")
    group.add_argument(
        "--warmup-seconds",
        type=float,
        default=default_seconds,
        help=(
            "Minimum time spent running the exact workload before measurement; "
            "zero makes the operation count the only stopping condition."
        ),
    )
    group.add_argument(
        "--warmup-min-operations",
        type=int,
        default=default_min_operations,
        help="Minimum complete workload operations before measurement.",
    )


def validate_workload_warmup_arguments(args: argparse.Namespace) -> None:
    if not math.isfinite(args.warmup_seconds) or args.warmup_seconds < 0:
        raise ValueError("--warmup-seconds must be finite and nonnegative")
    if args.warmup_min_operations < 0:
        raise ValueError("--warmup-min-operations must be nonnegative")
    if args.warmup_seconds == 0 and args.warmup_min_operations == 0:
        return
    if args.warmup_min_operations == 0:
        raise ValueError(
            "--warmup-min-operations must be positive when warmup time is nonzero"
        )


def workload_warmup_configuration(args: argparse.Namespace) -> dict[str, Any]:
    validate_workload_warmup_arguments(args)
    return {
        "kind": "same_workload_same_policy",
        "minimum_seconds": args.warmup_seconds,
        "minimum_operations": args.warmup_min_operations,
        "completion_rule": "elapsed_seconds >= minimum_seconds AND completed_operations >= minimum_operations",
        "adaptive_timing_stop": False,
    }


def run_workload_warmup(
    operation: Callable[[], _T],
    *,
    minimum_seconds: float,
    minimum_operations: int,
    clock: Callable[[], float] = time.perf_counter,
) -> tuple[_T | None, WarmupResult]:
    """Run an exact workload until both fixed lower bounds are satisfied."""
    if not math.isfinite(minimum_seconds) or minimum_seconds < 0:
        raise ValueError("minimum_seconds must be finite and nonnegative")
    if minimum_operations < 0:
        raise ValueError("minimum_operations must be nonnegative")
    started = clock()
    completed = 0
    last: _T | None = None
    elapsed = 0.0
    while completed < minimum_operations or elapsed < minimum_seconds:
        last = operation()
        completed += 1
        elapsed = clock() - started
    return last, WarmupResult(elapsed, completed)


def calibration_order(
    subjects: Sequence[str],
    candidates: Sequence[float],
    repetitions: int,
    *,
    seed: int,
) -> tuple[list[tuple[int, str, float]], dict[str, Any]]:
    """Build complete, counterbalanced candidate orders for every subject."""
    order = order_configuration(
        repetitions,
        [("warmup_candidates", candidates)],
    )
    blocks = [
        (repetition, subject)
        for repetition in range(repetitions)
        for subject in subjects
    ]
    random.Random(seed).shuffle(blocks)
    schedule = [
        (repetition, subject, candidate)
        for repetition, subject in blocks
        for candidate in balanced_order(
            candidates,
            repetition=repetition,
            seed=seed,
            block_key=subject,
        )
    ]
    return schedule, order


def analyze_warmup_calibration(
    records: Sequence[dict[str, Any]],
    candidates: Sequence[float],
    *,
    seed: int,
    bootstrap_samples: int,
    median_tolerance: float = 0.01,
    ci_tolerance: float = 0.02,
) -> list[dict[str, Any]]:
    """Select the smallest T equivalent to both 2T and 4T for each subject."""
    if not 0 < median_tolerance <= ci_tolerance < 1:
        raise ValueError("require 0 < median_tolerance <= ci_tolerance < 1")
    candidate_set = set(candidates)
    subjects = sorted({str(record["subject"]) for record in records})
    summaries = []
    for subject in subjects:
        samples: dict[float, dict[int, float]] = {value: {} for value in candidates}
        for record in records:
            if record["subject"] != subject:
                continue
            samples[float(record["warmup_seconds"])][int(record["repetition"])] = float(
                record["measurement_seconds"]
            )
        candidate_summaries = []
        for candidate in candidates:
            values = list(samples[candidate].values())
            candidate_summaries.append(
                {
                    "warmup_seconds": candidate,
                    "sample_count": len(values),
                    "median_measurement_seconds": statistics.median(values),
                    "mad_measurement_seconds": median_absolute_deviation(values),
                }
            )

        selected = None
        comparisons = []
        for candidate in candidates:
            if (
                candidate <= 0
                or 2 * candidate not in candidate_set
                or 4 * candidate not in candidate_set
            ):
                continue
            candidate_passes = True
            for reference in (2 * candidate, 4 * candidate):
                repetitions = sorted(set(samples[candidate]) & set(samples[reference]))
                ratios = [
                    samples[candidate][repetition] / samples[reference][repetition]
                    for repetition in repetitions
                ]
                median_ratio = statistics.median(ratios)
                interval = bootstrap_median_ci(
                    ratios,
                    random.Random(f"{seed}:{subject}:{candidate}:{reference}"),
                    bootstrap_samples,
                )
                equivalent = (
                    abs(median_ratio - 1.0) <= median_tolerance
                    and interval[0] >= 1.0 - ci_tolerance
                    and interval[1] <= 1.0 + ci_tolerance
                )
                comparisons.append(
                    {
                        "warmup_seconds": candidate,
                        "reference_seconds": reference,
                        "paired_sample_count": len(ratios),
                        "median_ratio": median_ratio,
                        "bootstrap_95pct_ci": interval,
                        "equivalent": equivalent,
                    }
                )
                candidate_passes = candidate_passes and equivalent
            if candidate_passes and selected is None:
                selected = candidate
        summaries.append(
            {
                "subject": subject,
                "selected_warmup_seconds": selected,
                "status": "plateau_found"
                if selected is not None
                else "extend_candidates",
                "next_candidate_seconds": None
                if selected is not None
                else 2 * max(candidates),
                "candidate_summaries": candidate_summaries,
                "comparisons": comparisons,
            }
        )
    return summaries
