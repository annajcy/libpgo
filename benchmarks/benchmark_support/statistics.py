"""Deterministic, dependency-free summary statistics for benchmark runners."""

from __future__ import annotations

import math
import random
import statistics
from collections.abc import Sequence


def percentile(values: Sequence[float], probability: float) -> float:
    """Return the linearly interpolated percentile of a non-empty sample."""
    if not values:
        raise ValueError("cannot take a percentile of no values")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = probability * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def bootstrap_median_ci(
    values: Sequence[float], randomizer: random.Random, samples: int
) -> list[float]:
    """Return a two-sided 95% bootstrap confidence interval for the median."""
    if not values:
        raise ValueError("cannot bootstrap an empty sample")
    if samples < 1:
        raise ValueError("bootstrap samples must be positive")
    medians = [
        statistics.median(values[randomizer.randrange(len(values))] for _ in values)
        for _ in range(samples)
    ]
    return [percentile(medians, 0.025), percentile(medians, 0.975)]


def median_absolute_deviation(values: Sequence[float]) -> float:
    if not values:
        raise ValueError("cannot calculate MAD of no values")
    center = statistics.median(values)
    return statistics.median(abs(value - center) for value in values)


def median_and_mad(values: Sequence[float]) -> tuple[float, float]:
    """Return the median and its unscaled median absolute deviation."""
    if not values:
        raise ValueError("cannot summarize no values")
    median = statistics.median(values)
    return median, statistics.median(abs(value - median) for value in values)
