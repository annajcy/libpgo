#pragma once

#include "benchmark_argument_parser.h"

#include <chrono>
#include <stdexcept>

namespace pgo::benchmark_helpers
{

struct TimedMeasurementArguments
{
  double minimumSeconds;
  int minimumOperations;
};

inline TimedMeasurementArguments parseTimedMeasurementArguments(
  int argc, char **argv)
{
  const double minimumSeconds = parseNonnegativeDouble(
    requireValue(argc, argv, "--measurement-min-seconds="),
    "--measurement-min-seconds");
  const int minimumOperations = parseNonnegativeInteger(
    requireValue(argc, argv, "--measurement-min-operations="),
    "--measurement-min-operations");
  if (minimumSeconds == 0.0 && minimumOperations == 0)
    throw std::invalid_argument("measurement must require time or operations.");
  return { minimumSeconds, minimumOperations };
}

struct TimedMeasurementResult
{
  double elapsedSeconds;
  int completedOperations;
};

template<typename Operation>
TimedMeasurementResult runTimedMeasurement(Operation &&operation,
  const TimedMeasurementArguments &arguments)
{
  const auto started = std::chrono::steady_clock::now();
  double elapsed = 0.0;
  int completed = 0;
  while (completed < arguments.minimumOperations ||
    elapsed < arguments.minimumSeconds) {
    operation();
    ++completed;
    elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started)
                .count();
  }
  return { elapsed, completed };
}

}  // namespace pgo::benchmark_helpers
