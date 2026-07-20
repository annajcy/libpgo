#pragma once

#include <chrono>
#include <stdexcept>

namespace pgo::benchmark_helpers
{

struct WorkloadWarmupResult
{
  double elapsedSeconds;
  int completedOperations;
};

template<typename Operation>
WorkloadWarmupResult runWorkloadWarmup(
  Operation &&operation, double minimumSeconds, int minimumOperations)
{
  if (minimumSeconds < 0.0)
    throw std::invalid_argument("warmup minimum seconds must be nonnegative");
  if (minimumOperations < 0)
    throw std::invalid_argument("warmup minimum operations must be nonnegative");

  const auto started = std::chrono::steady_clock::now();
  double elapsed = 0.0;
  int completed = 0;
  while (completed < minimumOperations || elapsed < minimumSeconds) {
    operation();
    ++completed;
    elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started)
                .count();
  }
  return { elapsed, completed };
}

}  // namespace pgo::benchmark_helpers
