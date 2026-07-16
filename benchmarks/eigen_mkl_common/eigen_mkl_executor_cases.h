#pragma once

#include <array>
#include <stdexcept>
#include <string_view>

namespace pgo::benchmark_helpers
{

enum class MklExecutorCase
{
  ExecutorDefault = 0,
  ExecutorLocal1 = 1,
  ExecutorDefaultArena1 = 2,
  ExecutorLocal1Arena1 = 3,
  ExecutorMKLC = 4,
  ExecutorMKLCArena1 = 5,
};

inline constexpr std::array allMklExecutorCases = {
  MklExecutorCase::ExecutorDefault,
  MklExecutorCase::ExecutorLocal1,
  MklExecutorCase::ExecutorDefaultArena1,
  MklExecutorCase::ExecutorLocal1Arena1,
  MklExecutorCase::ExecutorMKLC,
  MklExecutorCase::ExecutorMKLCArena1,
};

struct MklExecutorSpec
{
  int arenaConcurrency;
  int mklLocalThreadBudget;
};

inline const char *mklExecutorCaseName(MklExecutorCase policy) noexcept
{
  switch (policy) {
  case MklExecutorCase::ExecutorDefault:
    return "ExecutorDefault";
  case MklExecutorCase::ExecutorLocal1:
    return "ExecutorLocal1";
  case MklExecutorCase::ExecutorDefaultArena1:
    return "ExecutorDefaultArena1";
  case MklExecutorCase::ExecutorLocal1Arena1:
    return "ExecutorLocal1Arena1";
  case MklExecutorCase::ExecutorMKLC:
    return "ExecutorMKLC";
  case MklExecutorCase::ExecutorMKLCArena1:
    return "ExecutorMKLCArena1";
  }
  return "Unknown";
}

inline MklExecutorCase parseMklExecutorCase(std::string_view value)
{
  for (MklExecutorCase policy : allMklExecutorCases) {
    if (value == mklExecutorCaseName(policy))
      return policy;
  }
  throw std::invalid_argument("Unknown MKL executor case.");
}

inline MklExecutorSpec mklExecutorSpec(
  MklExecutorCase policy, int configuredConcurrency)
{
  if (configuredConcurrency <= 0)
    throw std::invalid_argument("Configured concurrency must be positive.");

  switch (policy) {
  case MklExecutorCase::ExecutorDefault:
    return { configuredConcurrency, 0 };
  case MklExecutorCase::ExecutorLocal1:
    return { configuredConcurrency, 1 };
  case MklExecutorCase::ExecutorDefaultArena1:
    return { 1, 0 };
  case MklExecutorCase::ExecutorLocal1Arena1:
    return { 1, 1 };
  case MklExecutorCase::ExecutorMKLC:
    return { configuredConcurrency, configuredConcurrency };
  case MklExecutorCase::ExecutorMKLCArena1:
    return { 1, configuredConcurrency };
  }
  throw std::invalid_argument("Invalid MKL executor case.");
}

}  // namespace pgo::benchmark_helpers
