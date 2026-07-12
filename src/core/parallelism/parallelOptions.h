#pragma once

#include <optional>

namespace pgo::parallel
{

enum class NestedKernelPolicy
{
  Suppress,
  Inherit,
};

struct Options
{
  int grainSize = 0;
  NestedKernelPolicy nestedKernelPolicy = NestedKernelPolicy::Suppress;
};

struct RuntimeOptions
{
  std::optional<int> maxTbbConcurrency;
};

struct ExecutorOptions
{
  std::optional<int> maxConcurrency;
};

struct RuntimeInfo
{
  bool initialized = false;
  bool usingDefaultConcurrency = false;
  std::optional<int> maxConcurrency;
  int defaultConcurrency = 1;
  int effectiveTbbMaxAllowedParallelism = 1;
  int tbbWorkerCeiling = 0;
  int currentWorkerParticipants = 0;
  int currentExternalParticipants = 0;
  int currentTotalParticipants = 0;
  int peakTotalParticipants = 0;
  bool participantPressureObserved = false;
};

}  // namespace pgo::parallel
