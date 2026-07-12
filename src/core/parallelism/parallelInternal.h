#pragma once

#include "parallelExecutor.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>

#ifdef PGO_PARALLELISM_HAS_TBB
#  include <tbb/global_control.h>
#endif

namespace pgo::parallel::detail
{

constexpr int kParticipantLaneBits = 32;
constexpr std::uint64_t kWorkerParticipant = 1;
constexpr std::uint64_t kExternalParticipant = std::uint64_t{ 1 } << kParticipantLaneBits;

inline std::uint64_t packParticipants(int workers, int external) noexcept
{
  return static_cast<std::uint64_t>(static_cast<std::uint32_t>(workers)) |
    (static_cast<std::uint64_t>(static_cast<std::uint32_t>(external)) << kParticipantLaneBits);
}

inline int workerParticipants(std::uint64_t participants) noexcept
{
  return static_cast<int>(static_cast<std::uint32_t>(participants));
}

inline int externalParticipants(std::uint64_t participants) noexcept
{
  return static_cast<int>(static_cast<std::uint32_t>(participants >> kParticipantLaneBits));
}

inline int totalParticipants(std::uint64_t participants) noexcept
{
  return workerParticipants(participants) + externalParticipants(participants);
}

struct RuntimeCounters
{
  std::atomic<std::uint64_t> currentParticipants{ 0 };
  std::atomic<int> peakTotalParticipants{ 0 };
  std::atomic<bool> participantPressureObserved{ false };
};

struct RuntimeState
{
  RuntimeState(int resolvedMaxConcurrency, bool usingDefaultConcurrency);

  const int resolvedMaxConcurrency;
  const bool usingDefaultConcurrency;
  RuntimeCounters counters;
  std::atomic<int> participantPressureThreshold{ 1 };
#ifdef PGO_PARALLELISM_HAS_TBB
  std::unique_ptr<tbb::global_control> tbbControl;
#endif
};

using ExecutorBody = std::function<void(const ExecutorState *executor)>;

std::shared_ptr<ExecutorState> makeExecutorState(
  std::shared_ptr<RuntimeState> runtime, int maxConcurrency);
void executeInArena(const std::shared_ptr<ExecutorState> &executor, const ExecutorBody &body);

int effectiveTbbMaxAllowedParallelism();

}  // namespace pgo::parallel::detail
