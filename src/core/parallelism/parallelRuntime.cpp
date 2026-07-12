#include "parallelism/parallelRuntime.h"

#include "parallelInternal.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>

#ifdef PGO_PARALLELISM_HAS_TBB
#  include <tbb/info.h>
#endif

namespace pgo::parallel
{
namespace detail
{

RuntimeState::RuntimeState(int resolvedMaxConcurrency_, bool usingDefaultConcurrency_):
  resolvedMaxConcurrency(resolvedMaxConcurrency_),
  usingDefaultConcurrency(usingDefaultConcurrency_)
#ifdef PGO_PARALLELISM_HAS_TBB
  ,
  tbbControl(std::make_unique<tbb::global_control>(
    tbb::global_control::max_allowed_parallelism, static_cast<std::size_t>(resolvedMaxConcurrency_)))
#endif
{
#ifdef PGO_PARALLELISM_HAS_TBB
  participantPressureThreshold.store(
    effectiveTbbMaxAllowedParallelism(), std::memory_order_relaxed);
#endif
}

int effectiveTbbMaxAllowedParallelism()
{
#ifdef PGO_PARALLELISM_HAS_TBB
  const std::size_t value =
    tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism);
  return static_cast<int>(
    std::min<std::size_t>(value, static_cast<std::size_t>(std::numeric_limits<int>::max())));
#else
  return 1;
#endif
}

}  // namespace detail

namespace
{

std::atomic<ParallelRuntime *> &publishedRuntime()
{
  static std::atomic<ParallelRuntime *> value{ nullptr };
  return value;
}

std::mutex &runtimeInitializationMutex()
{
  static std::mutex mutex;
  return mutex;
}

void validatePositive(std::optional<int> value, const char *name)
{
  if (value.has_value() && *value <= 0)
    throw std::invalid_argument(std::string(name) + " must be positive or unset.");
}

int queriedDefaultConcurrency()
{
#ifdef PGO_PARALLELISM_HAS_TBB
  return std::max(1, tbb::info::default_concurrency());
#else
  return 1;
#endif
}

int resolveRuntimeConcurrency(const RuntimeOptions &options)
{
  validatePositive(options.maxTbbConcurrency, "maxTbbConcurrency");
#ifdef PGO_PARALLELISM_HAS_TBB
  return options.maxTbbConcurrency.value_or(queriedDefaultConcurrency());
#else
  return 1;
#endif
}

RuntimeInfo makeRuntimeInfo(detail::RuntimeState *state)
{
  RuntimeInfo info;
  info.defaultConcurrency = queriedDefaultConcurrency();
  info.effectiveTbbMaxAllowedParallelism = detail::effectiveTbbMaxAllowedParallelism();
  info.tbbWorkerCeiling = std::max(0, info.effectiveTbbMaxAllowedParallelism - 1);
  if (state == nullptr)
    return info;

  state->participantPressureThreshold.store(
    info.effectiveTbbMaxAllowedParallelism, std::memory_order_relaxed);

  info.initialized = true;
  info.usingDefaultConcurrency = state->usingDefaultConcurrency;
  info.maxConcurrency = state->resolvedMaxConcurrency;
  const std::uint64_t participants =
    state->counters.currentParticipants.load(std::memory_order_relaxed);
  info.currentWorkerParticipants = detail::workerParticipants(participants);
  info.currentExternalParticipants = detail::externalParticipants(participants);
  info.currentTotalParticipants = detail::totalParticipants(participants);
  info.peakTotalParticipants =
    std::max(info.currentTotalParticipants,
      state->counters.peakTotalParticipants.load(std::memory_order_relaxed));
  info.participantPressureObserved =
    state->counters.participantPressureObserved.load(std::memory_order_relaxed);
  return info;
}

}  // namespace

ParallelRuntime::ParallelRuntime(int resolvedMaxConcurrency, bool usingDefaultConcurrency):
  state_(std::make_shared<detail::RuntimeState>(resolvedMaxConcurrency, usingDefaultConcurrency)),
  defaultExecutor_(detail::makeExecutorState(state_, resolvedMaxConcurrency))
{
}

ParallelExecutor ParallelRuntime::createExecutor(ExecutorOptions options)
{
  validatePositive(options.maxConcurrency, "maxConcurrency");
#ifdef PGO_PARALLELISM_HAS_TBB
  const int resolved = options.maxConcurrency.value_or(state_->resolvedMaxConcurrency);
#else
  const int resolved = 1;
#endif
  return ParallelExecutor(detail::makeExecutorState(state_, resolved));
}

ParallelExecutor ParallelRuntime::defaultExecutor() const
{
  return defaultExecutor_;
}

RuntimeInfo ParallelRuntime::info() const
{
  return makeRuntimeInfo(state_.get());
}

ParallelRuntime &initializeRuntime(RuntimeOptions options)
{
  validatePositive(options.maxTbbConcurrency, "maxTbbConcurrency");
  std::lock_guard<std::mutex> lock(runtimeInitializationMutex());
  const int requested = resolveRuntimeConcurrency(options);

  if (ParallelRuntime *existing = publishedRuntime().load(std::memory_order_acquire)) {
    const int existingValue = existing->state_->resolvedMaxConcurrency;
    if (existingValue == requested)
      return *existing;

    std::ostringstream message;
    message << "ParallelRuntime is already initialized with max concurrency " << existingValue
            << "; requested " << requested << '.';
    throw std::logic_error(message.str());
  }

  // This intentionally retained object owns the sole pgo global_control for process lifetime.
  auto *created = new ParallelRuntime(requested, !options.maxTbbConcurrency.has_value());
  publishedRuntime().store(created, std::memory_order_release);
  return *created;
}

ParallelRuntime &runtime()
{
  if (ParallelRuntime *existing = publishedRuntime().load(std::memory_order_acquire))
    return *existing;
  return initializeRuntime();
}

int defaultConcurrency()
{
  return queriedDefaultConcurrency();
}

RuntimeInfo runtimeInfo()
{
  ParallelRuntime *existing = publishedRuntime().load(std::memory_order_acquire);
  return existing != nullptr ? existing->info() : makeRuntimeInfo(nullptr);
}

}  // namespace pgo::parallel
