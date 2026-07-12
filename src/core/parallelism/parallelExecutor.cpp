#include "parallelism/parallelExecutor.h"

#include "parallelInternal.h"

#include <atomic>
#include <memory>
#include <utility>

#ifdef PGO_PARALLELISM_HAS_TBB
#  include <tbb/task_arena.h>
#  include <tbb/task_scheduler_observer.h>
#endif

namespace pgo::parallel
{
namespace detail
{

#ifdef PGO_PARALLELISM_HAS_TBB
namespace
{

void updatePeak(std::atomic<int> &peak, int value) noexcept
{
  int observed = peak.load(std::memory_order_relaxed);
  while (observed < value &&
    !peak.compare_exchange_weak(observed, value, std::memory_order_relaxed, std::memory_order_relaxed)) {
  }
}

class ArenaObserver final : public tbb::task_scheduler_observer
{
public:
  ArenaObserver(tbb::task_arena &arena, RuntimeState &runtime):
    tbb::task_scheduler_observer(arena), runtime_(runtime)
  {
    observe(true);
  }

  ~ArenaObserver() override
  {
    observe(false);

    const int workers = localWorkerParticipants_.exchange(0, std::memory_order_relaxed);
    const int external = localExternalParticipants_.exchange(0, std::memory_order_relaxed);
    runtime_.counters.currentParticipants.fetch_sub(packParticipants(workers, external));
  }

  void on_scheduler_entry(bool isWorker) noexcept override
  {
    if (isWorker)
      localWorkerParticipants_.fetch_add(1, std::memory_order_relaxed);
    else
      localExternalParticipants_.fetch_add(1, std::memory_order_relaxed);

    const std::uint64_t participant = isWorker ? kWorkerParticipant : kExternalParticipant;
    const std::uint64_t previous =
      runtime_.counters.currentParticipants.fetch_add(participant, std::memory_order_relaxed);
    const int total = totalParticipants(previous) + 1;
    updatePeak(runtime_.counters.peakTotalParticipants, total);
    if (total > runtime_.participantPressureThreshold.load(std::memory_order_relaxed))
      runtime_.counters.participantPressureObserved.store(true, std::memory_order_relaxed);
  }

  void on_scheduler_exit(bool isWorker) noexcept override
  {
    if (isWorker)
      localWorkerParticipants_.fetch_sub(1, std::memory_order_relaxed);
    else
      localExternalParticipants_.fetch_sub(1, std::memory_order_relaxed);

    runtime_.counters.currentParticipants.fetch_sub(
      isWorker ? kWorkerParticipant : kExternalParticipant, std::memory_order_relaxed);
  }

private:
  RuntimeState &runtime_;
  std::atomic<int> localWorkerParticipants_{ 0 };
  std::atomic<int> localExternalParticipants_{ 0 };
};

}  // namespace
#endif

struct ExecutorState
{
  ExecutorState(std::shared_ptr<RuntimeState> runtime_, int maxConcurrency_):
    runtime(std::move(runtime_)),
    maxConcurrency(maxConcurrency_)
#ifdef PGO_PARALLELISM_HAS_TBB
    ,
    arena(maxConcurrency_, 1),
    observer(arena, *runtime)
#endif
  {
  }

  std::shared_ptr<RuntimeState> runtime;
  const int maxConcurrency;
#ifdef PGO_PARALLELISM_HAS_TBB
  tbb::task_arena arena;
  ArenaObserver observer;
#endif
};

const std::shared_ptr<ExecutorState> &executorState(const ParallelExecutor &executor)
{
  return executor.state_;
}

std::shared_ptr<ExecutorState> makeExecutorState(
  std::shared_ptr<RuntimeState> runtime, int maxConcurrency)
{
  return std::make_shared<ExecutorState>(std::move(runtime), maxConcurrency);
}

void executeInArena(const std::shared_ptr<ExecutorState> &executor, const ExecutorBody &body)
{
#ifdef PGO_PARALLELISM_HAS_TBB
  executor->runtime->participantPressureThreshold.store(
    effectiveTbbMaxAllowedParallelism(), std::memory_order_relaxed);
  executor->arena.execute([&] {
    body(executor.get());
  });
#else
  body(executor.get());
#endif
}

}  // namespace detail

ParallelExecutor::ParallelExecutor(std::shared_ptr<detail::ExecutorState> state): state_(std::move(state))
{
}

std::optional<int> ParallelExecutor::maxConcurrency() const noexcept
{
  if (!state_)
    return std::nullopt;
  return state_->maxConcurrency;
}

}  // namespace pgo::parallel
