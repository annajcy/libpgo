#include "parallel/arenaThreadingExecutor.h"

#include <chrono>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include <tbb/task_scheduler_observer.h>

namespace pgo::parallel
{
namespace
{

int validateMaxConcurrency(int value)
{
  if (value <= 0)
    throw std::invalid_argument("Arena max concurrency must be positive.");
  return value;
}

int validateReservedSlots(int reservedSlots, int maxConcurrency)
{
  if (reservedSlots < 0 || reservedSlots > maxConcurrency)
    throw std::invalid_argument("Arena reserved slots must be between zero and max concurrency.");
  return reservedSlots;
}

ThreadingPolicy validatePolicy(ThreadingPolicy policy)
{
  detail::validateCompleteThreadingPolicy(policy);
  return policy;
}

struct ThreadingPolicyFrame
{
  const void *owner = nullptr;
  detail::ThreadingPolicySnapshot previous;
};

std::vector<ThreadingPolicyFrame> &threadingPolicyStack()
{
  thread_local std::vector<ThreadingPolicyFrame> stack = [] {
    std::vector<ThreadingPolicyFrame> value;
    value.reserve(4);
    return value;
  }();
  return stack;
}

}  // namespace

class ArenaThreadingExecutor::Observer final : public tbb::task_scheduler_observer
{
public:
  Observer(tbb::task_arena &arena, ThreadingPolicy policy): tbb::task_scheduler_observer(arena), policy_(std::move(policy))
  {
    observe(true);
  }

  ~Observer() override
  {
    observe(false);
  }

private:
  void on_scheduler_entry(bool) noexcept override
  {
    const detail::ThreadingPolicySnapshot previous = detail::exchangeThreadingPolicyNoexcept(policy_);
    try {
      threadingPolicyStack().push_back({ .owner = this, .previous = previous });
    }
    catch (...) {
      detail::restoreThreadingPolicyNoexcept(previous);
      std::terminate();
    }
  }

  void on_scheduler_exit(bool) noexcept override
  {
    std::vector<ThreadingPolicyFrame> &stack = threadingPolicyStack();
    if (stack.empty() || stack.back().owner != this)
      std::terminate();

    const detail::ThreadingPolicySnapshot previous = stack.back().previous;
    detail::restoreThreadingPolicyNoexcept(previous);
    stack.pop_back();
  }

  const ThreadingPolicy policy_;
};

ArenaThreadingExecutor::State::State(int maxConcurrency, ThreadingPolicy policy,
  int reservedSlots, tbb::task_arena::priority priority): arena_(validateMaxConcurrency(maxConcurrency),
                                                            validateReservedSlots(reservedSlots, maxConcurrency), priority),
                                                          policy_(validatePolicy(std::move(policy))),
                                                          observer_(std::make_unique<Observer>(arena_, policy_))
{
}

ArenaThreadingExecutor::State::~State() noexcept
{
  if (observer_ && observer_->is_observing())
    std::terminate();
}

namespace detail
{

class ArenaThreadingExecutorRetirementRegistry
{
public:
  static ArenaThreadingExecutorRetirementRegistry &instance()
  {
    static auto *registry = new ArenaThreadingExecutorRetirementRegistry;
    return *registry;
  }

  void retire(std::unique_ptr<ArenaThreadingExecutor::State> state) noexcept
  {
    if (!state)
      return;

    std::unique_ptr<ArenaThreadingExecutor::State> reclaimed;
    {
      std::lock_guard lock(mutex_);

      ArenaThreadingExecutor::State *retiring = state.get();
      state->nextRetired_ = std::move(retired_);
      retired_ = std::move(state);
      ++retiredCount_;

      retiring->arena_.terminate();
      retiring->terminationFinished_ = true;
      collectLocked(reclaimed);
    }
    destroyList(std::move(reclaimed));
  }

  std::size_t collect() noexcept
  {
    std::unique_ptr<ArenaThreadingExecutor::State> reclaimed;
    std::size_t reclaimedCount = 0;
    {
      std::lock_guard lock(mutex_);
      const std::size_t previousCount = retiredCount_;
      collectLocked(reclaimed);
      reclaimedCount = previousCount - retiredCount_;
    }
    destroyList(std::move(reclaimed));
    return reclaimedCount;
  }

  std::size_t count() noexcept
  {
    std::lock_guard lock(mutex_);
    return retiredCount_;
  }

  bool drain(std::chrono::milliseconds timeout) noexcept
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
      collect();
      if (count() == 0)
        return true;
      if (std::chrono::steady_clock::now() >= deadline)
        return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (true);
  }

private:
  void collectLocked(std::unique_ptr<ArenaThreadingExecutor::State> &reclaimed) noexcept
  {
    std::unique_ptr<ArenaThreadingExecutor::State> *link = &retired_;
    while (*link) {
      ArenaThreadingExecutor::State *state = link->get();
      if (state->terminationFinished_ && !state->observer_->is_observing()) {
        std::unique_ptr<ArenaThreadingExecutor::State> ready = std::move(*link);
        *link = std::move(ready->nextRetired_);
        ready->nextRetired_ = std::move(reclaimed);
        reclaimed = std::move(ready);
        --retiredCount_;
      }
      else {
        link = &state->nextRetired_;
      }
    }
  }

  static void destroyList(std::unique_ptr<ArenaThreadingExecutor::State> list) noexcept
  {
    while (list) {
      std::unique_ptr<ArenaThreadingExecutor::State> next = std::move(list->nextRetired_);
      list.reset();
      list = std::move(next);
    }
  }

  std::mutex mutex_;
  std::unique_ptr<ArenaThreadingExecutor::State> retired_;
  std::size_t retiredCount_ = 0;
};

}  // namespace detail

ArenaThreadingExecutor::ArenaThreadingExecutor(int maxConcurrency, ThreadingPolicy policy,
  int reservedSlots, tbb::task_arena::priority priority): state_(std::make_unique<State>(maxConcurrency, std::move(policy), reservedSlots, priority))
{
}

ArenaThreadingExecutor::~ArenaThreadingExecutor() noexcept
{
  detail::ArenaThreadingExecutorRetirementRegistry::instance().retire(std::move(state_));
}

int ArenaThreadingExecutor::maxConcurrency() const noexcept
{
  return state_->arena_.max_concurrency();
}

const ThreadingPolicy &ArenaThreadingExecutor::threadingPolicy() const noexcept
{
  return state_->policy_;
}

std::size_t collectRetiredArenaThreadingExecutorStates() noexcept
{
  return detail::ArenaThreadingExecutorRetirementRegistry::instance().collect();
}

std::size_t retiredArenaThreadingExecutorStateCount() noexcept
{
  return detail::ArenaThreadingExecutorRetirementRegistry::instance().count();
}

bool drainRetiredArenaThreadingExecutorStates(std::chrono::milliseconds timeout) noexcept
{
  return detail::ArenaThreadingExecutorRetirementRegistry::instance().drain(timeout);
}

}  // namespace pgo::parallel
