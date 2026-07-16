#pragma once

#include "parallel/threadingPolicy.h"

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <utility>

#include <tbb/task_arena.h>

namespace pgo::parallel
{

namespace detail
{
class ArenaThreadingExecutorRetirementRegistry;
}

// Owns a private oneTBB arena and scopes the active BLAS backend TLS to each thread's participation
// in that arena. The handle may execute repeatedly. Its destructor retires the arena without waiting
// for lingering oneTBB participants; the arena and observer remain alive until oneTBB detaches the
// observer after the internal arena becomes empty. As with ordinary C++ object lifetime, callers
// must not invoke execute() concurrently with destruction of the handle.
class ArenaThreadingExecutor final
{
public:
  explicit ArenaThreadingExecutor(int maxConcurrency, ThreadingPolicy policy,
    int reservedSlots = 1, tbb::task_arena::priority priority = tbb::task_arena::priority::normal);
  ~ArenaThreadingExecutor() noexcept;

  ArenaThreadingExecutor(const ArenaThreadingExecutor &) = delete;
  ArenaThreadingExecutor &operator=(const ArenaThreadingExecutor &) = delete;
  ArenaThreadingExecutor(ArenaThreadingExecutor &&) = delete;
  ArenaThreadingExecutor &operator=(ArenaThreadingExecutor &&) = delete;

  template<class Fn>
  decltype(auto) execute(Fn &&fn)
  {
    return state_->arena_.execute(std::forward<Fn>(fn));
  }

  int maxConcurrency() const noexcept;
  const ThreadingPolicy &threadingPolicy() const noexcept;

private:
  friend class detail::ArenaThreadingExecutorRetirementRegistry;

  class Observer;
  class State final
  {
    friend class ArenaThreadingExecutor;
    friend class detail::ArenaThreadingExecutorRetirementRegistry;

  public:
    State(int maxConcurrency, ThreadingPolicy policy, int reservedSlots,
      tbb::task_arena::priority priority);
    ~State() noexcept;

    State(const State &) = delete;
    State &operator=(const State &) = delete;
    State(State &&) = delete;
    State &operator=(State &&) = delete;

  private:
    tbb::task_arena arena_;
    const ThreadingPolicy policy_;
    std::unique_ptr<Observer> observer_;

    bool terminationFinished_ = false;
    std::unique_ptr<State> nextRetired_;
  };

  std::unique_ptr<State> state_;
};

// Reclaims retired executor states whose observers have been detached by oneTBB. Retirement also
// performs an opportunistic collection pass. These functions never force observation off while an
// internal arena may still have participants. drain...() may block its caller for up to timeout; the
// executor destructor itself never performs that wait.
std::size_t collectRetiredArenaThreadingExecutorStates() noexcept;
std::size_t retiredArenaThreadingExecutorStateCount() noexcept;
bool drainRetiredArenaThreadingExecutorStates(std::chrono::milliseconds timeout) noexcept;

}  // namespace pgo::parallel
