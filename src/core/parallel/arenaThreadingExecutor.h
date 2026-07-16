#pragma once

#include "parallel/threadingPolicy.h"

#include <functional>
#include <memory>
#include <utility>

#include <tbb/task_arena.h>

namespace pgo::parallel
{

// Owns a private oneTBB arena and persistently overwrites the active BLAS backend TLS whenever a
// thread enters that arena. Leaving the arena does not restore the thread's previous state.
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
    return arena_.execute(std::forward<Fn>(fn));
  }

  int maxConcurrency() const noexcept;
  const ThreadingPolicy &threadingPolicy() const noexcept;

private:
  class Observer;

  tbb::task_arena arena_;
  const ThreadingPolicy policy_;
  std::unique_ptr<Observer> observer_;
};

}  // namespace pgo::parallel
