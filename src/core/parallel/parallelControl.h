#pragma once

#include <algorithm>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

#include <tbb/task_arena.h>

namespace pgo::parallel
{

// Sets the pgo-owned process-wide TBB ceiling and returns the currently effective ceiling. Passing
// std::nullopt restores the oneTBB default concurrency value. Calls are safe to repeat, but callers
// should reconfigure at application quiescence boundaries because active work is not preempted.
int setMaxConcurrency(std::optional<int> maxConcurrency = std::nullopt);

namespace detail
{

int activeTbbConcurrency() noexcept;
std::shared_ptr<tbb::task_arena> globalAlignedArena();
std::shared_ptr<tbb::task_arena> localBoundedArena(int maxConcurrency);

inline int currentArenaConcurrency() noexcept
{
  return std::max(1, tbb::this_task_arena::max_concurrency());
}

}  // namespace detail

template<class Fn>
decltype(auto) withGlobalTbbConcurrency(Fn &&fn)
{
  const int active = detail::activeTbbConcurrency();
  if (detail::currentArenaConcurrency() <= active)
    return fn();
  return detail::globalAlignedArena()->execute(fn);
}

template<class Fn>
decltype(auto) withTbbConcurrencyLimit(int maxConcurrency, Fn &&fn)
{
  if (maxConcurrency <= 0)
    throw std::invalid_argument("TBB concurrency limit must be positive.");

  const int current = detail::currentArenaConcurrency();
  const int active = detail::activeTbbConcurrency();
  const int resolved = std::min({ maxConcurrency, active, current });
  if (resolved >= current)
    return fn();
  if (resolved == active)
    return withGlobalTbbConcurrency(fn);
  return detail::localBoundedArena(resolved)->execute(fn);
}

template<class Fn>
decltype(auto) withSingleThreadedTbb(Fn &&fn)
{
  return withTbbConcurrencyLimit(1, std::forward<Fn>(fn));
}

}  // namespace pgo::parallel
