#include "parallel/parallelControl.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <mutex>
#include <unordered_map>

#include <tbb/global_control.h>
#include <tbb/info.h>

namespace pgo::parallel
{
namespace
{

int clampConcurrency(std::size_t value) noexcept
{
  return static_cast<int>(std::min(value, static_cast<std::size_t>(std::numeric_limits<int>::max())));
}

int activeConcurrency() noexcept
{
  return std::max(1, clampConcurrency(tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism)));
}

class GlobalControlState
{
public:
  int set(std::optional<int> maxConcurrency)
  {
    const int requested = requestedOrDefault(maxConcurrency);
    auto replacement = std::make_unique<tbb::global_control>(
      tbb::global_control::max_allowed_parallelism, static_cast<std::size_t>(requested));

    std::lock_guard<std::mutex> lock(mutex_);
    control_ = std::move(replacement);
    return activeConcurrency();
  }

  std::shared_ptr<tbb::task_arena> arena()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const int active = activeConcurrency();
    if (!arena_ || arenaConcurrency_ != active) {
      arena_ = std::make_shared<tbb::task_arena>(active, 1);
      arenaConcurrency_ = active;
    }
    return arena_;
  }

private:
  static int requestedOrDefault(std::optional<int> value)
  {
    if (value.has_value() && *value <= 0)
      throw std::invalid_argument("maxConcurrency must be positive or unset.");
    return value.value_or(std::max(1, tbb::info::default_concurrency()));
  }

  std::mutex mutex_;
  std::unique_ptr<tbb::global_control> control_;
  std::shared_ptr<tbb::task_arena> arena_;
  int arenaConcurrency_ = 0;
};

GlobalControlState &globalControlState()
{
  static GlobalControlState state;
  return state;
}

}  // namespace

int setMaxConcurrency(std::optional<int> maxConcurrency)
{
  return globalControlState().set(maxConcurrency);
}

namespace detail
{

int activeTbbConcurrency() noexcept
{
  return activeConcurrency();
}

std::shared_ptr<tbb::task_arena> globalAlignedArena()
{
  return globalControlState().arena();
}

std::shared_ptr<tbb::task_arena> localBoundedArena(int maxConcurrency)
{
  thread_local std::unordered_map<int, std::shared_ptr<tbb::task_arena>> arenas;
  auto [it, inserted] = arenas.try_emplace(maxConcurrency);
  if (inserted)
    it->second = std::make_shared<tbb::task_arena>(maxConcurrency, 1);
  return it->second;
}

}  // namespace detail
}  // namespace pgo::parallel
