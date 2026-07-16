#include "parallel/parallelControl.h"

#include <algorithm>
#include <stdexcept>

#include <tbb/task_arena.h>

namespace pgo::parallel
{
namespace
{

std::size_t validateGlobalConcurrency(int value)
{
  if (value <= 0)
    throw std::invalid_argument("Global TBB concurrency must be positive.");
  return static_cast<std::size_t>(value);
}

}  // namespace

GlobalTbbControl::GlobalTbbControl(int maxAllowedParallelism): control_(tbb::global_control::max_allowed_parallelism,
                                                                 validateGlobalConcurrency(maxAllowedParallelism))
{
}

int resolveNonExpandingTbbConcurrency(int requestedConcurrency)
{
  if (requestedConcurrency <= 0)
    throw std::invalid_argument("Requested TBB concurrency must be positive.");
  return std::min(requestedConcurrency,
    tbb::this_task_arena::max_concurrency());
}

}  // namespace pgo::parallel
