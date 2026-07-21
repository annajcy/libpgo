#include "parallel/parallelControl.h"

#include <stdexcept>

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

}  // namespace pgo::parallel
