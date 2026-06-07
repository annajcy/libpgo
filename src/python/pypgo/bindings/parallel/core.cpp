#include "core.h"

#include "parallelism/parallelOptions.h"

#include <optional>

namespace pgo
{

nanobind::object getNumThreads()
{
  const auto numThreads = pgo::parallel::defaultNumThreads();
  if (!numThreads.has_value())
    return nanobind::none();
  return nanobind::int_(*numThreads);
}

void resetNumThreads()
{
  pgo::parallel::setDefaultNumThreads(std::nullopt);
}

void setNumThreads(int numThreads)
{
  if (numThreads <= 0)
    throw nanobind::value_error("num_threads must be a positive integer or None");
  pgo::parallel::setDefaultNumThreads(numThreads);
}

}  // namespace pgo
