#include "core.h"

#include "parallel/parallelControl.h"

namespace pgo
{
int parallelInitialize(std::optional<int> maxConcurrency)
{
  return parallel::setMaxConcurrency(maxConcurrency);
}

}  // namespace pgo
