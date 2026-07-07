#include "core.h"

#include "parallelism/parallelOptions.h"

#include <optional>

namespace pgo
{

nanobind::object getWorkerLimit()
{
  const auto numWorkers = pgo::parallel::workerLimit();
  if (!numWorkers.has_value())
    return nanobind::none();
  return nanobind::int_(*numWorkers);
}

void resetWorkerLimit()
{
  pgo::parallel::setWorkerLimit(std::nullopt);
}

void setWorkerLimit(int numWorkers)
{
  if (numWorkers <= 0)
    throw nanobind::value_error("num_workers must be a positive integer or None");
  pgo::parallel::setWorkerLimit(numWorkers);
}

namespace
{

void putOptionalInt(nanobind::dict &dict, const char *key, std::optional<int> value)
{
  if (value.has_value())
    dict[key] = nanobind::int_(*value);
  else
    dict[key] = nanobind::none();
}

}  // namespace

nanobind::dict runtimeInfo()
{
  const pgo::parallel::RuntimeInfo info = pgo::parallel::runtimeInfo();
  nanobind::dict dict;
  putOptionalInt(dict, "worker_limit", info.workerLimit);
  if (pgo::parallel::supportsCpuAffinityLimit()) {
    putOptionalInt(dict, "cpu_affinity_limit", info.cpuAffinityLimit);
    putOptionalInt(dict, "current_cpu_affinity_cpus", info.currentCpuAffinityCpus);
  }
  putOptionalInt(dict, "tbb_max_allowed_parallelism", info.tbbMaxAllowedParallelism);
  return dict;
}

bool supportsCpuAffinityLimit()
{
  return pgo::parallel::supportsCpuAffinityLimit();
}

nanobind::object getCpuAffinityLimit()
{
  const auto numCpus = pgo::parallel::cpuAffinityLimit();
  if (!numCpus.has_value())
    return nanobind::none();
  return nanobind::int_(*numCpus);
}

void resetCpuAffinityLimit()
{
  pgo::parallel::setCpuAffinityLimit(std::nullopt);
}

void setCpuAffinityLimit(int numCpus)
{
  if (numCpus <= 0)
    throw nanobind::value_error("num_cpus must be a positive integer or None");
  pgo::parallel::setCpuAffinityLimit(numCpus);
}

}  // namespace pgo
