#include "core.h"

#include "parallelism/parallelOptions.h"

#include <optional>

namespace pgo
{

nanobind::object getNumThreads()
{
  const auto numThreads = pgo::parallel::threadLimit();
  if (!numThreads.has_value())
    return nanobind::none();
  return nanobind::int_(*numThreads);
}

void resetNumThreads()
{
  pgo::parallel::setThreadLimit(std::nullopt);
}

void setNumThreads(int numThreads)
{
  if (numThreads <= 0)
    throw nanobind::value_error("num_threads must be a positive integer or None");
  pgo::parallel::setThreadLimit(numThreads);
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
  putOptionalInt(dict, "thread_limit", info.threadLimit);
  putOptionalInt(dict, "cpu_affinity_limit", info.cpuAffinityLimit);
  putOptionalInt(dict, "current_cpu_affinity_cpus", info.currentCpuAffinityCpus);
  putOptionalInt(dict, "tbb_max_allowed_parallelism", info.tbbMaxAllowedParallelism);
  putOptionalInt(dict, "eigen_num_threads", info.eigenNumThreads);
  putOptionalInt(dict, "openmp_max_threads", info.openMPMaxThreads);
  putOptionalInt(dict, "mkl_max_threads", info.mklMaxThreads);
  putOptionalInt(dict, "mkl_effective_thread_limit", info.mklEffectiveThreadLimit);
  putOptionalInt(dict, "mkl_pardiso_max_threads", info.mklPardisoMaxThreads);
  putOptionalInt(dict, "openblas_num_threads", info.openBLASNumThreads);
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
