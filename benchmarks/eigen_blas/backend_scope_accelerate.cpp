#include "backend_scope.h"

#include "parallel/parallel.h"

#include <Accelerate/Accelerate.h>

#include <array>
#include <exception>
#include <stdexcept>

namespace pgo::benchmark_helpers
{
std::span<const EigenBlasBackend> availableEigenBlasBackends() noexcept
{
  static constexpr std::array backends = {
    EigenBlasBackend::AccelerateSingle,
    EigenBlasBackend::AccelerateMulti,
  };
  return backends;
}

const char *eigenBlasBackendName(EigenBlasBackend backend) noexcept
{
  switch (backend) {
  case EigenBlasBackend::AccelerateSingle:
    return "AccelerateSingle";
  case EigenBlasBackend::AccelerateMulti:
    return "AccelerateMulti";
  default:
    return "Unknown";
  }
}

bool runInEigenBlasBackendScope(
  EigenBlasBackend backend,
  const std::function<void()> &fn,
  EigenBlasBackendTelemetry &telemetry,
  std::string &error)
{
  if (backend != EigenBlasBackend::AccelerateSingle &&
    backend != EigenBlasBackend::AccelerateMulti) {
    error = "The Accelerate executable received a non-Accelerate backend.";
    return false;
  }

  telemetry = {};

  try {
    telemetry.configuredConcurrency = static_cast<int>(tbb::global_control::active_value(
      tbb::global_control::max_allowed_parallelism));
    if (backend == EigenBlasBackend::AccelerateSingle) {
      pgo::parallel::setThreadingPolicy({
        .accelerate = pgo::parallel::AccelerateThreading::single,
      });
      if (BLASGetThreading() != BLAS_THREADING_SINGLE_THREADED)
        throw std::runtime_error("The threading policy did not establish Accelerate SINGLE mode.");
      fn();
    }
    else {
      pgo::parallel::setThreadingPolicy({
        .accelerate = pgo::parallel::AccelerateThreading::multi,
      });
      if (BLASGetThreading() != BLAS_THREADING_MULTI_THREADED)
        throw std::runtime_error("The threading policy did not establish Accelerate MULTI mode.");
      fn();
    }
    return true;
  }
  catch (const std::exception &exception) {
    error = exception.what();
    return false;
  }
}

}  // namespace pgo::benchmark_helpers
