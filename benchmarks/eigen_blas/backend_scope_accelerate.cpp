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
    EigenBlasBackend::AccelerateDefaultSingle,
    EigenBlasBackend::AccelerateExperimentalMulti,
  };
  return backends;
}

const char *eigenBlasBackendName(EigenBlasBackend backend) noexcept
{
  switch (backend) {
  case EigenBlasBackend::AccelerateDefaultSingle:
    return "AccelerateDefaultSingle";
  case EigenBlasBackend::AccelerateExperimentalMulti:
    return "AccelerateExperimentalMulti";
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
  if (backend != EigenBlasBackend::AccelerateDefaultSingle &&
    backend != EigenBlasBackend::AccelerateExperimentalMulti) {
    error = "The Accelerate executable received a non-Accelerate backend.";
    return false;
  }

  telemetry = {};

  try {
    telemetry.configuredConcurrency = pgo::parallel::initialize();
    if (backend == EigenBlasBackend::AccelerateDefaultSingle) {
      if (BLASGetThreading() != BLAS_THREADING_SINGLE_THREADED)
        throw std::runtime_error("initialize() did not establish Accelerate SINGLE mode.");
      fn();
    }
    else {
      pgo::parallel::experimental::withMultiThreadedAccelerate([&] {
        if (BLASGetThreading() != BLAS_THREADING_MULTI_THREADED)
          throw std::runtime_error("The experimental scope did not establish Accelerate MULTI mode.");
        fn();
      });
    }

    if (BLASGetThreading() != BLAS_THREADING_SINGLE_THREADED) {
      error = "The Accelerate benchmark did not restore the initialized SINGLE mode.";
      return false;
    }
    return true;
  }
  catch (const std::exception &exception) {
    error = exception.what();
    return false;
  }
}

}  // namespace pgo::benchmark_helpers
