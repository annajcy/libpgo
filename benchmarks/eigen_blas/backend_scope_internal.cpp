#include "backend_scope.h"

#include <array>
#include <exception>

namespace pgo::benchmark_helpers
{

std::span<const EigenBlasBackend> availableEigenBlasBackends() noexcept
{
  static constexpr std::array backends = { EigenBlasBackend::EigenInternal };
  return backends;
}

const char *eigenBlasBackendName(EigenBlasBackend backend) noexcept
{
  return backend == EigenBlasBackend::EigenInternal ? "EigenInternal" : "Unknown";
}

bool runInEigenBlasBackendScope(
  EigenBlasBackend backend,
  const std::function<void()> &fn,
  EigenBlasBackendTelemetry &telemetry,
  std::string &error)
{
  if (backend != EigenBlasBackend::EigenInternal) {
    error = "The Eigen-internal executable received a vendor BLAS backend.";
    return false;
  }

  telemetry = {};
  try {
    fn();
    return true;
  }
  catch (const std::exception &exception) {
    error = exception.what();
    return false;
  }
}

}  // namespace pgo::benchmark_helpers
