#pragma once

#include <functional>
#include <span>
#include <string>

namespace pgo::benchmark_helpers
{

enum class EigenBlasBackend
{
  EigenInternal = 0,
  AccelerateDefaultSingle = 1,
  AccelerateExperimentalMulti = 2,
  MklTbbSingle = 3,
  MklTbbGlobal = 4,
};

struct EigenBlasBackendTelemetry
{
  int configuredConcurrency = 0;
  int arenaConcurrency = 0;
  int vendorMaxThreads = 0;
};

std::span<const EigenBlasBackend> availableEigenBlasBackends() noexcept;
const char *eigenBlasBackendName(EigenBlasBackend backend) noexcept;

bool runInEigenBlasBackendScope(
  EigenBlasBackend backend,
  const std::function<void()> &fn,
  EigenBlasBackendTelemetry &telemetry,
  std::string &error);

}  // namespace pgo::benchmark_helpers
