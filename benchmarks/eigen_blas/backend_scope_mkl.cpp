#include "backend_scope.h"

#include "parallel/parallelControl.h"

#include <mkl.h>
#include <tbb/info.h>
#include <tbb/task_arena.h>

#include <array>
#include <charconv>
#include <cstdlib>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string_view>

namespace pgo::benchmark_helpers
{
namespace
{

constexpr std::string_view concurrencyEnvironment = "PGO_EIGEN_BLAS_MAX_CONCURRENCY";

std::optional<int> requestedConcurrency()
{
  const char *value = std::getenv(concurrencyEnvironment.data());
  if (!value || *value == '\0')
    return std::nullopt;

  int parsed = 0;
  const std::string_view text(value);
  const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), parsed);
  if (error != std::errc() || end != text.data() + text.size() || parsed <= 0) {
    throw std::invalid_argument(
      std::string(concurrencyEnvironment) + " must be a positive integer.");
  }
  return parsed;
}

void observeMkl(EigenBlasBackendTelemetry &telemetry)
{
  telemetry.arenaConcurrency = tbb::this_task_arena::max_concurrency();
}

}  // namespace

std::span<const EigenBlasBackend> availableEigenBlasBackends() noexcept
{
  static constexpr std::array backends = {
    EigenBlasBackend::MklTbbSingle,
    EigenBlasBackend::MklTbbGlobal,
  };
  return backends;
}

const char *eigenBlasBackendName(EigenBlasBackend backend) noexcept
{
  switch (backend) {
  case EigenBlasBackend::MklTbbSingle:
    return "MklTbbSingle";
  case EigenBlasBackend::MklTbbGlobal:
    return "MklTbbGlobal";
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
  try {
    telemetry = {};
    const int configuredConcurrency = requestedConcurrency().value_or(
      tbb::info::default_concurrency());
    pgo::parallel::GlobalTbbControl control(configuredConcurrency);
    telemetry.configuredConcurrency = static_cast<int>(tbb::global_control::active_value(
      tbb::global_control::max_allowed_parallelism));
    tbb::task_arena arena(configuredConcurrency, 1);
    tbb::task_arena singleArena(1, 1);

    const auto invoke = [&] {
      observeMkl(telemetry);
      fn();
    };

    switch (backend) {
    case EigenBlasBackend::MklTbbSingle:
      singleArena.execute(invoke);
      return true;
    case EigenBlasBackend::MklTbbGlobal:
      arena.execute(invoke);
      return true;
    default:
      error = "The MKL executable received a non-MKL backend.";
      return false;
    }
  }
  catch (const std::exception &exception) {
    error = exception.what();
    return false;
  }
}

}  // namespace pgo::benchmark_helpers
