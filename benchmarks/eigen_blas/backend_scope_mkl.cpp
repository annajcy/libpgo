#include "backend_scope.h"

#include "../eigen_mkl_common/eigen_mkl_executor_cases.h"
#include "parallel/arenaThreadingExecutor.h"
#include "parallel/parallelControl.h"

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

MklExecutorCase executorCase(EigenBlasBackend backend)
{
  switch (backend) {
  case EigenBlasBackend::MklLocal1:
    return MklExecutorCase::ExecutorLocal1;
  case EigenBlasBackend::MklC:
    return MklExecutorCase::ExecutorMKLC;
  default:
    throw std::invalid_argument("The MKL executable received a non-MKL backend.");
  }
}

}  // namespace

std::span<const EigenBlasBackend> availableEigenBlasBackends() noexcept
{
  static constexpr std::array backends = {
    EigenBlasBackend::MklLocal1,
    EigenBlasBackend::MklC,
  };
  return backends;
}

const char *eigenBlasBackendName(EigenBlasBackend backend) noexcept
{
  switch (backend) {
  case EigenBlasBackend::MklLocal1:
    return "MklLocal1";
  case EigenBlasBackend::MklC:
    return "MklC";
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
    const auto spec = mklExecutorSpec(executorCase(backend), configuredConcurrency);
    telemetry.mklLocalThreadBudget = spec.mklLocalThreadBudget;
    pgo::parallel::ArenaThreadingExecutor executor(spec.arenaConcurrency,
      { .mklLocalThreadBudget = spec.mklLocalThreadBudget });

    executor.execute([&] {
      telemetry.arenaConcurrency = tbb::this_task_arena::max_concurrency();
      fn();
    });
    return true;
  }
  catch (const std::exception &exception) {
    error = exception.what();
    return false;
  }
}

}  // namespace pgo::benchmark_helpers
