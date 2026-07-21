#include "../benchmark_argument_parser.h"
#include "../eigen_blas/eigen_gemm_kernel.h"
#include "../eigen_mkl_common/eigen_mkl_executor_cases.h"
#include "../timed_workload.h"
#include "../workload_warmup.h"

#include "parallel/arenaThreadingExecutor.h"
#include "parallel/parallelControl.h"

#include <tbb/info.h>
#include <tbb/task_arena.h>

#include <charconv>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace
{

namespace P = pgo::parallel;
using pgo::benchmark_helpers::EigenGemmWorkspace;
using pgo::benchmark_helpers::MklExecutorCase;
using pgo::benchmark_helpers::mklExecutorCaseName;
using pgo::benchmark_helpers::mklExecutorSpec;
using pgo::benchmark_helpers::parseMklExecutorCase;
using pgo::benchmark_helpers::parseNonnegativeDouble;
using pgo::benchmark_helpers::parseNonnegativeInteger;
using pgo::benchmark_helpers::parsePositiveInteger;
using pgo::benchmark_helpers::parseTimedMeasurementArguments;
using pgo::benchmark_helpers::requireValue;
using pgo::benchmark_helpers::runTimedMeasurement;
using pgo::benchmark_helpers::runWorkloadWarmup;
using pgo::benchmark_helpers::TimedMeasurementArguments;

constexpr std::string_view concurrencyEnvironment =
  "PGO_EIGEN_MKL_CROSSOVER_MAX_CONCURRENCY";

struct Arguments
{
  MklExecutorCase policy;
  int matrixN;
  double warmupSeconds;
  int warmupMinOperations;
  TimedMeasurementArguments measurement;
};

std::optional<int> requestedConcurrency()
{
  const char *value = std::getenv(concurrencyEnvironment.data());
  if (!value || *value == '\0')
    return std::nullopt;
  int parsed = 0;
  const std::string_view text(value);
  const auto [end, error] =
    std::from_chars(text.data(), text.data() + text.size(), parsed);
  if (error != std::errc() || end != text.data() + text.size() || parsed <= 0) {
    throw std::invalid_argument(
      std::string(concurrencyEnvironment) + " must be a positive integer.");
  }
  return parsed;
}

Arguments parseArguments(int argc, char **argv)
{
  return {
    parseMklExecutorCase(requireValue(argc, argv, "--policy=")),
    parsePositiveInteger(requireValue(argc, argv, "--matrix-n="), "--matrix-n"),
    parseNonnegativeDouble(
      requireValue(argc, argv, "--warmup-seconds="), "--warmup-seconds"),
    parseNonnegativeInteger(requireValue(argc, argv, "--warmup-min-operations="),
      "--warmup-min-operations"),
    parseTimedMeasurementArguments(argc, argv),
  };
}

void run(const Arguments &arguments)
{
  const std::optional<int> requested = requestedConcurrency();
  const int configuredConcurrency =
    requested.value_or(tbb::info::default_concurrency());
  P::GlobalTbbControl control(configuredConcurrency);
  const int effectiveConcurrency = static_cast<int>(tbb::global_control::active_value(
    tbb::global_control::max_allowed_parallelism));
  const auto spec = mklExecutorSpec(arguments.policy, configuredConcurrency);
  P::ArenaThreadingExecutor executor(spec.arenaConcurrency,
    { .mklLocalThreadBudget = spec.mklLocalThreadBudget });
  EigenGemmWorkspace workspace(arguments.matrixN);
  int observedArenaConcurrency = 0;

  pgo::benchmark_helpers::WorkloadWarmupResult warmup{};
  pgo::benchmark_helpers::TimedMeasurementResult measurement{};
  executor.execute([&] {
    observedArenaConcurrency = tbb::this_task_arena::max_concurrency();
    warmup = runWorkloadWarmup([&] { workspace.run(); },
      arguments.warmupSeconds, arguments.warmupMinOperations);
    measurement = runTimedMeasurement(
      [&] { workspace.run(); }, arguments.measurement);
  });

  const double checksum = workspace.checksum();
  if (!std::isfinite(checksum))
    throw std::runtime_error("Eigen/MKL GEMM produced a non-finite checksum.");

  const double matrixN = static_cast<double>(arguments.matrixN);
  const double flopsPerOperation = 2.0 * matrixN * matrixN * matrixN;
  std::cout << std::setprecision(17)
            << "PGO_EIGEN_MKL_CROSSOVER_RESULT"
            << " policy=" << mklExecutorCaseName(arguments.policy)
            << " matrix_n=" << arguments.matrixN
            << " requested_concurrency=" << requested.value_or(0)
            << " effective_global_concurrency=" << effectiveConcurrency
            << " configured_arena_concurrency=" << spec.arenaConcurrency
            << " configured_mkl_local_budget=" << spec.mklLocalThreadBudget
            << " observed_arena_concurrency=" << observedArenaConcurrency
            << " configured_warmup_seconds=" << arguments.warmupSeconds
            << " configured_warmup_min_operations="
            << arguments.warmupMinOperations
            << " actual_warmup_seconds=" << warmup.elapsedSeconds
            << " actual_warmup_operations=" << warmup.completedOperations
            << " configured_measurement_min_seconds="
            << arguments.measurement.minimumSeconds
            << " measurement_operations=" << measurement.completedOperations
            << " measurement_wall_seconds=" << measurement.elapsedSeconds
            << " checksum=" << checksum
            << " flops_per_operation=" << flopsPerOperation << '\n';
}

}  // namespace

int main(int argc, char **argv)
{
  try {
    run(parseArguments(argc, argv));
    return 0;
  }
  catch (const std::exception &error) {
    std::cerr << "eigen_mkl_crossover_benchmark: " << error.what() << '\n';
    return 1;
  }
}
