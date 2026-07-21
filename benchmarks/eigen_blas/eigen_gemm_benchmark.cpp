#include "backend_scope.h"
#include "eigen_gemm_kernel.h"

#include "../benchmark_argument_parser.h"
#include "../parallelism_benchmark_helpers.h"
#include "../timed_workload.h"
#include "../workload_warmup.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace
{

using pgo::benchmark_helpers::adjustedExtraThreads;
using pgo::benchmark_helpers::availableEigenBlasBackends;
using pgo::benchmark_helpers::eigenBlasBackendName;
using pgo::benchmark_helpers::EigenBlasBackend;
using pgo::benchmark_helpers::EigenBlasBackendTelemetry;
using pgo::benchmark_helpers::EigenGemmWorkspace;
using pgo::benchmark_helpers::parseNonnegativeDouble;
using pgo::benchmark_helpers::parseNonnegativeInteger;
using pgo::benchmark_helpers::parsePositiveInteger;
using pgo::benchmark_helpers::parseTimedMeasurementArguments;
using pgo::benchmark_helpers::requireValue;
using pgo::benchmark_helpers::runInEigenBlasBackendScope;
using pgo::benchmark_helpers::runTimedMeasurement;
using pgo::benchmark_helpers::runWorkloadWarmup;
using pgo::benchmark_helpers::ThreadSampler;
using pgo::benchmark_helpers::TimedMeasurementArguments;

struct Arguments
{
  EigenBlasBackend backend;
  int matrixN;
  double warmupSeconds;
  int warmupMinOperations;
  TimedMeasurementArguments measurement;
};

EigenBlasBackend parseBackend(std::string_view value)
{
  for (const auto backend : availableEigenBlasBackends()) {
    if (value == eigenBlasBackendName(backend))
      return backend;
  }
  throw std::invalid_argument("Unsupported --backend value: " + std::string(value));
}

Arguments parseArguments(int argc, char **argv)
{
  return {
    parseBackend(requireValue(argc, argv, "--backend=")),
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
  EigenGemmWorkspace workspace(arguments.matrixN);
  EigenBlasBackendTelemetry telemetry;
  std::string scopeError;
  int baselineThreads = 0;
  int peakThreads = 0;
  pgo::benchmark_helpers::WorkloadWarmupResult warmup{};
  pgo::benchmark_helpers::TimedMeasurementResult measurement{};

  const bool succeeded = runInEigenBlasBackendScope(
    arguments.backend,
    [&] {
      ThreadSampler sampler;
      sampler.start();
      warmup = runWorkloadWarmup([&] { workspace.run(); },
        arguments.warmupSeconds, arguments.warmupMinOperations);
      peakThreads = sampler.stop();
      baselineThreads = sampler.baseline();
      measurement = runTimedMeasurement(
        [&] { workspace.run(); }, arguments.measurement);
    },
    telemetry,
    scopeError);
  if (!succeeded)
    throw std::runtime_error(scopeError);

  const double checksum = workspace.checksum();
  if (!std::isfinite(checksum))
    throw std::runtime_error("GEMM produced a non-finite checksum.");

  const double matrixN = static_cast<double>(arguments.matrixN);
  const double flopsPerOperation = 2.0 * matrixN * matrixN * matrixN;
  std::cout << std::setprecision(17)
            << "PGO_EIGEN_BLAS_RESULT"
            << " backend=" << eigenBlasBackendName(arguments.backend)
            << " matrix_n=" << arguments.matrixN
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
            << " baseline_threads=" << baselineThreads
            << " peak_threads=" << peakThreads
            << " extra_threads="
            << adjustedExtraThreads(baselineThreads, peakThreads)
            << " configured_concurrency=" << telemetry.configuredConcurrency
            << " arena_concurrency=" << telemetry.arenaConcurrency
            << " mkl_local_thread_budget=" << telemetry.mklLocalThreadBudget
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
    std::cerr << "eigen_gemm_benchmark: " << error.what() << '\n';
    return 1;
  }
}
