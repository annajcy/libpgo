#include "backend_scope.h"
#include "eigen_gemm_kernel.h"

#include "../parallelism_benchmark_helpers.h"

#include <benchmark/benchmark.h>

#include <cmath>
#include <string>

namespace
{

using pgo::benchmark_helpers::adjustedExtraThreads;
using pgo::benchmark_helpers::availableEigenBlasBackends;
using pgo::benchmark_helpers::eigenBlasBackendName;
using pgo::benchmark_helpers::EigenBlasBackend;
using pgo::benchmark_helpers::EigenBlasBackendTelemetry;
using pgo::benchmark_helpers::EigenGemmWorkspace;
using pgo::benchmark_helpers::runInEigenBlasBackendScope;
using pgo::benchmark_helpers::ThreadSampler;

void runGemmBenchmark(benchmark::State &state, EigenBlasBackend backend, int matrixN)
{
  EigenGemmWorkspace workspace(matrixN);
  EigenBlasBackendTelemetry telemetry;
  std::string scopeError;
  int baselineThreads = 0;
  int peakThreads = 0;

  const bool succeeded = runInEigenBlasBackendScope(
    backend,
    [&] {
      // The untimed sample primes caches and any vendor worker pool without perturbing the timed
      // iterations. The backend scope deliberately covers both warm-up and measurement.
      ThreadSampler sampler;
      sampler.start();
      workspace.run();
      peakThreads = sampler.stop();
      baselineThreads = sampler.baseline();

      for (auto _ : state) {
        workspace.run();
        benchmark::DoNotOptimize(&workspace);
        benchmark::ClobberMemory();
      }
    },
    telemetry,
    scopeError);

  if (!succeeded) {
    state.SkipWithError(scopeError.c_str());
    return;
  }

  const double checksum = workspace.checksum();
  if (!std::isfinite(checksum)) {
    state.SkipWithError("GEMM produced a non-finite checksum.");
    return;
  }

  const double flopsPerIteration =
    2.0 * static_cast<double>(matrixN) * static_cast<double>(matrixN) *
    static_cast<double>(matrixN);
  state.counters["backend"] = static_cast<int>(backend);
  state.counters["matrix_n"] = matrixN;
  state.counters["checksum"] = checksum;
  state.counters["baseline_threads"] = baselineThreads;
  state.counters["peak_threads"] = peakThreads;
  state.counters["extra_threads"] = adjustedExtraThreads(baselineThreads, peakThreads);
  state.counters["configured_concurrency"] = telemetry.configuredConcurrency;
  state.counters["arena_concurrency"] = telemetry.arenaConcurrency;
  state.counters["vendor_max_threads"] = telemetry.vendorMaxThreads;
  state.counters["flops"] = benchmark::Counter(
    flopsPerIteration * static_cast<double>(state.iterations()),
    benchmark::Counter::kIsRate);
}

void registerBenchmarks()
{
  constexpr int matrixSizes[] = { 32, 64, 128, 256, 512, 1024 };

  for (EigenBlasBackend backend : availableEigenBlasBackends()) {
    for (int matrixN : matrixSizes) {
      const std::string name = std::string("EigenBlas/Gemm/") +
        eigenBlasBackendName(backend) + "/n_" + std::to_string(matrixN);
      benchmark::RegisterBenchmark(name.c_str(), [=](benchmark::State &state) {
        runGemmBenchmark(state, backend, matrixN);
      })->UseRealTime()
        ->Unit(benchmark::kMicrosecond);
    }
  }
}

const bool registered = [] {
  registerBenchmarks();
  return true;
}();

}  // namespace

BENCHMARK_MAIN();
