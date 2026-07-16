#include "../eigen_blas/eigen_gemm_kernel.h"
#include "../eigen_mkl_common/eigen_mkl_executor_cases.h"

#include "parallel/arenaThreadingExecutor.h"
#include "parallel/parallelControl.h"

#include <benchmark/benchmark.h>

#include <tbb/info.h>
#include <tbb/task_arena.h>

#include <array>
#include <charconv>
#include <cmath>
#include <cstdlib>
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

constexpr std::string_view concurrencyEnvironment =
  "PGO_EIGEN_MKL_CROSSOVER_MAX_CONCURRENCY";

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

void runBenchmark(
  benchmark::State &state, MklExecutorCase policy, int matrixN)
{
  const std::optional<int> requested = requestedConcurrency();
  const int configuredConcurrency =
    requested.value_or(tbb::info::default_concurrency());
  P::GlobalTbbControl control(configuredConcurrency);
  const int effectiveConcurrency = static_cast<int>(tbb::global_control::active_value(
    tbb::global_control::max_allowed_parallelism));
  const auto spec = mklExecutorSpec(policy, configuredConcurrency);
  P::ArenaThreadingExecutor executor(spec.arenaConcurrency,
    { .mklLocalThreadBudget = spec.mklLocalThreadBudget });
  EigenGemmWorkspace workspace(matrixN);
  int observedArenaConcurrency = 0;

  executor.execute([&] {
    observedArenaConcurrency = tbb::this_task_arena::max_concurrency();
    workspace.run();
    for (auto _ : state) {
      workspace.run();
      benchmark::DoNotOptimize(&workspace);
      benchmark::ClobberMemory();
    }
  });

  const double checksum = workspace.checksum();
  if (!std::isfinite(checksum)) {
    state.SkipWithError("Eigen/MKL GEMM produced a non-finite checksum.");
    return;
  }

  const double flopsPerIteration =
    2.0 * static_cast<double>(matrixN) * static_cast<double>(matrixN) *
    static_cast<double>(matrixN);
  state.counters["policy"] = static_cast<int>(policy);
  state.counters["matrix_n"] = matrixN;
  state.counters["requested_concurrency"] = requested.value_or(0);
  state.counters["effective_global_concurrency"] = effectiveConcurrency;
  state.counters["configured_arena_concurrency"] = spec.arenaConcurrency;
  state.counters["configured_mkl_local_budget"] = spec.mklLocalThreadBudget;
  state.counters["observed_arena_concurrency"] = observedArenaConcurrency;
  state.counters["checksum"] = checksum;
  state.counters["flops"] = benchmark::Counter(
    flopsPerIteration * static_cast<double>(state.iterations()),
    benchmark::Counter::kIsRate);
}

void registerBenchmarks()
{
  constexpr std::array policies = {
    MklExecutorCase::ExecutorLocal1,
    MklExecutorCase::ExecutorMKLC,
  };

  for (int matrixN = 64; matrixN <= 128; matrixN += 4) {
    for (MklExecutorCase policy : policies) {
      const std::string name = std::string("EigenMklCrossover/") +
        mklExecutorCaseName(policy) + "/n_" + std::to_string(matrixN);
      benchmark::RegisterBenchmark(name.c_str(), [=](benchmark::State &state) {
        runBenchmark(state, policy, matrixN);
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
