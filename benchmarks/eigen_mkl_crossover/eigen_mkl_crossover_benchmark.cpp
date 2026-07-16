#include "../eigen_blas/eigen_gemm_kernel.h"

#include "parallel/parallelControl.h"

#include <benchmark/benchmark.h>

#include <tbb/info.h>
#include <tbb/task_arena.h>

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

constexpr std::string_view concurrencyEnvironment =
  "PGO_EIGEN_MKL_CROSSOVER_MAX_CONCURRENCY";

enum class Policy
{
  MklTbbSingle = 0,
  MklTbbGlobal = 1,
};

const char *policyName(Policy policy) noexcept
{
  switch (policy) {
  case Policy::MklTbbSingle:
    return "MklTbbSingle";
  case Policy::MklTbbGlobal:
    return "MklTbbGlobal";
  }
  return "Unknown";
}

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

void runBenchmark(benchmark::State &state, Policy policy, int matrixN)
{
  const std::optional<int> requested = requestedConcurrency();
  const int configuredConcurrency = requested.value_or(tbb::info::default_concurrency());
  P::GlobalTbbControl control(configuredConcurrency);
  const int effectiveConcurrency = static_cast<int>(tbb::global_control::active_value(
    tbb::global_control::max_allowed_parallelism));
  tbb::task_arena arena(configuredConcurrency, 1);
  tbb::task_arena singleArena(1, 1);
  EigenGemmWorkspace workspace(matrixN);
  int arenaConcurrency = 0;

  const auto measure = [&] {
    arenaConcurrency = tbb::this_task_arena::max_concurrency();

    // Prime the selected MKL-TBB policy before Google Benchmark starts timing.
    workspace.run();
    for (auto _ : state) {
      workspace.run();
      benchmark::DoNotOptimize(&workspace);
      benchmark::ClobberMemory();
    }
  };

  if (policy == Policy::MklTbbSingle)
    singleArena.execute(measure);
  else
    arena.execute(measure);

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
  state.counters["effective_concurrency"] = effectiveConcurrency;
  state.counters["arena_concurrency"] = arenaConcurrency;
  state.counters["checksum"] = checksum;
  state.counters["flops"] = benchmark::Counter(
    flopsPerIteration * static_cast<double>(state.iterations()),
    benchmark::Counter::kIsRate);
}

void registerBenchmarks()
{
  constexpr Policy policies[] = {
    Policy::MklTbbSingle,
    Policy::MklTbbGlobal,
  };

  for (int matrixN = 64; matrixN <= 128; matrixN += 4) {
    for (Policy policy : policies) {
      const std::string name = std::string("EigenMklCrossover/") +
        policyName(policy) + "/n_" + std::to_string(matrixN);
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
