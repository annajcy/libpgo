#include "eigen_accelerate_nested_kernel.h"

#include "../parallelism_benchmark_helpers.h"
#include "parallel/parallel.h"

#include <Accelerate/Accelerate.h>
#include <benchmark/benchmark.h>

#include <tbb/blocked_range.h>
#include <tbb/info.h>
#include <tbb/parallel_for.h>
#include <tbb/partitioner.h>
#include <tbb/task_arena.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <optional>
#include <string>

namespace
{

namespace P = pgo::parallel;
using pgo::benchmark_helpers::adjustedExtraThreads;
using pgo::benchmark_helpers::NestedEigenAccelerateWorkload;
using pgo::benchmark_helpers::ThreadSampler;
using pgo::benchmark_helpers::updateMaximum;

enum class Policy
{
  ExecutorSingle = 0,
  ExecutorMulti = 1,
};

const char *policyName(Policy policy)
{
  switch (policy) {
  case Policy::ExecutorSingle:
    return "ExecutorSingle";
  case Policy::ExecutorMulti:
    return "ExecutorMulti";
  }
  return "Unknown";
}

struct RunTelemetry
{
  std::atomic<int> activeCallbacks{ 0 };
  std::atomic<int> peakCallbacks{ 0 };
  std::atomic<int> arenaConcurrency{ 0 };
  std::atomic<int> singleModeCalls{ 0 };
  std::atomic<int> multiModeCalls{ 0 };
  std::atomic<int> otherModeCalls{ 0 };
  std::atomic<int> bodyCalls{ 0 };

  void reset() noexcept
  {
    activeCallbacks.store(0, std::memory_order_relaxed);
    peakCallbacks.store(0, std::memory_order_relaxed);
    arenaConcurrency.store(0, std::memory_order_relaxed);
    singleModeCalls.store(0, std::memory_order_relaxed);
    multiModeCalls.store(0, std::memory_order_relaxed);
    otherModeCalls.store(0, std::memory_order_relaxed);
    bodyCalls.store(0, std::memory_order_relaxed);
  }
};

class ActiveCallback
{
public:
  explicit ActiveCallback(RunTelemetry &telemetry): telemetry_(telemetry)
  {
    const int active =
      telemetry_.activeCallbacks.fetch_add(1, std::memory_order_relaxed) + 1;
    updateMaximum(telemetry_.peakCallbacks, active);
  }

  ~ActiveCallback()
  {
    telemetry_.activeCallbacks.fetch_sub(1, std::memory_order_relaxed);
  }

private:
  RunTelemetry &telemetry_;
};

void recordAccelerateMode(RunTelemetry &telemetry, BLAS_THREADING mode)
{
  switch (mode) {
  case BLAS_THREADING_SINGLE_THREADED:
    telemetry.singleModeCalls.fetch_add(1, std::memory_order_relaxed);
    break;
  case BLAS_THREADING_MULTI_THREADED:
    telemetry.multiModeCalls.fetch_add(1, std::memory_order_relaxed);
    break;
  default:
    telemetry.otherModeCalls.fetch_add(1, std::memory_order_relaxed);
    break;
  }
}

void runOuterLoop(Policy policy, int outerTasks,
  NestedEigenAccelerateWorkload &workload, RunTelemetry &telemetry,
  P::ArenaThreadingExecutor &singleExecutor,
  P::ArenaThreadingExecutor &multiExecutor)
{
  const auto body = [&](int taskIndex) {
    ActiveCallback active(telemetry);
    updateMaximum(telemetry.arenaConcurrency,
      tbb::this_task_arena::max_concurrency());
    recordAccelerateMode(telemetry, BLASGetThreading());
    workload.run(taskIndex);
    telemetry.bodyCalls.fetch_add(1, std::memory_order_relaxed);
  };

  auto &executor = policy == Policy::ExecutorSingle ? singleExecutor : multiExecutor;
  executor.execute([&] {
    tbb::parallel_for(
      tbb::blocked_range<int>(0, outerTasks, 1),
      [&](const tbb::blocked_range<int> &range) {
        for (int taskIndex = range.begin(); taskIndex < range.end(); ++taskIndex)
          body(taskIndex);
      },
      tbb::auto_partitioner{});
  });
}

void runBenchmark(benchmark::State &state,
  Policy policy, int configuredConcurrency, int outerTasks, int matrixN)
{
  P::GlobalTbbControl control(configuredConcurrency);
  const int effectiveConcurrency = static_cast<int>(tbb::global_control::active_value(
    tbb::global_control::max_allowed_parallelism));
  P::ArenaThreadingExecutor singleExecutor(configuredConcurrency,
    { .accelerate = P::AccelerateThreading::single });
  P::ArenaThreadingExecutor multiExecutor(configuredConcurrency,
    { .accelerate = P::AccelerateThreading::multi });
  NestedEigenAccelerateWorkload workload(outerTasks, matrixN);
  RunTelemetry telemetry;

  ThreadSampler sampler;
  sampler.start();
  runOuterLoop(policy, outerTasks, workload, telemetry, singleExecutor, multiExecutor);
  const int peakThreads = sampler.stop();
  const int baselineThreads = sampler.baseline();

  telemetry.reset();
  for (auto _ : state) {
    runOuterLoop(policy, outerTasks, workload, telemetry, singleExecutor, multiExecutor);
    benchmark::DoNotOptimize(&workload);
    benchmark::ClobberMemory();
  }

  const double checksum = workload.checksum();
  if (!std::isfinite(checksum)) {
    state.SkipWithError("Nested Eigen/Accelerate GEMM produced a non-finite checksum.");
    return;
  }

  const double flopsPerIteration =
    2.0 * static_cast<double>(matrixN) * static_cast<double>(matrixN) *
    static_cast<double>(matrixN) * static_cast<double>(outerTasks);
  state.counters["policy"] = static_cast<int>(policy);
  state.counters["configured_concurrency"] = configuredConcurrency;
  state.counters["effective_concurrency"] = effectiveConcurrency;
  state.counters["outer_tasks"] = outerTasks;
  state.counters["matrix_n"] = matrixN;
  state.counters["arena_concurrency"] =
    telemetry.arenaConcurrency.load(std::memory_order_relaxed);
  state.counters["outer_active_peak"] =
    telemetry.peakCallbacks.load(std::memory_order_relaxed);
  state.counters["single_mode_calls"] =
    telemetry.singleModeCalls.load(std::memory_order_relaxed);
  state.counters["multi_mode_calls"] =
    telemetry.multiModeCalls.load(std::memory_order_relaxed);
  state.counters["other_mode_calls"] =
    telemetry.otherModeCalls.load(std::memory_order_relaxed);
  state.counters["body_calls"] = telemetry.bodyCalls.load(std::memory_order_relaxed);
  state.counters["baseline_threads"] = baselineThreads;
  state.counters["peak_threads"] = peakThreads;
  state.counters["extra_threads"] =
    adjustedExtraThreads(baselineThreads, peakThreads);
  state.counters["checksum"] = checksum;
  state.counters["flops"] = benchmark::Counter(
    flopsPerIteration * static_cast<double>(state.iterations()),
    benchmark::Counter::kIsRate);
}

void registerBenchmarks()
{
  constexpr Policy policies[] = {
    Policy::ExecutorSingle,
    Policy::ExecutorMulti,
  };
  constexpr int concurrencyValues[] = { 1, 2, 4, 8, 16 };
  constexpr struct
  {
    int matrixN;
    int taskMultiplier;
  } loads[] = {
    { 128, 4 },
    { 256, 4 },
    { 512, 2 },
    { 1024, 1 },
  };

  const int defaultConcurrency = std::max(1, tbb::info::default_concurrency());
  for (int concurrency : concurrencyValues) {
    if (concurrency > defaultConcurrency)
      continue;
    for (const auto &load : loads) {
      const int outerTasks = concurrency * load.taskMultiplier;
      for (Policy policy : policies) {
        const std::string name = std::string("NestedEigenAccelerate/") +
          policyName(policy) + "/c_" + std::to_string(concurrency) +
          "/tasks_" + std::to_string(outerTasks) +
          "/n_" + std::to_string(load.matrixN);
        benchmark::RegisterBenchmark(name.c_str(), [=](benchmark::State &state) {
          runBenchmark(state, policy, concurrency, outerTasks, load.matrixN);
        })->UseRealTime()
          ->Unit(benchmark::kMillisecond);
      }
    }
  }
}

const bool registered = [] {
  registerBenchmarks();
  return true;
}();

}  // namespace

BENCHMARK_MAIN();
