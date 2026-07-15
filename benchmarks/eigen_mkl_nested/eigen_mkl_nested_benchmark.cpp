#include "eigen_mkl_nested_kernel.h"

#include "../parallelism_benchmark_helpers.h"
#include "parallel/parallelControl.h"
#include "parallel/parallelFor.h"

#include <benchmark/benchmark.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/partitioner.h>
#include <tbb/task_arena.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <optional>
#include <set>
#include <string>

namespace
{

namespace P = pgo::parallel;
using pgo::benchmark_helpers::adjustedExtraThreads;
using pgo::benchmark_helpers::NestedEigenMklWorkload;
using pgo::benchmark_helpers::ThreadSampler;

enum class Policy
{
  PgoMulti = 0,
  PgoSingle = 1,
  TbbMulti = 2,
  TbbSingle = 3,
};

const char *policyName(Policy policy)
{
  switch (policy) {
  case Policy::PgoMulti:
    return "PgoMulti";
  case Policy::PgoSingle:
    return "PgoSingle";
  case Policy::TbbMulti:
    return "TbbMulti";
  case Policy::TbbSingle:
    return "TbbSingle";
  }
  return "Unknown";
}

bool usesPgoOuterLoop(Policy policy)
{
  return policy == Policy::PgoMulti || policy == Policy::PgoSingle;
}

bool usesSingleParticipantInnerArena(Policy policy)
{
  return policy == Policy::PgoSingle || policy == Policy::TbbSingle;
}

void updateMaximum(std::atomic<int> &target, int value) noexcept
{
  int observed = target.load(std::memory_order_relaxed);
  while (value > observed &&
    !target.compare_exchange_weak(observed, value, std::memory_order_relaxed)) {
  }
}

struct RunTelemetry
{
  std::atomic<int> activeOuterCallbacks{ 0 };
  std::atomic<int> peakOuterCallbacks{ 0 };
  std::atomic<int> outerArenaConcurrency{ 0 };
  std::atomic<int> innerArenaConcurrency{ 0 };
  std::atomic<int> bodyCalls{ 0 };

  void reset() noexcept
  {
    activeOuterCallbacks.store(0, std::memory_order_relaxed);
    peakOuterCallbacks.store(0, std::memory_order_relaxed);
    outerArenaConcurrency.store(0, std::memory_order_relaxed);
    innerArenaConcurrency.store(0, std::memory_order_relaxed);
    bodyCalls.store(0, std::memory_order_relaxed);
  }
};

class ActiveOuterCallback
{
public:
  explicit ActiveOuterCallback(RunTelemetry &telemetry): telemetry_(telemetry)
  {
    const int active =
      telemetry_.activeOuterCallbacks.fetch_add(1, std::memory_order_relaxed) + 1;
    updateMaximum(telemetry_.peakOuterCallbacks, active);
  }

  ~ActiveOuterCallback()
  {
    telemetry_.activeOuterCallbacks.fetch_sub(1, std::memory_order_relaxed);
  }

private:
  RunTelemetry &telemetry_;
};

void runOuterLoop(
  Policy policy, int outerTasks, NestedEigenMklWorkload &workload, RunTelemetry &telemetry)
{
  const auto body = [&](int taskIndex) {
    ActiveOuterCallback active(telemetry);
    updateMaximum(telemetry.outerArenaConcurrency,
      tbb::this_task_arena::max_concurrency());

    const auto runInnerKernel = [&] {
      updateMaximum(telemetry.innerArenaConcurrency,
        tbb::this_task_arena::max_concurrency());
      workload.run(taskIndex);
    };

    // Every policy executes the same Eigen -> oneMKL BLAS kernel. The Single
    // policies only place that call in a one-participant TBB arena, so oneMKL
    // keeps using its TBB threading layer but cannot recruit another worker.
    if (usesSingleParticipantInnerArena(policy))
      P::withSingleThreadedTbb(runInnerKernel);
    else
      runInnerKernel();

    telemetry.bodyCalls.fetch_add(1, std::memory_order_relaxed);
  };

  if (usesPgoOuterLoop(policy)) {
    P::parallelFor(0, outerTasks, body,
      std::optional<int>{ 1 }, tbb::auto_partitioner{});
    return;
  }

  tbb::parallel_for(
    tbb::blocked_range<int>(0, outerTasks, 1),
    [&](const tbb::blocked_range<int> &range) {
      for (int taskIndex = range.begin(); taskIndex < range.end(); ++taskIndex)
        body(taskIndex);
    },
    tbb::auto_partitioner{});
}

void runBenchmark(benchmark::State &state,
  Policy policy, int configuredConcurrency, int outerTasks, int matrixN)
{
  const int effectiveConcurrency = P::setMaxConcurrency(configuredConcurrency);
  NestedEigenMklWorkload workload(outerTasks, matrixN);
  RunTelemetry telemetry;

  ThreadSampler sampler;
  sampler.start();
  runOuterLoop(policy, outerTasks, workload, telemetry);
  const int peakThreads = sampler.stop();
  const int baselineThreads = sampler.baseline();

  telemetry.reset();
  for (auto _ : state) {
    runOuterLoop(policy, outerTasks, workload, telemetry);
    benchmark::DoNotOptimize(&workload);
    benchmark::ClobberMemory();
  }

  const double checksum = workload.checksum();
  if (!std::isfinite(checksum)) {
    state.SkipWithError("Nested Eigen/MKL GEMM produced a non-finite checksum.");
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
  state.counters["outer_arena_concurrency"] =
    telemetry.outerArenaConcurrency.load(std::memory_order_relaxed);
  state.counters["inner_arena_concurrency"] =
    telemetry.innerArenaConcurrency.load(std::memory_order_relaxed);
  state.counters["outer_active_peak"] =
    telemetry.peakOuterCallbacks.load(std::memory_order_relaxed);
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
    Policy::PgoMulti,
    Policy::PgoSingle,
    Policy::TbbMulti,
    Policy::TbbSingle,
  };
  constexpr int concurrencyValues[] = { 1, 4, 8, 16 };
  constexpr int matrixSizes[] = { 256, 1024 };

  for (int concurrency : concurrencyValues) {
    const std::set<int> taskCounts = { 1, concurrency, concurrency * 4 };
    for (int outerTasks : taskCounts) {
      for (int matrixN : matrixSizes) {
        for (Policy policy : policies) {
          const std::string name = std::string("NestedEigenMkl/") +
            policyName(policy) + "/c_" + std::to_string(concurrency) +
            "/tasks_" + std::to_string(outerTasks) +
            "/n_" + std::to_string(matrixN);
          benchmark::RegisterBenchmark(name.c_str(), [=](benchmark::State &state) {
            runBenchmark(state, policy, concurrency, outerTasks, matrixN);
          })->UseRealTime()
            ->Unit(benchmark::kMillisecond);
        }
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
