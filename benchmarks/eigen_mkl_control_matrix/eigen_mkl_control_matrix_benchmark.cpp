#include "../eigen_mkl_nested/eigen_mkl_nested_kernel.h"

#include "no_blas_workload.h"

#include "../parallelism_benchmark_helpers.h"
#include "parallel/parallelControl.h"
#include "parallel/parallelFor.h"

#include <benchmark/benchmark.h>
#include <mkl.h>

#include <tbb/partitioner.h>
#include <tbb/task_arena.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

namespace
{

namespace P = pgo::parallel;
using pgo::benchmark_helpers::adjustedExtraThreads;
using pgo::benchmark_helpers::NestedEigenMklWorkload;
using pgo::benchmark_helpers::NoBlasWorkload;
using pgo::benchmark_helpers::ThreadSampler;

enum class WorkloadKind
{
  EigenMklGemm = 0,
  NoBlas = 1,
};

const char *workloadName(WorkloadKind workload) noexcept
{
  switch (workload) {
  case WorkloadKind::EigenMklGemm:
    return "EigenMklGemm";
  case WorkloadKind::NoBlas:
    return "NoBlas";
  }
  return "Unknown";
}

bool usesBlas(WorkloadKind workload) noexcept
{
  return workload == WorkloadKind::EigenMklGemm;
}

enum class Policy
{
  DefaultArenaGlobal = 0,
  Local1ArenaGlobal = 1,
  DefaultArena1 = 2,
  Local1Arena1 = 3,
};

const char *policyName(Policy policy) noexcept
{
  switch (policy) {
  case Policy::DefaultArenaGlobal:
    return "DefaultArenaGlobal";
  case Policy::Local1ArenaGlobal:
    return "Local1ArenaGlobal";
  case Policy::DefaultArena1:
    return "DefaultArena1";
  case Policy::Local1Arena1:
    return "Local1Arena1";
  }
  return "Unknown";
}

bool usesMklLocalOne(Policy policy) noexcept
{
  return policy == Policy::Local1ArenaGlobal || policy == Policy::Local1Arena1;
}

bool usesSingleParticipantArena(Policy policy) noexcept
{
  return policy == Policy::DefaultArena1 || policy == Policy::Local1Arena1;
}

void updateMaximum(std::atomic<int> &target, int value) noexcept
{
  int observed = target.load(std::memory_order_relaxed);
  while (value > observed &&
    !target.compare_exchange_weak(observed, value, std::memory_order_relaxed)) {
  }
}

void updateMinimum(std::atomic<int> &target, int value) noexcept
{
  int observed = target.load(std::memory_order_relaxed);
  while (value < observed &&
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
  std::atomic<int> mklApiReportedMin{ std::numeric_limits<int>::max() };
  std::atomic<int> mklApiReportedMax{ 0 };
  std::atomic<int> restorationMismatches{ 0 };

  void reset() noexcept
  {
    activeOuterCallbacks.store(0, std::memory_order_relaxed);
    peakOuterCallbacks.store(0, std::memory_order_relaxed);
    outerArenaConcurrency.store(0, std::memory_order_relaxed);
    innerArenaConcurrency.store(0, std::memory_order_relaxed);
    bodyCalls.store(0, std::memory_order_relaxed);
    mklApiReportedMin.store(std::numeric_limits<int>::max(), std::memory_order_relaxed);
    mklApiReportedMax.store(0, std::memory_order_relaxed);
    restorationMismatches.store(0, std::memory_order_relaxed);
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

class ScopedMklLocalOne
{
public:
  ScopedMklLocalOne(): previous_(mkl_set_num_threads_local(1)) {}

  ~ScopedMklLocalOne()
  {
    mkl_set_num_threads_local(previous_);
  }

  ScopedMklLocalOne(const ScopedMklLocalOne &) = delete;
  ScopedMklLocalOne &operator=(const ScopedMklLocalOne &) = delete;

private:
  int previous_ = 0;
};

template<class Fn>
void runWithMklApiSetting(Policy policy, RunTelemetry &telemetry, Fn &&fn)
{
  const auto observeAndRun = [&] {
    // This is diagnostic service-API state, not evidence of the number of
    // participants actually used by the oneMKL TBB threading layer.
    const int reported = mkl_get_max_threads();
    updateMinimum(telemetry.mklApiReportedMin, reported);
    updateMaximum(telemetry.mklApiReportedMax, reported);
    fn();
  };

  if (!usesMklLocalOne(policy)) {
    observeAndRun();
    return;
  }

  const int before = mkl_get_max_threads();
  {
    // Intel documents this as an OpenMP control with no supported effect under
    // the TBB threading layer. It is included only to reproduce the old
    // benchmark intervention and must not be treated as a production policy.
    ScopedMklLocalOne localOne;
    observeAndRun();
  }
  if (mkl_get_max_threads() != before)
    telemetry.restorationMismatches.fetch_add(1, std::memory_order_relaxed);
}

template<class Workload>
void runOuterLoop(Policy policy, int outerTasks, Workload &workload, RunTelemetry &telemetry)
{
  const auto body = [&](int taskIndex) {
    ActiveOuterCallback active(telemetry);
    updateMaximum(telemetry.outerArenaConcurrency,
      tbb::this_task_arena::max_concurrency());

    const auto runInnerKernel = [&] {
      updateMaximum(telemetry.innerArenaConcurrency,
        tbb::this_task_arena::max_concurrency());
      runWithMklApiSetting(policy, telemetry, [&] {
        workload.run(taskIndex);
      });
    };

    if (usesSingleParticipantArena(policy))
      P::withSingleThreadedTbb(runInnerKernel);
    else
      runInnerKernel();

    telemetry.bodyCalls.fetch_add(1, std::memory_order_relaxed);
  };

  P::parallelFor(0, outerTasks, body,
    std::optional<int>{ 1 }, tbb::auto_partitioner{});
}

template<class Workload>
void runWorkloadBenchmark(benchmark::State &state, WorkloadKind workloadKind,
  Policy policy, int configuredConcurrency, int outerTasks, int matrixN)
{
  const int effectiveConcurrency = P::initialize(configuredConcurrency);
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
    state.SkipWithError("MKL/TBB control-matrix GEMM produced a non-finite checksum.");
    return;
  }

  const int reportedMin = telemetry.mklApiReportedMin.load(std::memory_order_relaxed);
  const double flopsPerIteration =
    2.0 * static_cast<double>(matrixN) * static_cast<double>(matrixN) *
    static_cast<double>(matrixN) * static_cast<double>(outerTasks);
  state.counters["policy"] = static_cast<int>(policy);
  state.counters["workload"] = static_cast<int>(workloadKind);
  state.counters["uses_blas"] = usesBlas(workloadKind) ? 1 : 0;
  state.counters["configured_concurrency"] = configuredConcurrency;
  state.counters["effective_concurrency"] = effectiveConcurrency;
  state.counters["outer_tasks"] = outerTasks;
  state.counters["matrix_n"] = matrixN;
  state.counters["uses_mkl_local_one"] = usesMklLocalOne(policy) ? 1 : 0;
  state.counters["uses_arena_one"] = usesSingleParticipantArena(policy) ? 1 : 0;
  state.counters["outer_arena_concurrency"] =
    telemetry.outerArenaConcurrency.load(std::memory_order_relaxed);
  state.counters["inner_arena_concurrency"] =
    telemetry.innerArenaConcurrency.load(std::memory_order_relaxed);
  state.counters["outer_active_peak"] =
    telemetry.peakOuterCallbacks.load(std::memory_order_relaxed);
  state.counters["body_calls"] = telemetry.bodyCalls.load(std::memory_order_relaxed);
  state.counters["mkl_api_reported_min"] =
    reportedMin == std::numeric_limits<int>::max() ? 0 : reportedMin;
  state.counters["mkl_api_reported_max"] =
    telemetry.mklApiReportedMax.load(std::memory_order_relaxed);
  state.counters["restoration_mismatches"] =
    telemetry.restorationMismatches.load(std::memory_order_relaxed);
  state.counters["baseline_threads"] = baselineThreads;
  state.counters["peak_threads"] = peakThreads;
  state.counters["extra_threads"] =
    adjustedExtraThreads(baselineThreads, peakThreads);
  state.counters["checksum"] = checksum;
  state.counters["flops"] = benchmark::Counter(
    flopsPerIteration * static_cast<double>(state.iterations()),
    benchmark::Counter::kIsRate);
}

void runBenchmark(benchmark::State &state, WorkloadKind workloadKind,
  Policy policy, int configuredConcurrency, int outerTasks, int matrixN)
{
  switch (workloadKind) {
  case WorkloadKind::EigenMklGemm:
    runWorkloadBenchmark<NestedEigenMklWorkload>(state, workloadKind, policy,
      configuredConcurrency, outerTasks, matrixN);
    return;
  case WorkloadKind::NoBlas:
    runWorkloadBenchmark<NoBlasWorkload>(state, workloadKind, policy,
      configuredConcurrency, outerTasks, matrixN);
    return;
  }
  state.SkipWithError("Unknown control-matrix workload.");
}

void registerBenchmarks()
{
  constexpr Policy policies[] = {
    Policy::DefaultArenaGlobal,
    Policy::Local1ArenaGlobal,
    Policy::DefaultArena1,
    Policy::Local1Arena1,
  };
  constexpr int concurrencyValues[] = { 4, 8 };
  constexpr WorkloadKind workloads[] = {
    WorkloadKind::EigenMklGemm,
    WorkloadKind::NoBlas,
  };
  constexpr int matrixN = 1024;

  for (int concurrency : concurrencyValues) {
    constexpr int taskMultipliers[] = { 1, 4 };
    for (int taskMultiplier : taskMultipliers) {
      const int outerTasks = concurrency * taskMultiplier;
      for (WorkloadKind workload : workloads) {
        for (Policy policy : policies) {
          const std::string name = std::string("EigenMklControlMatrix/") +
            workloadName(workload) + "/" + policyName(policy) + "/c_" +
            std::to_string(concurrency) + "/tasks_" +
            std::to_string(outerTasks) + "/n_" + std::to_string(matrixN);
          benchmark::RegisterBenchmark(name.c_str(), [=](benchmark::State &state) {
            runBenchmark(state, workload, policy, concurrency, outerTasks, matrixN);
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
