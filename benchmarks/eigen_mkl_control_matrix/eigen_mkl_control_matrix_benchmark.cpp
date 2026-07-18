#if !defined(PGO_EIGEN_MKL_CONTROL_NO_BLAS)
#  include "../eigen_mkl_common/eigen_mkl_gemm_workload.h"
#endif

#if defined(PGO_EIGEN_MKL_CONTROL_NO_BLAS)
#  include "no_blas_workload.h"
#endif

#include "../eigen_mkl_common/eigen_mkl_executor_cases.h"
#include "../parallelism_benchmark_helpers.h"
#include "parallel/arenaThreadingExecutor.h"
#include "parallel/parallelControl.h"

#include <benchmark/benchmark.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/partitioner.h>
#include <tbb/task_arena.h>

#include <atomic>
#include <cmath>
#include <string>

namespace
{

namespace P = pgo::parallel;
using pgo::benchmark_helpers::adjustedExtraThreads;
using pgo::benchmark_helpers::allMklExecutorCases;
#if !defined(PGO_EIGEN_MKL_CONTROL_NO_BLAS)
using pgo::benchmark_helpers::EigenMklGemmWorkload;
#endif
using pgo::benchmark_helpers::MklExecutorCase;
using pgo::benchmark_helpers::mklExecutorCaseName;
using pgo::benchmark_helpers::mklExecutorSpec;
#if defined(PGO_EIGEN_MKL_CONTROL_NO_BLAS)
using pgo::benchmark_helpers::NoBlasWorkload;
#endif
using pgo::benchmark_helpers::ThreadSampler;
using pgo::benchmark_helpers::updateMaximum;

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

struct RunTelemetry
{
  std::atomic<int> activeOuterCallbacks{ 0 };
  std::atomic<int> peakOuterCallbacks{ 0 };
  std::atomic<int> observedArenaConcurrency{ 0 };
  std::atomic<int> bodyCalls{ 0 };

  void reset() noexcept
  {
    activeOuterCallbacks.store(0, std::memory_order_relaxed);
    peakOuterCallbacks.store(0, std::memory_order_relaxed);
    observedArenaConcurrency.store(0, std::memory_order_relaxed);
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

template<class Workload>
void runOuterLoop(P::ArenaThreadingExecutor &executor, int outerTasks,
  Workload &workload, RunTelemetry &telemetry)
{
  executor.execute([&] {
    tbb::parallel_for(
      tbb::blocked_range<int>(0, outerTasks, 1),
      [&](const tbb::blocked_range<int> &range) {
        for (int taskIndex = range.begin(); taskIndex < range.end(); ++taskIndex) {
          ActiveOuterCallback active(telemetry);
          updateMaximum(telemetry.observedArenaConcurrency,
            tbb::this_task_arena::max_concurrency());
          workload.run(taskIndex);
          telemetry.bodyCalls.fetch_add(1, std::memory_order_relaxed);
        }
      },
      tbb::auto_partitioner{});
  });
}

template<class Workload>
void runWorkloadBenchmark(benchmark::State &state, WorkloadKind workloadKind,
  MklExecutorCase policy, int configuredConcurrency, int outerTasks, int matrixN)
{
  P::GlobalTbbControl control(configuredConcurrency);
  const int effectiveConcurrency = static_cast<int>(tbb::global_control::active_value(
    tbb::global_control::max_allowed_parallelism));
  const auto spec = mklExecutorSpec(policy, configuredConcurrency);
  P::ArenaThreadingExecutor executor(spec.arenaConcurrency,
    { .mklLocalThreadBudget = spec.mklLocalThreadBudget });
  Workload workload(outerTasks, matrixN);
  RunTelemetry telemetry;

  ThreadSampler sampler;
  sampler.start();
  runOuterLoop(executor, outerTasks, workload, telemetry);
  const int peakThreads = sampler.stop();
  const int baselineThreads = sampler.baseline();

  telemetry.reset();
  for (auto _ : state) {
    runOuterLoop(executor, outerTasks, workload, telemetry);
    benchmark::DoNotOptimize(&workload);
    benchmark::ClobberMemory();
  }

  const double checksum = workload.checksum();
  if (!std::isfinite(checksum)) {
    state.SkipWithError("MKL/TBB control-matrix workload produced a non-finite checksum.");
    return;
  }

  const double flopsPerIteration =
    2.0 * static_cast<double>(matrixN) * static_cast<double>(matrixN) *
    static_cast<double>(matrixN) * static_cast<double>(outerTasks);
  state.counters["policy"] = static_cast<int>(policy);
  state.counters["workload"] = static_cast<int>(workloadKind);
  state.counters["uses_blas"] = usesBlas(workloadKind) ? 1 : 0;
  state.counters["configured_global_concurrency"] = configuredConcurrency;
  state.counters["effective_global_concurrency"] = effectiveConcurrency;
  state.counters["configured_arena_concurrency"] = spec.arenaConcurrency;
  state.counters["configured_mkl_local_budget"] = spec.mklLocalThreadBudget;
  state.counters["outer_tasks"] = outerTasks;
  state.counters["matrix_n"] = matrixN;
  state.counters["observed_arena_concurrency"] =
    telemetry.observedArenaConcurrency.load(std::memory_order_relaxed);
  state.counters["outer_active_peak"] =
    telemetry.peakOuterCallbacks.load(std::memory_order_relaxed);
  state.counters["body_calls"] =
    telemetry.bodyCalls.load(std::memory_order_relaxed);
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
  MklExecutorCase policy, int configuredConcurrency, int outerTasks, int matrixN)
{
  switch (workloadKind) {
#if !defined(PGO_EIGEN_MKL_CONTROL_NO_BLAS)
  case WorkloadKind::EigenMklGemm:
    runWorkloadBenchmark<EigenMklGemmWorkload>(state, workloadKind, policy,
      configuredConcurrency, outerTasks, matrixN);
    return;
#endif
#if defined(PGO_EIGEN_MKL_CONTROL_NO_BLAS)
  case WorkloadKind::NoBlas:
    runWorkloadBenchmark<NoBlasWorkload>(state, workloadKind, policy,
      configuredConcurrency, outerTasks, matrixN);
    return;
#endif
  }
  state.SkipWithError("Unknown control-matrix workload.");
}

void registerBenchmarks()
{
  constexpr int concurrencyValues[] = { 4, 8 };
#if defined(PGO_EIGEN_MKL_CONTROL_NO_BLAS)
  constexpr WorkloadKind workloads[] = { WorkloadKind::NoBlas };
#else
  constexpr WorkloadKind workloads[] = { WorkloadKind::EigenMklGemm };
#endif
  constexpr int matrixN = 1024;

  for (int concurrency : concurrencyValues) {
    constexpr int taskMultipliers[] = { 1, 4 };
    for (int taskMultiplier : taskMultipliers) {
      const int outerTasks = concurrency * taskMultiplier;
      for (WorkloadKind workload : workloads) {
        for (MklExecutorCase policy : allMklExecutorCases) {
          const std::string name = std::string("EigenMklControlMatrix/") +
            workloadName(workload) + "/" + mklExecutorCaseName(policy) + "/c_" +
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
