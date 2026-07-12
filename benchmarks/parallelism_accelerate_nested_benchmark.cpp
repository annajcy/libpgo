#include "parallelism/parallelFor.h"
#include "parallelism/parallelOptions.h"
#include "parallelism_benchmark_helpers.h"
#include "parallelism_policy_workloads.h"

#include <Accelerate/Accelerate.h>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <atomic>
#include <numeric>
#include <string>
#include <vector>

namespace
{

namespace P = pgo::parallel;
using pgo::benchmark_helpers::MatrixPool;
using pgo::benchmark_helpers::MatrixSet;
using pgo::benchmark_helpers::RepresentativeWorkload;
using pgo::benchmark_helpers::ThreadSampler;
using pgo::benchmark_helpers::adjustedExtraThreads;
using pgo::benchmark_helpers::currentProcessThreadCount;
using pgo::benchmark_helpers::runRepresentativeWorkload;
using pgo::benchmark_helpers::workloadName;

enum class Policy
{
  Suppress,
  Inherit,
};

enum class AccelerateMode
{
  Single,
  Multi,
};

constexpr int kThresholdWorkerLimit = 8;
constexpr int kRuntimeConcurrency = 8;

P::ParallelRuntime &benchmarkRuntime()
{
  return P::initializeRuntime({ .maxTbbConcurrency = kRuntimeConcurrency });
}

class ScopedAccelerateThreading
{
public:
  explicit ScopedAccelerateThreading(AccelerateMode mode): previousThreading_(BLASGetThreading())
  {
    const BLAS_THREADING requested =
      mode == AccelerateMode::Single ? BLAS_THREADING_SINGLE_THREADED : BLAS_THREADING_MULTI_THREADED;
    active_ = BLASSetThreading(requested) == 0;
  }

  ~ScopedAccelerateThreading()
  {
    if (active_)
      BLASSetThreading(previousThreading_);
  }

  ScopedAccelerateThreading(const ScopedAccelerateThreading &) = delete;
  ScopedAccelerateThreading &operator=(const ScopedAccelerateThreading &) = delete;

private:
  BLAS_THREADING previousThreading_;
  bool active_ = false;
};

P::Options optionsForPolicy(Policy policy)
{
  P::Options options;
  options.grainSize = 1;
  options.nestedKernelPolicy =
    policy == Policy::Suppress ? P::NestedKernelPolicy::Suppress : P::NestedKernelPolicy::Inherit;
  return options;
}

const char *policyName(Policy policy)
{
  return policy == Policy::Suppress ? "Suppress" : "Inherit";
}

const char *accelerateModeName(AccelerateMode mode)
{
  return mode == AccelerateMode::Single ? "Single" : "Multi";
}

double runAccelerateDgemm(MatrixSet &matrices)
{
#ifdef __clang__
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wdeprecated-declarations"
#endif
  cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
    matrices.n, matrices.n, matrices.n,
    1.0, matrices.a.data(), matrices.n,
    matrices.b.data(), matrices.n,
    0.0, matrices.c.data(), matrices.n);
#ifdef __clang__
#  pragma clang diagnostic pop
#endif
  return matrices.centerValue();
}

void recordCommonCounters(benchmark::State &state, int arenaConcurrency, int matrixN)
{
  const P::RuntimeInfo runtime = P::runtimeInfo();
  state.counters["runtime_concurrency"] = *runtime.maxConcurrency;
  state.counters["arena_concurrency"] = arenaConcurrency;
  state.counters["matrix_n"] = matrixN;
  state.counters["accelerate_threading"] = static_cast<int>(BLASGetThreading());
  state.counters["tbb_max_allowed_parallelism"] = runtime.effectiveTbbMaxAllowedParallelism;
  state.counters["peak_participants"] = runtime.peakTotalParticipants;
  state.counters["participant_pressure"] = runtime.participantPressureObserved ? 1 : 0;
}

void benchmarkRepresentativeWorkload(benchmark::State &state, RepresentativeWorkload workload,
  Policy policy, int outerWorkers, int outerTasks, int workItems)
{
  ScopedAccelerateThreading inheritedThreading(AccelerateMode::Multi);
  const P::ParallelExecutor executor =
    benchmarkRuntime().createExecutor({ .maxConcurrency = outerWorkers });
  std::vector<double> results(static_cast<std::size_t>(outerTasks), 0.0);
  std::atomic<int> observedThreadingInWorker = -1;
  const P::Options options = optionsForPolicy(policy);

  const int baselineThreads = currentProcessThreadCount();
  int peakThreads = baselineThreads;
  for (auto _ : state) {
    state.PauseTiming();
    ThreadSampler sampler;
    sampler.start();
    state.ResumeTiming();

    P::parallelFor(executor, 0, outerTasks, options, [&](int taskIndex) {
      observedThreadingInWorker.store(static_cast<int>(BLASGetThreading()), std::memory_order_relaxed);
      results[static_cast<std::size_t>(taskIndex)] =
        runRepresentativeWorkload(workload, taskIndex, workItems);
    });
    benchmark::DoNotOptimize(results.data());

    state.PauseTiming();
    peakThreads = std::max(peakThreads, sampler.stop());
    state.ResumeTiming();
  }

  const double checksum = std::accumulate(results.begin(), results.end(), 0.0);
  benchmark::DoNotOptimize(&checksum);
  recordCommonCounters(state, outerWorkers, 0);
  state.counters["outer_tasks"] = outerTasks;
  state.counters["work_items_per_task"] = workItems;
  state.counters["baseline_threads"] = baselineThreads;
  state.counters["peak_threads"] = peakThreads;
  state.counters["extra_threads"] = adjustedExtraThreads(baselineThreads, peakThreads);
  state.counters["policy"] = policy == Policy::Suppress ? 0 : 1;
  state.counters["worker_accelerate_threading"] = observedThreadingInWorker.load(std::memory_order_relaxed);
  state.counters["checksum"] = checksum;
}

void recordThresholdCounters(benchmark::State &state, int matrixN)
{
  const P::RuntimeInfo runtime = P::runtimeInfo();
  state.counters["thread_limit"] = kThresholdWorkerLimit;
  state.counters["matrix_n"] = matrixN;
  state.counters["accelerate_threading"] = static_cast<int>(BLASGetThreading());
  state.counters["tbb_max_allowed_parallelism"] = runtime.effectiveTbbMaxAllowedParallelism;
}

void benchmarkNestedParallelDgemm(
  benchmark::State &state, Policy policy, int outerWorkers, int outerTasks, int matrixN)
{
  ScopedAccelerateThreading inheritedThreading(AccelerateMode::Multi);
  const P::ParallelExecutor executor =
    benchmarkRuntime().createExecutor({ .maxConcurrency = outerWorkers });
  MatrixPool pool(std::max(1, outerWorkers), matrixN);
  std::vector<double> results(static_cast<std::size_t>(outerTasks), 0.0);
  std::atomic<int> observedThreadingInWorker = -1;

  int baselineThreads = currentProcessThreadCount();
  int maxIterationBaselineThreads = baselineThreads;
  int peakThreads = baselineThreads;
  const P::Options options = optionsForPolicy(policy);

  for (auto _ : state) {
    state.PauseTiming();
    ThreadSampler sampler;
    sampler.start();
    state.ResumeTiming();

    P::parallelFor(executor, 0, outerTasks, options, [&](int taskIndex) {
      observedThreadingInWorker.store(static_cast<int>(BLASGetThreading()), std::memory_order_relaxed);
      results[static_cast<std::size_t>(taskIndex)] = runAccelerateDgemm(pool.current());
    });

    benchmark::DoNotOptimize(results.data());

    state.PauseTiming();
    peakThreads = std::max(peakThreads, sampler.stop());
    maxIterationBaselineThreads = std::max(maxIterationBaselineThreads, sampler.baseline());
    state.ResumeTiming();
  }

  double checksum = std::accumulate(results.begin(), results.end(), 0.0);
  benchmark::DoNotOptimize(checksum);

  recordCommonCounters(state, outerWorkers, matrixN);
  state.counters["outer_tasks"] = outerTasks;
  state.counters["baseline_threads"] = baselineThreads;
  state.counters["max_iteration_baseline_threads"] = maxIterationBaselineThreads;
  state.counters["peak_threads"] = peakThreads;
  state.counters["raw_extra_threads"] = peakThreads - baselineThreads;
  state.counters["extra_threads"] = adjustedExtraThreads(baselineThreads, peakThreads);
  state.counters["policy"] = policy == Policy::Suppress ? 0 : 1;
  state.counters["worker_accelerate_threading"] = observedThreadingInWorker.load(std::memory_order_relaxed);
  state.counters["checksum"] = checksum;
}

void benchmarkAccelerateDgemmThreadThreshold(
  benchmark::State &state, AccelerateMode mode, int matrixN)
{
  benchmarkRuntime();
  ScopedAccelerateThreading threading(mode);
  MatrixSet matrices(matrixN);

  int baselineThreads = currentProcessThreadCount();
  int maxIterationBaselineThreads = baselineThreads;
  int peakThreads = baselineThreads;
  double checksum = 0.0;

  for (auto _ : state) {
    state.PauseTiming();
    ThreadSampler sampler;
    sampler.start();
    state.ResumeTiming();

    checksum += runAccelerateDgemm(matrices);
    benchmark::DoNotOptimize(checksum);

    state.PauseTiming();
    peakThreads = std::max(peakThreads, sampler.stop());
    maxIterationBaselineThreads = std::max(maxIterationBaselineThreads, sampler.baseline());
    state.ResumeTiming();
  }

  recordThresholdCounters(state, matrixN);
  state.counters["baseline_threads"] = baselineThreads;
  state.counters["max_iteration_baseline_threads"] = maxIterationBaselineThreads;
  state.counters["peak_threads"] = peakThreads;
  state.counters["raw_extra_threads"] = peakThreads - baselineThreads;
  state.counters["extra_threads"] = adjustedExtraThreads(baselineThreads, peakThreads);
  state.counters["accelerate_mode"] = mode == AccelerateMode::Single ? 0 : 1;
}

void registerNestedParallelDgemmBenchmarks()
{
  constexpr int workerCounts[] = { 1, 2, 4, 8, 16 };
  constexpr int matrixSizes[] = { 256, 512, 1024, 1536, 2048 };
  constexpr int taskMultipliers[] = { 1, 4 };
  constexpr Policy policies[] = { Policy::Suppress, Policy::Inherit };

  for (Policy policy : policies) {
    for (int outerWorkers : workerCounts) {
      for (int taskMultiplier : taskMultipliers) {
        for (int matrixN : matrixSizes) {
          const int outerTasks = outerWorkers * taskMultiplier;
          const std::string name = std::string("NestedParallelDgemmAccelerate/") + policyName(policy) +
            "/workers_" + std::to_string(outerWorkers) +
            "/tasks_" + std::to_string(outerTasks) +
            "/n_" + std::to_string(matrixN);
          benchmark::RegisterBenchmark(name.c_str(), [=](benchmark::State &state) {
            benchmarkNestedParallelDgemm(state, policy, outerWorkers, outerTasks, matrixN);
          })->UseRealTime()
            ->Unit(benchmark::kMillisecond);
        }
      }
    }
  }
}

void registerAccelerateDgemmThreadThresholdBenchmarks()
{
  constexpr int matrixSizes[] = { 32, 64, 96, 128, 192, 256, 384, 512, 768, 1024, 1536, 2048 };
  constexpr AccelerateMode modes[] = { AccelerateMode::Single, AccelerateMode::Multi };

  for (AccelerateMode mode : modes) {
    for (int matrixN : matrixSizes) {
      const std::string name = std::string("AccelerateDgemmThreadThreshold/") + accelerateModeName(mode) +
        "/n_" + std::to_string(matrixN);
      benchmark::RegisterBenchmark(name.c_str(), [=](benchmark::State &state) {
        benchmarkAccelerateDgemmThreadThreshold(state, mode, matrixN);
      })->UseRealTime()
        ->Unit(benchmark::kMicrosecond);
    }
  }
}

void registerRepresentativeWorkloadBenchmarks()
{
  constexpr RepresentativeWorkload workloads[] = {
    RepresentativeWorkload::NoBlas,
    RepresentativeWorkload::FemElements,
    RepresentativeWorkload::ContactPairs,
  };
  constexpr Policy policies[] = { Policy::Suppress, Policy::Inherit };
  constexpr int workerCounts[] = { 1, 4, 8 };
  constexpr int taskMultipliers[] = { 1, 4 };

  for (RepresentativeWorkload workload : workloads) {
    const int workItems = workload == RepresentativeWorkload::NoBlas ? 32768 :
                                                                       (workload == RepresentativeWorkload::FemElements ? 4096 : 8192);
    for (Policy policy : policies) {
      for (int outerWorkers : workerCounts) {
        for (int taskMultiplier : taskMultipliers) {
          const int outerTasks = outerWorkers * taskMultiplier;
          const std::string name = std::string("PolicyDecision/") + workloadName(workload) + "/" +
            policyName(policy) + "/workers_" + std::to_string(outerWorkers) +
            "/tasks_" + std::to_string(outerTasks) + "/size_" + std::to_string(workItems);
          benchmark::RegisterBenchmark(name.c_str(), [=](benchmark::State &state) {
            benchmarkRepresentativeWorkload(
              state, workload, policy, outerWorkers, outerTasks, workItems);
          })->UseRealTime()
            ->Unit(benchmark::kMillisecond);
        }
      }
    }
  }
}

const bool registered = [] {
  registerNestedParallelDgemmBenchmarks();
  registerAccelerateDgemmThreadThresholdBenchmarks();
  registerRepresentativeWorkloadBenchmarks();
  return true;
}();

}  // namespace

BENCHMARK_MAIN();
