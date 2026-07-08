#include "parallelism/parallelFor.h"
#include "parallelism/parallelOptions.h"
#include "parallelism_benchmark_helpers.h"

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
using pgo::benchmark_helpers::ThreadSampler;
using pgo::benchmark_helpers::adjustedExtraThreads;
using pgo::benchmark_helpers::currentProcessThreadCount;

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

class ScopedAccelerateThreading
{
public:
  explicit ScopedAccelerateThreading(AccelerateMode mode)
    : previousThreading_(BLASGetThreading())
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
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#endif
  cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
    matrices.n, matrices.n, matrices.n,
    1.0, matrices.a.data(), matrices.n,
    matrices.b.data(), matrices.n,
    0.0, matrices.c.data(), matrices.n);
#ifdef __clang__
#pragma clang diagnostic pop
#endif
  return matrices.centerValue();
}

void recordCommonCounters(benchmark::State &state, int workerLimit, int matrixN)
{
  const P::RuntimeInfo runtime = P::runtimeInfo();
  state.counters["worker_limit"] = workerLimit;
  state.counters["matrix_n"] = matrixN;
  state.counters["accelerate_threading"] = static_cast<int>(BLASGetThreading());
  if (runtime.tbbMaxAllowedParallelism.has_value())
    state.counters["tbb_max_allowed_parallelism"] = *runtime.tbbMaxAllowedParallelism;
}

void benchmarkNestedParallelDgemm(
  benchmark::State &state, Policy policy, int outerWorkers, int outerTasks, int matrixN)
{
  ScopedAccelerateThreading inheritedThreading(AccelerateMode::Multi);
  P::ScopedWorkerLimit workerLimit(outerWorkers);
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

    P::parallelFor(0, outerTasks, options, [&](int taskIndex) {
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
}

void benchmarkAccelerateDgemmThreadThreshold(
  benchmark::State &state, AccelerateMode mode, int workerLimitValue, int matrixN)
{
  P::ScopedWorkerLimit workerLimit(workerLimitValue);
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

  recordCommonCounters(state, workerLimitValue, matrixN);
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
            })
            ->UseRealTime()
            ->Unit(benchmark::kMillisecond);
        }
      }
    }
  }
}

void registerAccelerateDgemmThreadThresholdBenchmarks()
{
  constexpr int workerLimits[] = { 4, 8, 16 };
  constexpr int matrixSizes[] = { 32, 64, 96, 128, 192, 256, 384, 512, 768, 1024, 1536, 2048 };
  constexpr AccelerateMode modes[] = { AccelerateMode::Single, AccelerateMode::Multi };

  for (AccelerateMode mode : modes) {
    for (int workerLimit : workerLimits) {
      for (int matrixN : matrixSizes) {
        const std::string name = std::string("AccelerateDgemmThreadThreshold/") + accelerateModeName(mode) +
          "/workers_" + std::to_string(workerLimit) +
          "/n_" + std::to_string(matrixN);
        benchmark::RegisterBenchmark(name.c_str(), [=](benchmark::State &state) {
            benchmarkAccelerateDgemmThreadThreshold(state, mode, workerLimit, matrixN);
          })
          ->UseRealTime()
          ->Unit(benchmark::kMicrosecond);
      }
    }
  }
}

const bool registered = [] {
  registerNestedParallelDgemmBenchmarks();
  registerAccelerateDgemmThreadThresholdBenchmarks();
  return true;
}();

}  // namespace

BENCHMARK_MAIN();
