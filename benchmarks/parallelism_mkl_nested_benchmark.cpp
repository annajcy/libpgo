#include "parallelism/parallelFor.h"
#include "parallelism/parallelOptions.h"
#include "parallelism_benchmark_helpers.h"

#include <benchmark/benchmark.h>
#include <mkl.h>

#include <algorithm>
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

enum class MklMode
{
  LocalOne,
  Default,
};

constexpr int kThresholdWorkerLimit = 16;
constexpr int kRuntimeConcurrency = 16;

P::ParallelRuntime &benchmarkRuntime()
{
  return P::initializeRuntime({ .maxTbbConcurrency = kRuntimeConcurrency });
}

class ScopedMklLocalThreads
{
public:
  explicit ScopedMklLocalThreads(MklMode mode)
  {
    if (mode == MklMode::LocalOne) {
      previousThreads_ = mkl_set_num_threads_local(1);
      active_ = true;
    }
  }

  ~ScopedMklLocalThreads()
  {
    if (active_)
      mkl_set_num_threads_local(previousThreads_);
  }

  ScopedMklLocalThreads(const ScopedMklLocalThreads &) = delete;
  ScopedMklLocalThreads &operator=(const ScopedMklLocalThreads &) = delete;

private:
  int previousThreads_ = 0;
  bool active_ = false;
};

double runMklDgemm(MatrixSet &matrices)
{
  cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
    matrices.n, matrices.n, matrices.n,
    1.0, matrices.a.data(), matrices.n,
    matrices.b.data(), matrices.n,
    0.0, matrices.c.data(), matrices.n);
  return matrices.centerValue();
}

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

const char *mklModeName(MklMode mode)
{
  return mode == MklMode::LocalOne ? "Local1" : "Default";
}

void recordCommonCounters(benchmark::State &state, int arenaConcurrency, int matrixN)
{
  const P::RuntimeInfo runtime = P::runtimeInfo();
  state.counters["runtime_concurrency"] = *runtime.maxConcurrency;
  state.counters["arena_concurrency"] = arenaConcurrency;
  state.counters["matrix_n"] = matrixN;
  state.counters["mkl_max_threads"] = mkl_get_max_threads();
  state.counters["mkl_dynamic"] = mkl_get_dynamic();
  state.counters["tbb_max_allowed_parallelism"] = runtime.effectiveTbbMaxAllowedParallelism;
  state.counters["peak_participants"] = runtime.peakTotalParticipants;
  state.counters["participant_pressure"] = runtime.participantPressureObserved ? 1 : 0;
}

void recordThresholdCounters(benchmark::State &state, int matrixN)
{
  const P::RuntimeInfo runtime = P::runtimeInfo();
  state.counters["thread_limit"] = kThresholdWorkerLimit;
  state.counters["matrix_n"] = matrixN;
  state.counters["mkl_max_threads"] = mkl_get_max_threads();
  state.counters["mkl_dynamic"] = mkl_get_dynamic();
  state.counters["tbb_max_allowed_parallelism"] = runtime.effectiveTbbMaxAllowedParallelism;
}

void benchmarkNestedParallelDgemm(
  benchmark::State &state, Policy policy, int outerWorkers, int outerTasks, int matrixN)
{
  const P::ParallelExecutor executor =
    benchmarkRuntime().createExecutor({ .maxConcurrency = outerWorkers });
  MatrixPool pool(std::max(1, outerWorkers), matrixN);
  std::vector<double> results(static_cast<std::size_t>(outerTasks), 0.0);

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
      results[static_cast<std::size_t>(taskIndex)] = runMklDgemm(pool.current());
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
}

void benchmarkMklDgemmThreadThreshold(
  benchmark::State &state, MklMode mode, int matrixN)
{
  benchmarkRuntime();
  MatrixSet matrices(matrixN);
  ScopedMklLocalThreads localThreads(mode);

  int baselineThreads = currentProcessThreadCount();
  int maxIterationBaselineThreads = baselineThreads;
  int peakThreads = baselineThreads;
  double checksum = 0.0;

  for (auto _ : state) {
    state.PauseTiming();
    ThreadSampler sampler;
    sampler.start();
    state.ResumeTiming();

    checksum += runMklDgemm(matrices);
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
  state.counters["mkl_mode"] = mode == MklMode::LocalOne ? 0 : 1;
}

void registerNestedParallelDgemmBenchmarks()
{
  constexpr int workerCounts[] = { 1, 2, 4, 8, 16 };
  constexpr int matrixSizes[] = { 512, 1024, 2048 };
  constexpr int taskMultipliers[] = { 1, 4 };
  constexpr Policy policies[] = { Policy::Suppress, Policy::Inherit };

  for (Policy policy : policies) {
    for (int outerWorkers : workerCounts) {
      for (int taskMultiplier : taskMultipliers) {
        for (int matrixN : matrixSizes) {
          const int outerTasks = outerWorkers * taskMultiplier;
          const std::string name = std::string("NestedParallelDgemm/") + policyName(policy) +
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

void registerMklDgemmThreadThresholdBenchmarks()
{
  constexpr int matrixSizes[] = { 64, 96, 128, 192, 256, 384, 512, 768, 1024, 1536, 2048 };
  constexpr MklMode modes[] = { MklMode::LocalOne, MklMode::Default };

  for (MklMode mode : modes) {
    for (int matrixN : matrixSizes) {
      const std::string name = std::string("MklDgemmThreadThreshold/") + mklModeName(mode) +
        "/n_" + std::to_string(matrixN);
      benchmark::RegisterBenchmark(name.c_str(), [=](benchmark::State &state) {
        benchmarkMklDgemmThreadThreshold(state, mode, matrixN);
      })->UseRealTime()
        ->Unit(benchmark::kMicrosecond);
    }
  }
}

const bool registered = [] {
  registerNestedParallelDgemmBenchmarks();
  registerMklDgemmThreadThresholdBenchmarks();
  return true;
}();

}  // namespace

BENCHMARK_MAIN();
