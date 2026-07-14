#include "parallel/parallelFor.h"
#include "parallel/parallelControl.h"
#include "parallelism_benchmark_helpers.h"
#include "parallelism_policy_workloads.h"

#include <benchmark/benchmark.h>
#include <mkl.h>

#include <algorithm>
#include <atomic>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace
{

namespace P = pgo::parallel;
using pgo::benchmark_helpers::MatrixSet;
using pgo::benchmark_helpers::CurrentArenaThreadObserver;
using pgo::benchmark_helpers::RepresentativeWorkload;
using pgo::benchmark_helpers::ThreadSampler;
using pgo::benchmark_helpers::adjustedExtraThreads;
using pgo::benchmark_helpers::currentProcessThreadCount;
using pgo::benchmark_helpers::runRepresentativeWorkload;
using pgo::benchmark_helpers::workloadName;
using pgo::benchmark_helpers::updateMaximum;

enum class Policy
{
  Bounded1,
  Multi,
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

double runMklDgemm(const MatrixSet &matrices, std::vector<double> &output)
{
  cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
    matrices.n, matrices.n, matrices.n,
    1.0, matrices.a.data(), matrices.n,
    matrices.b.data(), matrices.n,
    0.0, output.data(), matrices.n);
  return output[static_cast<std::size_t>(matrices.n / 2) * static_cast<std::size_t>(matrices.n) +
    static_cast<std::size_t>(matrices.n / 2)];
}

template<class Fn>
decltype(auto) runWithPolicy(Policy policy, Fn &&fn)
{
  if (policy == Policy::Bounded1)
    return P::withSingleThreadedTbb(std::forward<Fn>(fn));
  return fn();
}

const char *policyName(Policy policy)
{
  switch (policy) {
  case Policy::Bounded1:
    return "Bounded1";
  case Policy::Multi:
    return "Multi";
  }
  return "Unknown";
}

int policyCode(Policy policy)
{
  return static_cast<int>(policy);
}

void recordCommonCounters(benchmark::State &state, int arenaConcurrency, int matrixN)
{
  state.counters["runtime_concurrency"] = arenaConcurrency;
  state.counters["arena_concurrency"] = arenaConcurrency;
  state.counters["matrix_n"] = matrixN;
  state.counters["tbb_max_allowed_parallelism"] = arenaConcurrency;
}

void benchmarkRepresentativeWorkload(benchmark::State &state, RepresentativeWorkload workload,
  Policy policy, int outerWorkers, int outerTasks, int workItems)
{
  const int effectiveConcurrency = P::setMaxConcurrency(outerWorkers);
  std::vector<double> results(static_cast<std::size_t>(outerTasks), 0.0);
  std::atomic<int> observedArenaConcurrency = -1;
  std::atomic<int> observedPeakArenaThreads = 1;
  const std::optional<int> grainSize{ 1 };

  const int baselineThreads = currentProcessThreadCount();
  int peakThreads = baselineThreads;
  for (auto _ : state) {
    state.PauseTiming();
    ThreadSampler sampler;
    sampler.start();
    state.ResumeTiming();

    P::parallelFor(0, outerTasks, [&](int taskIndex) { runWithPolicy(policy, [&] {
                                                         CurrentArenaThreadObserver observer;
                                                         observedArenaConcurrency.store(
                                                           tbb::this_task_arena::max_concurrency(), std::memory_order_relaxed);
                                                         results[static_cast<std::size_t>(taskIndex)] =
                                                           runRepresentativeWorkload(workload, taskIndex, workItems);
                                                         updateMaximum(observedPeakArenaThreads, observer.peak());
                                                       }); }, grainSize);
    benchmark::DoNotOptimize(results.data());

    state.PauseTiming();
    peakThreads = std::max(peakThreads, sampler.stop());
    state.ResumeTiming();
  }

  const double checksum = std::accumulate(results.begin(), results.end(), 0.0);
  benchmark::DoNotOptimize(&checksum);
  recordCommonCounters(state, effectiveConcurrency, 0);
  state.counters["outer_tasks"] = outerTasks;
  state.counters["work_items_per_task"] = workItems;
  state.counters["baseline_threads"] = baselineThreads;
  state.counters["peak_threads"] = peakThreads;
  state.counters["extra_threads"] = adjustedExtraThreads(baselineThreads, peakThreads);
  state.counters["policy"] = policyCode(policy);
  state.counters["worker_arena_concurrency"] = observedArenaConcurrency.load(std::memory_order_relaxed);
  state.counters["observed_peak_arena_threads"] = observedPeakArenaThreads.load(std::memory_order_relaxed);
  state.counters["checksum"] = checksum;
}

void benchmarkNestedParallelDgemm(
  benchmark::State &state, Policy policy, int outerWorkers, int outerTasks, int matrixN)
{
  const int effectiveConcurrency = P::setMaxConcurrency(outerWorkers);
  MatrixSet matrices(matrixN);
  const std::size_t matrixEntries =
    static_cast<std::size_t>(matrixN) * static_cast<std::size_t>(matrixN);
  // A nested TBB kernel may re-enter outer work while its caller is suspended. Indexing a
  // workspace by the current arena worker can therefore let two live DGEMMs share C. Give every
  // logical outer task its own output while sharing the read-only A/B inputs.
  std::vector<std::vector<double>> taskOutputs(
    static_cast<std::size_t>(outerTasks), std::vector<double>(matrixEntries, 0.0));
  std::vector<double> results(static_cast<std::size_t>(outerTasks), 0.0);
  std::atomic<int> observedArenaConcurrency = -1;
  std::atomic<int> observedPeakArenaThreads = 1;

  int baselineThreads = currentProcessThreadCount();
  int maxIterationBaselineThreads = baselineThreads;
  int peakThreads = baselineThreads;

  const std::optional<int> grainSize{ 1 };

  for (auto _ : state) {
    state.PauseTiming();
    ThreadSampler sampler;
    sampler.start();
    state.ResumeTiming();

    P::parallelFor(0, outerTasks, [&](int taskIndex) { runWithPolicy(policy, [&] {
                                                         CurrentArenaThreadObserver observer;
                                                         observedArenaConcurrency.store(
                                                           tbb::this_task_arena::max_concurrency(), std::memory_order_relaxed);
                                                         results[static_cast<std::size_t>(taskIndex)] =
                                                           runMklDgemm(matrices, taskOutputs[static_cast<std::size_t>(taskIndex)]);
                                                         updateMaximum(observedPeakArenaThreads, observer.peak());
                                                       }); }, grainSize);

    benchmark::DoNotOptimize(results.data());

    state.PauseTiming();
    peakThreads = std::max(peakThreads, sampler.stop());
    maxIterationBaselineThreads = std::max(maxIterationBaselineThreads, sampler.baseline());
    state.ResumeTiming();
  }

  const double checksum = std::accumulate(results.begin(), results.end(), 0.0);

  recordCommonCounters(state, effectiveConcurrency, matrixN);
  state.counters["outer_tasks"] = outerTasks;
  state.counters["baseline_threads"] = baselineThreads;
  state.counters["max_iteration_baseline_threads"] = maxIterationBaselineThreads;
  state.counters["peak_threads"] = peakThreads;
  state.counters["raw_extra_threads"] = peakThreads - baselineThreads;
  state.counters["extra_threads"] = adjustedExtraThreads(baselineThreads, peakThreads);
  state.counters["policy"] = policyCode(policy);
  state.counters["worker_arena_concurrency"] = observedArenaConcurrency.load(std::memory_order_relaxed);
  state.counters["observed_peak_arena_threads"] = observedPeakArenaThreads.load(std::memory_order_relaxed);
  state.counters["checksum"] = checksum;
}

void registerNestedParallelDgemmBenchmarks()
{
  constexpr int workerCounts[] = { 1, 2, 4, 8, 16 };
  constexpr int matrixSizes[] = { 512, 1024, 2048 };
  constexpr int taskMultipliers[] = { 1, 4 };
  constexpr Policy policies[] = { Policy::Bounded1, Policy::Multi };

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

void registerRepresentativeWorkloadBenchmarks()
{
  constexpr RepresentativeWorkload workloads[] = {
    RepresentativeWorkload::NoBlas,
    RepresentativeWorkload::FemElements,
    RepresentativeWorkload::ContactPairs,
  };
  constexpr Policy policies[] = { Policy::Bounded1, Policy::Multi };
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

void registerMklTbbBoundDecisionBenchmarks()
{
  constexpr Policy policies[] = { Policy::Bounded1, Policy::Multi };
  constexpr int workerCounts[] = { 1, 4, 8, 16 };
  constexpr int matrixSizes[] = { 512, 1024 };

  for (int outerWorkers : workerCounts) {
    const std::set<int> taskCounts = {
      1,
      std::max(1, outerWorkers / 2),
      outerWorkers,
      outerWorkers * 4,
    };
    for (int outerTasks : taskCounts) {
      for (Policy policy : policies) {
        for (int matrixN : matrixSizes) {
          const std::string name = std::string("MklTbbBoundDecision/Dgemm/") +
            policyName(policy) + "/workers_" + std::to_string(outerWorkers) +
            "/tasks_" + std::to_string(outerTasks) + "/n_" + std::to_string(matrixN);
          benchmark::RegisterBenchmark(name.c_str(), [=](benchmark::State &state) {
            benchmarkNestedParallelDgemm(state, policy, outerWorkers, outerTasks, matrixN);
          })->UseRealTime()
            ->Unit(benchmark::kMillisecond);
        }

        const std::string noBlasName = std::string("MklTbbBoundDecision/NoBlasControl/") +
          policyName(policy) + "/workers_" + std::to_string(outerWorkers) +
          "/tasks_" + std::to_string(outerTasks) + "/size_32768";
        benchmark::RegisterBenchmark(noBlasName.c_str(), [=](benchmark::State &state) {
          benchmarkRepresentativeWorkload(
            state, RepresentativeWorkload::NoBlas, policy, outerWorkers, outerTasks, 32768);
        })->UseRealTime()
          ->Unit(benchmark::kMillisecond);
      }
    }
  }
}

const bool registered = [] {
  registerNestedParallelDgemmBenchmarks();
  registerRepresentativeWorkloadBenchmarks();
  registerMklTbbBoundDecisionBenchmarks();
  return true;
}();

}  // namespace

BENCHMARK_MAIN();
