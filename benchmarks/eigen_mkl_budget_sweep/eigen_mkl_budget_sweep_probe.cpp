#include "../eigen_mkl_common/eigen_mkl_gemm_workload.h"
#include "../benchmark_argument_parser.h"
#include "../parallelism_benchmark_helpers.h"

#include "parallel/arenaThreadingExecutor.h"
#include "parallel/parallelControl.h"

#include <mkl.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/partitioner.h>
#include <tbb/task_arena.h>

#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace
{

namespace P = pgo::parallel;
using pgo::benchmark_helpers::EigenMklGemmWorkload;
using pgo::benchmark_helpers::parseNonnegativeInteger;
using pgo::benchmark_helpers::parsePositiveInteger;
using pgo::benchmark_helpers::requireValue;
using pgo::benchmark_helpers::updateMaximum;
using pgo::benchmark_helpers::updateMinimum;

struct Arguments
{
  int concurrency;
  int arenaConcurrency;
  int mklLocalThreadBudget;
  int outerTasks;
  int matrixN;
  int warmupIterations;
  int profileIterations;
};

Arguments parseArguments(int argc, char **argv)
{
  return {
    parsePositiveInteger(
      requireValue(argc, argv, "--concurrency="), "--concurrency"),
    parsePositiveInteger(requireValue(argc, argv, "--arena-concurrency="),
      "--arena-concurrency"),
    parseNonnegativeInteger(
      requireValue(argc, argv, "--mkl-local-thread-budget="),
      "--mkl-local-thread-budget"),
    parsePositiveInteger(
      requireValue(argc, argv, "--outer-tasks="), "--outer-tasks"),
    parsePositiveInteger(
      requireValue(argc, argv, "--matrix-n="), "--matrix-n"),
    parseNonnegativeInteger(requireValue(argc, argv, "--warmup-iterations="),
      "--warmup-iterations"),
    parsePositiveInteger(requireValue(argc, argv, "--profile-iterations="),
      "--profile-iterations"),
  };
}

struct RunTelemetry
{
  std::atomic<int> observedArenaConcurrency{ 0 };
  std::atomic<int> observedMklMaxThreadsMinimum{ INT_MAX };
  std::atomic<int> observedMklMaxThreadsMaximum{ 0 };
  std::atomic<int> bodyCalls{ 0 };

  void observe() noexcept
  {
    updateMaximum(observedArenaConcurrency,
      tbb::this_task_arena::max_concurrency());
    const int mklMaxThreads = mkl_get_max_threads();
    updateMinimum(observedMklMaxThreadsMinimum, mklMaxThreads);
    updateMaximum(observedMklMaxThreadsMaximum, mklMaxThreads);
  }
};

void runBatch(P::ArenaThreadingExecutor &executor, int outerTasks,
  EigenMklGemmWorkload &workload, RunTelemetry &telemetry)
{
  executor.execute([&] {
    if (outerTasks == 1) {
      telemetry.observe();
      workload.run(0);
      telemetry.bodyCalls.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    tbb::parallel_for(
      tbb::blocked_range<int>(0, outerTasks, 1),
      [&](const tbb::blocked_range<int> &range) {
        for (int taskIndex = range.begin(); taskIndex < range.end(); ++taskIndex) {
          telemetry.observe();
          workload.run(taskIndex);
          telemetry.bodyCalls.fetch_add(1, std::memory_order_relaxed);
        }
      },
      tbb::auto_partitioner{});
  });
}

void runIterations(P::ArenaThreadingExecutor &executor, int outerTasks,
  EigenMklGemmWorkload &workload, int iterations, RunTelemetry &telemetry)
{
  for (int iteration = 0; iteration < iterations; ++iteration)
    runBatch(executor, outerTasks, workload, telemetry);
}

void run(const Arguments &arguments)
{
  P::GlobalTbbControl control(arguments.concurrency);
  const int effectiveConcurrency = static_cast<int>(tbb::global_control::active_value(
    tbb::global_control::max_allowed_parallelism));
  P::ArenaThreadingExecutor executor(arguments.arenaConcurrency,
    { .mklLocalThreadBudget = arguments.mklLocalThreadBudget });
  EigenMklGemmWorkload workload(arguments.outerTasks, arguments.matrixN);

  RunTelemetry warmupTelemetry;
  runIterations(executor, arguments.outerTasks, workload,
    arguments.warmupIterations, warmupTelemetry);

  RunTelemetry profileTelemetry;
  const std::clock_t cpuStart = std::clock();
  const auto wallStart = std::chrono::steady_clock::now();

  runIterations(executor, arguments.outerTasks, workload,
    arguments.profileIterations, profileTelemetry);

  const auto wallEnd = std::chrono::steady_clock::now();
  const std::clock_t cpuEnd = std::clock();
  const double checksum = workload.checksum();
  if (!std::isfinite(checksum))
    throw std::runtime_error("Eigen/oneMKL GEMM produced a non-finite checksum.");

  const double wallSeconds =
    std::chrono::duration<double>(wallEnd - wallStart).count();
  const double cpuSeconds =
    static_cast<double>(cpuEnd - cpuStart) / static_cast<double>(CLOCKS_PER_SEC);
  const int observedMklMinimum =
    profileTelemetry.observedMklMaxThreadsMinimum.load(std::memory_order_relaxed);

  std::cout << std::setprecision(17)
            << "PGO_MKL_BUDGET_SWEEP_RESULT"
            << " configured_global_concurrency=" << arguments.concurrency
            << " effective_global_concurrency=" << effectiveConcurrency
            << " configured_arena_concurrency=" << arguments.arenaConcurrency
            << " configured_mkl_local_budget=" << arguments.mklLocalThreadBudget
            << " observed_arena_concurrency="
            << profileTelemetry.observedArenaConcurrency.load(std::memory_order_relaxed)
            << " observed_mkl_max_threads_min="
            << (observedMklMinimum == INT_MAX ? 0 : observedMklMinimum)
            << " observed_mkl_max_threads_max="
            << profileTelemetry.observedMklMaxThreadsMaximum.load(
                 std::memory_order_relaxed)
            << " outer_tasks=" << arguments.outerTasks
            << " matrix_n=" << arguments.matrixN
            << " warmup_iterations=" << arguments.warmupIterations
            << " profile_iterations=" << arguments.profileIterations
            << " measured_gemm_calls="
            << profileTelemetry.bodyCalls.load(std::memory_order_relaxed)
            << " process_gemm_calls="
            << (arguments.warmupIterations + arguments.profileIterations) *
      arguments.outerTasks
            << " wall_seconds=" << wallSeconds
            << " process_cpu_seconds=" << cpuSeconds
            << " checksum=" << checksum << '\n';
}

}  // namespace

int main(int argc, char **argv)
{
  try {
    run(parseArguments(argc, argv));
    return 0;
  }
  catch (const std::exception &error) {
    std::cerr << "eigen_mkl_budget_sweep_probe: " << error.what() << '\n';
    return 1;
  }
}
