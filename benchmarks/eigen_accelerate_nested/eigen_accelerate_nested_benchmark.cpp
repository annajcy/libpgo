#include "eigen_accelerate_nested_kernel.h"

#include "../benchmark_argument_parser.h"
#include "../parallelism_benchmark_helpers.h"
#include "../timed_workload.h"
#include "../workload_warmup.h"
#include "parallel/parallel.h"

#include <Accelerate/Accelerate.h>

#include <tbb/blocked_range.h>
#include <tbb/info.h>
#include <tbb/parallel_for.h>
#include <tbb/partitioner.h>
#include <tbb/task_arena.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace
{

namespace P = pgo::parallel;
using pgo::benchmark_helpers::adjustedExtraThreads;
using pgo::benchmark_helpers::NestedEigenAccelerateWorkload;
using pgo::benchmark_helpers::parseNonnegativeDouble;
using pgo::benchmark_helpers::parseNonnegativeInteger;
using pgo::benchmark_helpers::parsePositiveInteger;
using pgo::benchmark_helpers::parseTimedMeasurementArguments;
using pgo::benchmark_helpers::requireValue;
using pgo::benchmark_helpers::runTimedMeasurement;
using pgo::benchmark_helpers::runWorkloadWarmup;
using pgo::benchmark_helpers::ThreadSampler;
using pgo::benchmark_helpers::TimedMeasurementArguments;
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

Policy parsePolicy(std::string_view value)
{
  if (value == "ExecutorSingle")
    return Policy::ExecutorSingle;
  if (value == "ExecutorMulti")
    return Policy::ExecutorMulti;
  throw std::invalid_argument("Unsupported --policy value: " + std::string(value));
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

struct Arguments
{
  Policy policy;
  int concurrency;
  int outerTasks;
  int matrixN;
  double warmupSeconds;
  int warmupMinOperations;
  TimedMeasurementArguments measurement;
};

Arguments parseArguments(int argc, char **argv)
{
  return {
    parsePolicy(requireValue(argc, argv, "--policy=")),
    parsePositiveInteger(requireValue(argc, argv, "--concurrency="),
      "--concurrency"),
    parsePositiveInteger(
      requireValue(argc, argv, "--outer-tasks="), "--outer-tasks"),
    parsePositiveInteger(requireValue(argc, argv, "--matrix-n="), "--matrix-n"),
    parseNonnegativeDouble(
      requireValue(argc, argv, "--warmup-seconds="), "--warmup-seconds"),
    parseNonnegativeInteger(requireValue(argc, argv, "--warmup-min-operations="),
      "--warmup-min-operations"),
    parseTimedMeasurementArguments(argc, argv),
  };
}

void runBenchmark(const Arguments &arguments)
{
  P::GlobalTbbControl control(arguments.concurrency);
  const int effectiveConcurrency = static_cast<int>(tbb::global_control::active_value(
    tbb::global_control::max_allowed_parallelism));
  P::ArenaThreadingExecutor singleExecutor(arguments.concurrency,
    { .accelerate = P::AccelerateThreading::single });
  P::ArenaThreadingExecutor multiExecutor(arguments.concurrency,
    { .accelerate = P::AccelerateThreading::multi });
  NestedEigenAccelerateWorkload workload(arguments.outerTasks, arguments.matrixN);
  RunTelemetry telemetry;

  ThreadSampler sampler;
  sampler.start();
  const auto warmup = runWorkloadWarmup(
    [&] {
      runOuterLoop(arguments.policy, arguments.outerTasks, workload, telemetry,
        singleExecutor, multiExecutor);
    },
    arguments.warmupSeconds, arguments.warmupMinOperations);
  const int peakThreads = sampler.stop();
  const int baselineThreads = sampler.baseline();

  telemetry.reset();
  const auto measurement = runTimedMeasurement(
    [&] {
      runOuterLoop(arguments.policy, arguments.outerTasks, workload, telemetry,
        singleExecutor, multiExecutor);
    },
    arguments.measurement);

  const double checksum = workload.checksum();
  if (!std::isfinite(checksum))
    throw std::runtime_error(
      "Nested Eigen/Accelerate GEMM produced a non-finite checksum.");

  const double matrixN = static_cast<double>(arguments.matrixN);
  const double flopsPerIteration =
    2.0 * matrixN * matrixN * matrixN * static_cast<double>(arguments.outerTasks);
  std::cout << std::setprecision(17)
            << "PGO_EIGEN_ACCELERATE_NESTED_RESULT"
            << " policy=" << policyName(arguments.policy)
            << " configured_concurrency=" << arguments.concurrency
            << " effective_concurrency=" << effectiveConcurrency
            << " outer_tasks=" << arguments.outerTasks
            << " matrix_n=" << arguments.matrixN
            << " arena_concurrency="
            << telemetry.arenaConcurrency.load(std::memory_order_relaxed)
            << " outer_active_peak="
            << telemetry.peakCallbacks.load(std::memory_order_relaxed)
            << " single_mode_calls="
            << telemetry.singleModeCalls.load(std::memory_order_relaxed)
            << " multi_mode_calls="
            << telemetry.multiModeCalls.load(std::memory_order_relaxed)
            << " other_mode_calls="
            << telemetry.otherModeCalls.load(std::memory_order_relaxed)
            << " body_calls="
            << telemetry.bodyCalls.load(std::memory_order_relaxed)
            << " baseline_threads=" << baselineThreads
            << " peak_threads=" << peakThreads
            << " extra_threads="
            << adjustedExtraThreads(baselineThreads, peakThreads)
            << " configured_warmup_seconds=" << arguments.warmupSeconds
            << " configured_warmup_min_operations="
            << arguments.warmupMinOperations
            << " actual_warmup_seconds=" << warmup.elapsedSeconds
            << " actual_warmup_operations=" << warmup.completedOperations
            << " configured_measurement_min_seconds="
            << arguments.measurement.minimumSeconds
            << " measurement_operations=" << measurement.completedOperations
            << " measurement_wall_seconds=" << measurement.elapsedSeconds
            << " checksum=" << checksum
            << " flops_per_operation=" << flopsPerIteration << '\n';
}

}  // namespace

int main(int argc, char **argv)
{
  try {
    runBenchmark(parseArguments(argc, argv));
    return 0;
  }
  catch (const std::exception &error) {
    std::cerr << "eigen_accelerate_nested_benchmark: " << error.what() << '\n';
    return 1;
  }
}
