#include "../eigen_mkl_common/eigen_mkl_gemm_workload.h"

#include "../benchmark_argument_parser.h"
#include "../eigen_mkl_common/eigen_mkl_executor_cases.h"
#include "../parallelism_benchmark_helpers.h"
#include "../timed_workload.h"
#include "../workload_warmup.h"
#include "parallel/arenaThreadingExecutor.h"
#include "parallel/parallelControl.h"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/partitioner.h>
#include <tbb/task_arena.h>

#include <atomic>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace
{

namespace P = pgo::parallel;
using pgo::benchmark_helpers::adjustedExtraThreads;
using pgo::benchmark_helpers::allMklExecutorCases;
using pgo::benchmark_helpers::EigenMklGemmWorkload;
using pgo::benchmark_helpers::MklExecutorCase;
using pgo::benchmark_helpers::mklExecutorCaseName;
using pgo::benchmark_helpers::mklExecutorSpec;
using pgo::benchmark_helpers::parseMklExecutorCase;
using pgo::benchmark_helpers::parseNonnegativeDouble;
using pgo::benchmark_helpers::parseNonnegativeInteger;
using pgo::benchmark_helpers::parsePositiveInteger;
using pgo::benchmark_helpers::parseTimedMeasurementArguments;
using pgo::benchmark_helpers::requireValue;
using pgo::benchmark_helpers::runTimedMeasurement;
using pgo::benchmark_helpers::runWorkloadWarmup;
using pgo::benchmark_helpers::ThreadSampler;
using pgo::benchmark_helpers::updateMaximum;
using pgo::benchmark_helpers::TimedMeasurementArguments;

enum class WorkloadKind
{
  EigenMklGemm = 0,
};

const char *workloadName(WorkloadKind workload) noexcept
{
  switch (workload) {
  case WorkloadKind::EigenMklGemm:
    return "EigenMklGemm";
  }
  return "Unknown";
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

struct Arguments
{
  MklExecutorCase policy;
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
    parseMklExecutorCase(requireValue(argc, argv, "--policy=")),
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

template<class Workload>
void runWorkloadBenchmark(
  WorkloadKind workloadKind, const Arguments &arguments)
{
  P::GlobalTbbControl control(arguments.concurrency);
  const int effectiveConcurrency = static_cast<int>(tbb::global_control::active_value(
    tbb::global_control::max_allowed_parallelism));
  const auto spec = mklExecutorSpec(arguments.policy, arguments.concurrency);
  P::ArenaThreadingExecutor executor(spec.arenaConcurrency,
    { .mklLocalThreadBudget = spec.mklLocalThreadBudget });
  Workload workload(arguments.outerTasks, arguments.matrixN);
  RunTelemetry telemetry;

  ThreadSampler sampler;
  sampler.start();
  const auto warmup = runWorkloadWarmup(
    [&] {
      runOuterLoop(executor, arguments.outerTasks, workload, telemetry);
    },
    arguments.warmupSeconds, arguments.warmupMinOperations);
  const int peakThreads = sampler.stop();
  const int baselineThreads = sampler.baseline();

  telemetry.reset();
  const auto measurement = runTimedMeasurement(
    [&] {
      runOuterLoop(executor, arguments.outerTasks, workload, telemetry);
    },
    arguments.measurement);

  const double checksum = workload.checksum();
  if (!std::isfinite(checksum))
    throw std::runtime_error(
      "MKL/TBB control-matrix workload produced a non-finite checksum.");

  const double matrixN = static_cast<double>(arguments.matrixN);
  const double flopsPerIteration =
    2.0 * matrixN * matrixN * matrixN * static_cast<double>(arguments.outerTasks);
  std::cout << std::setprecision(17)
            << "PGO_EIGEN_MKL_CONTROL_MATRIX_RESULT"
            << " policy=" << mklExecutorCaseName(arguments.policy)
            << " workload=" << workloadName(workloadKind)
            << " uses_blas=1"
            << " configured_global_concurrency=" << arguments.concurrency
            << " effective_global_concurrency=" << effectiveConcurrency
            << " configured_arena_concurrency=" << spec.arenaConcurrency
            << " configured_mkl_local_budget=" << spec.mklLocalThreadBudget
            << " outer_tasks=" << arguments.outerTasks
            << " matrix_n=" << arguments.matrixN
            << " observed_arena_concurrency="
            << telemetry.observedArenaConcurrency.load(std::memory_order_relaxed)
            << " outer_active_peak="
            << telemetry.peakOuterCallbacks.load(std::memory_order_relaxed)
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

void runBenchmark(const Arguments &arguments)
{
  runWorkloadBenchmark<EigenMklGemmWorkload>(
    WorkloadKind::EigenMklGemm, arguments);
}

}  // namespace

int main(int argc, char **argv)
{
  try {
    runBenchmark(parseArguments(argc, argv));
    return 0;
  }
  catch (const std::exception &error) {
    std::cerr << "eigen_mkl_control_matrix_benchmark: " << error.what() << '\n';
    return 1;
  }
}
