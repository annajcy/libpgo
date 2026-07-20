#include "../eigen_mkl_common/eigen_mkl_gemm_workload.h"
#include "../benchmark_argument_parser.h"
#include "../parallelism_benchmark_helpers.h"
#include "../workload_warmup.h"

#include "parallel/arenaThreadingExecutor.h"
#include "parallel/parallelControl.h"

#include <mkl.h>

#include <tbb/blocked_range.h>
#include <tbb/global_control.h>
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
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{

namespace P = pgo::parallel;
using pgo::benchmark_helpers::adjustedExtraThreads;
using pgo::benchmark_helpers::EigenMklGemmWorkload;
using pgo::benchmark_helpers::parseNonnegativeInteger;
using pgo::benchmark_helpers::parseNonnegativeDouble;
using pgo::benchmark_helpers::parsePositiveInteger;
using pgo::benchmark_helpers::requireValue;
using pgo::benchmark_helpers::observedMinimum;
using pgo::benchmark_helpers::updateMaximum;
using pgo::benchmark_helpers::updateMinimum;
using pgo::benchmark_helpers::ThreadSampler;

enum class Policy
{
  OuterDefault = 0,
  OuterLocal1 = 1,
  InnerArena1Default = 2,
  InnerArena1Local1 = 3,
};

struct PolicySpec
{
  int outerMklLocalThreadBudget;
  bool usesInnerArena;
  int innerMklLocalThreadBudget;
};

const char *policyName(Policy policy) noexcept
{
  switch (policy) {
  case Policy::OuterDefault:
    return "OuterDefault";
  case Policy::OuterLocal1:
    return "OuterLocal1";
  case Policy::InnerArena1Default:
    return "InnerArena1Default";
  case Policy::InnerArena1Local1:
    return "InnerArena1Local1";
  }
  return "Unknown";
}

Policy parsePolicy(std::string_view value)
{
  constexpr Policy policies[] = {
    Policy::OuterDefault,
    Policy::OuterLocal1,
    Policy::InnerArena1Default,
    Policy::InnerArena1Local1,
  };
  for (Policy policy : policies) {
    if (value == policyName(policy))
      return policy;
  }
  throw std::invalid_argument("Unknown nested-inner-arena policy.");
}

PolicySpec policySpec(Policy policy)
{
  switch (policy) {
  case Policy::OuterDefault:
    return { 0, false, -1 };
  case Policy::OuterLocal1:
    return { 1, false, -1 };
  case Policy::InnerArena1Default:
    return { 0, true, 0 };
  case Policy::InnerArena1Local1:
    return { 0, true, 1 };
  }
  throw std::invalid_argument("Invalid nested-inner-arena policy.");
}

struct Arguments
{
  Policy policy;
  int concurrency;
  int outerTasks;
  int matrixN;
  double warmupSeconds;
  int warmupMinOperations;
  int profileIterations;
};

Arguments parseArguments(int argc, char **argv)
{
  return {
    parsePolicy(requireValue(argc, argv, "--policy=")),
    parsePositiveInteger(
      requireValue(argc, argv, "--concurrency="), "--concurrency"),
    parsePositiveInteger(
      requireValue(argc, argv, "--outer-tasks="), "--outer-tasks"),
    parsePositiveInteger(requireValue(argc, argv, "--matrix-n="), "--matrix-n"),
    parseNonnegativeDouble(requireValue(argc, argv, "--warmup-seconds="),
      "--warmup-seconds"),
    parseNonnegativeInteger(
      requireValue(argc, argv, "--warmup-min-operations="),
      "--warmup-min-operations"),
    parsePositiveInteger(requireValue(argc, argv, "--profile-iterations="),
      "--profile-iterations"),
  };
}

struct RunTelemetry
{
  std::atomic<int> activeOuterCallbacks{ 0 };
  std::atomic<int> peakOuterCallbacks{ 0 };
  std::atomic<int> outerArenaConcurrencyMinimum{ INT_MAX };
  std::atomic<int> outerArenaConcurrencyMaximum{ 0 };
  std::atomic<int> innerArenaConcurrencyMinimum{ INT_MAX };
  std::atomic<int> innerArenaConcurrencyMaximum{ 0 };
  std::atomic<int> mklMaxThreadsMinimum{ INT_MAX };
  std::atomic<int> mklMaxThreadsMaximum{ 0 };
  std::atomic<int> bodyCalls{ 0 };

  void observeOuterArena() noexcept
  {
    const int concurrency = tbb::this_task_arena::max_concurrency();
    updateMinimum(outerArenaConcurrencyMinimum, concurrency);
    updateMaximum(outerArenaConcurrencyMaximum, concurrency);
  }

  void observeMklCall(bool insideInnerArena) noexcept
  {
    if (insideInnerArena) {
      const int concurrency = tbb::this_task_arena::max_concurrency();
      updateMinimum(innerArenaConcurrencyMinimum, concurrency);
      updateMaximum(innerArenaConcurrencyMaximum, concurrency);
    }

    const int mklMaxThreads = mkl_get_max_threads();
    updateMinimum(mklMaxThreadsMinimum, mklMaxThreads);
    updateMaximum(mklMaxThreadsMaximum, mklMaxThreads);
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

using ExecutorList = std::vector<std::unique_ptr<P::ArenaThreadingExecutor>>;

ExecutorList makeInnerExecutors(int outerTasks, const PolicySpec &spec)
{
  ExecutorList executors;
  if (!spec.usesInnerArena)
    return executors;

  executors.reserve(static_cast<std::size_t>(outerTasks));
  for (int taskIndex = 0; taskIndex < outerTasks; ++taskIndex) {
    executors.emplace_back(std::make_unique<P::ArenaThreadingExecutor>(1,
      P::ThreadingPolicy{
        .mklLocalThreadBudget = spec.innerMklLocalThreadBudget,
      }));
  }
  return executors;
}

void runBatch(P::ArenaThreadingExecutor &outerExecutor,
  const PolicySpec &spec, ExecutorList &innerExecutors, int outerTasks,
  EigenMklGemmWorkload &workload, RunTelemetry &telemetry)
{
  outerExecutor.execute([&] {
    tbb::parallel_for(
      tbb::blocked_range<int>(0, outerTasks, 1),
      [&](const tbb::blocked_range<int> &range) {
        for (int taskIndex = range.begin(); taskIndex < range.end(); ++taskIndex) {
          ActiveOuterCallback active(telemetry);
          telemetry.observeOuterArena();

          const auto invokeMkl = [&] {
            telemetry.observeMklCall(spec.usesInnerArena);
            workload.run(taskIndex);
          };
          if (spec.usesInnerArena)
            innerExecutors[static_cast<std::size_t>(taskIndex)]->execute(invokeMkl);
          else
            invokeMkl();

          telemetry.bodyCalls.fetch_add(1, std::memory_order_relaxed);
        }
      },
      tbb::auto_partitioner{});
  });
}

void runIterations(P::ArenaThreadingExecutor &outerExecutor,
  const PolicySpec &spec, ExecutorList &innerExecutors, int outerTasks,
  EigenMklGemmWorkload &workload, int iterations, RunTelemetry &telemetry)
{
  for (int iteration = 0; iteration < iterations; ++iteration) {
    runBatch(outerExecutor, spec, innerExecutors, outerTasks, workload,
      telemetry);
  }
}

void run(const Arguments &arguments)
{
  // With the TBB threading layer, oneMKL's process-global OpenMP thread
  // setters do not control parallelism. Record the process-default value so
  // budget 0 can be interpreted without pretending it equals TBB concurrency.
  const int processDefaultMklMaxThreads = mkl_get_max_threads();

  P::GlobalTbbControl control(arguments.concurrency);
  const int effectiveConcurrency = static_cast<int>(tbb::global_control::active_value(
    tbb::global_control::max_allowed_parallelism));
  const PolicySpec spec = policySpec(arguments.policy);
  P::ArenaThreadingExecutor outerExecutor(arguments.concurrency,
    { .mklLocalThreadBudget = spec.outerMklLocalThreadBudget });
  ExecutorList innerExecutors = makeInnerExecutors(arguments.outerTasks, spec);
  EigenMklGemmWorkload workload(arguments.outerTasks, arguments.matrixN);

  RunTelemetry warmupTelemetry;
  const auto warmup = pgo::benchmark_helpers::runWorkloadWarmup(
    [&] {
      runBatch(outerExecutor, spec, innerExecutors, arguments.outerTasks,
        workload, warmupTelemetry);
    },
    arguments.warmupSeconds, arguments.warmupMinOperations);

  ThreadSampler sampler;
  sampler.start();
  RunTelemetry profileTelemetry;
  const std::clock_t cpuStart = std::clock();
  const auto wallStart = std::chrono::steady_clock::now();

  runIterations(outerExecutor, spec, innerExecutors, arguments.outerTasks,
    workload, arguments.profileIterations, profileTelemetry);

  const auto wallEnd = std::chrono::steady_clock::now();
  const std::clock_t cpuEnd = std::clock();
  const int peakThreads = sampler.stop();
  const int baselineThreads = sampler.baseline();
  const double checksum = workload.checksum();
  if (!std::isfinite(checksum))
    throw std::runtime_error("Eigen/oneMKL GEMM produced a non-finite checksum.");

  const double wallSeconds =
    std::chrono::duration<double>(wallEnd - wallStart).count();
  const double cpuSeconds =
    static_cast<double>(cpuEnd - cpuStart) / static_cast<double>(CLOCKS_PER_SEC);

  std::cout << std::setprecision(17)
            << "PGO_MKL_NESTED_INNER_ARENA_RESULT"
            << " policy=" << policyName(arguments.policy)
            << " configured_global_concurrency=" << arguments.concurrency
            << " effective_global_concurrency=" << effectiveConcurrency
            << " process_default_mkl_max_threads="
            << processDefaultMklMaxThreads
            << " configured_outer_arena_concurrency=" << arguments.concurrency
            << " configured_outer_mkl_local_budget="
            << spec.outerMklLocalThreadBudget
            << " uses_inner_arena=" << (spec.usesInnerArena ? 1 : 0)
            << " configured_inner_arena_concurrency="
            << (spec.usesInnerArena ? 1 : 0)
            << " configured_inner_mkl_local_budget="
            << spec.innerMklLocalThreadBudget
            << " observed_outer_arena_concurrency_min="
            << observedMinimum(profileTelemetry.outerArenaConcurrencyMinimum)
            << " observed_outer_arena_concurrency_max="
            << profileTelemetry.outerArenaConcurrencyMaximum.load(
                 std::memory_order_relaxed)
            << " observed_inner_arena_concurrency_min="
            << observedMinimum(profileTelemetry.innerArenaConcurrencyMinimum)
            << " observed_inner_arena_concurrency_max="
            << profileTelemetry.innerArenaConcurrencyMaximum.load(
                 std::memory_order_relaxed)
            << " observed_mkl_max_threads_min="
            << observedMinimum(profileTelemetry.mklMaxThreadsMinimum)
            << " observed_mkl_max_threads_max="
            << profileTelemetry.mklMaxThreadsMaximum.load(
                 std::memory_order_relaxed)
            << " outer_active_peak="
            << profileTelemetry.peakOuterCallbacks.load(std::memory_order_relaxed)
            << " outer_tasks=" << arguments.outerTasks
            << " matrix_n=" << arguments.matrixN
            << " configured_warmup_seconds=" << arguments.warmupSeconds
            << " configured_warmup_min_operations="
            << arguments.warmupMinOperations
            << " actual_warmup_seconds=" << warmup.elapsedSeconds
            << " actual_warmup_operations=" << warmup.completedOperations
            << " profile_iterations=" << arguments.profileIterations
            << " measured_gemm_calls="
            << profileTelemetry.bodyCalls.load(std::memory_order_relaxed)
            << " process_gemm_calls="
            << (warmup.completedOperations + arguments.profileIterations) *
      arguments.outerTasks
            << " baseline_threads=" << baselineThreads
            << " peak_threads=" << peakThreads
            << " extra_threads="
            << adjustedExtraThreads(baselineThreads, peakThreads)
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
    std::cerr << "eigen_mkl_nested_inner_arena_probe: " << error.what() << '\n';
    return 1;
  }
}
