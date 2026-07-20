#include "../eigen_mkl_common/eigen_mkl_executor_cases.h"
#include "../eigen_mkl_common/eigen_mkl_gemm_workload.h"
#include "../benchmark_argument_parser.h"
#include "../workload_warmup.h"

#include "parallel/arenaThreadingExecutor.h"
#include "parallel/parallelControl.h"

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
using pgo::benchmark_helpers::MklExecutorCase;
using pgo::benchmark_helpers::mklExecutorCaseName;
using pgo::benchmark_helpers::mklExecutorSpec;
using pgo::benchmark_helpers::parseMklExecutorCase;
using pgo::benchmark_helpers::parseNonnegativeInteger;
using pgo::benchmark_helpers::parseNonnegativeDouble;
using pgo::benchmark_helpers::parsePositiveInteger;
using pgo::benchmark_helpers::requireValue;

struct Arguments
{
  MklExecutorCase policy;
  int concurrency;
  int matrixN;
  double warmupSeconds;
  int warmupMinOperations;
  int profileIterations;
};

Arguments parseArguments(int argc, char **argv)
{
  return {
    parseMklExecutorCase(requireValue(argc, argv, "--policy=")),
    parsePositiveInteger(
      requireValue(argc, argv, "--concurrency="), "--concurrency"),
    parsePositiveInteger(requireValue(argc, argv, "--matrix-n="), "--matrix-n"),
    parseNonnegativeDouble(
      requireValue(argc, argv, "--warmup-seconds="), "--warmup-seconds"),
    parseNonnegativeInteger(
      requireValue(argc, argv, "--warmup-min-operations="),
      "--warmup-min-operations"),
    parsePositiveInteger(
      requireValue(argc, argv, "--profile-iterations="), "--profile-iterations"),
  };
}

void runIterations(EigenMklGemmWorkload &workload, int iterations)
{
  for (int iteration = 0; iteration < iterations; ++iteration)
    workload.run(0);
}

void run(const Arguments &arguments)
{
  P::GlobalTbbControl control(arguments.concurrency);
  const int effectiveConcurrency = static_cast<int>(tbb::global_control::active_value(
    tbb::global_control::max_allowed_parallelism));
  const auto spec = mklExecutorSpec(arguments.policy, arguments.concurrency);
  P::ArenaThreadingExecutor executor(spec.arenaConcurrency,
    { .mklLocalThreadBudget = spec.mklLocalThreadBudget });
  EigenMklGemmWorkload workload(1, arguments.matrixN);

  const auto warmup = executor.execute([&] {
    return pgo::benchmark_helpers::runWorkloadWarmup(
      [&] { workload.run(0); }, arguments.warmupSeconds,
      arguments.warmupMinOperations);
  });

  const std::clock_t cpuStart = std::clock();
  const auto wallStart = std::chrono::steady_clock::now();

  std::cout << "PGO_MKL_TASK_PROFILE_BEGIN"
            << " policy=" << mklExecutorCaseName(arguments.policy)
            << " configured_global_concurrency=" << arguments.concurrency
            << " effective_global_concurrency=" << effectiveConcurrency
            << " configured_arena_concurrency=" << spec.arenaConcurrency
            << " configured_mkl_local_budget=" << spec.mklLocalThreadBudget
            << " matrix_n=" << arguments.matrixN
            << " configured_warmup_seconds=" << arguments.warmupSeconds
            << " configured_warmup_min_operations="
            << arguments.warmupMinOperations
            << " actual_warmup_seconds=" << warmup.elapsedSeconds
            << " actual_warmup_operations=" << warmup.completedOperations
            << " iterations=" << arguments.profileIterations << std::endl;

  executor.execute([&] { runIterations(workload, arguments.profileIterations); });

  const auto wallEnd = std::chrono::steady_clock::now();
  const std::clock_t cpuEnd = std::clock();
  const double checksum = workload.checksum();
  if (!std::isfinite(checksum))
    throw std::runtime_error("Eigen/oneMKL GEMM produced a non-finite checksum.");

  const double wallSeconds =
    std::chrono::duration<double>(wallEnd - wallStart).count();
  const double cpuSeconds =
    static_cast<double>(cpuEnd - cpuStart) / static_cast<double>(CLOCKS_PER_SEC);
  std::cout << std::setprecision(17)
            << "PGO_MKL_TASK_PROFILE_END"
            << " policy=" << mklExecutorCaseName(arguments.policy)
            << " wall_seconds=" << wallSeconds
            << " process_cpu_seconds=" << cpuSeconds
            << " checksum=" << checksum << std::endl;
}

}  // namespace

int main(int argc, char **argv)
{
  try {
    run(parseArguments(argc, argv));
    return 0;
  }
  catch (const std::exception &error) {
    std::cerr << "eigen_mkl_task_profile_probe: " << error.what() << '\n';
    return 1;
  }
}
