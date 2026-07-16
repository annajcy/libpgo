#include "../eigen_mkl_common/eigen_mkl_executor_cases.h"
#include "../eigen_mkl_common/eigen_mkl_gemm_workload.h"

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

struct Arguments
{
  MklExecutorCase policy;
  int concurrency;
  int matrixN;
  int warmupIterations;
  int profileIterations;
};

int parseNonnegativeInteger(std::string_view value, std::string_view option)
{
  char *end = nullptr;
  const long parsed = std::strtol(value.data(), &end, 10);
  if (end == value.data() || *end != '\0' || parsed < 0 || parsed > INT_MAX)
    throw std::invalid_argument(std::string(option) + " must be a nonnegative integer.");
  return static_cast<int>(parsed);
}

int parsePositiveInteger(std::string_view value, std::string_view option)
{
  const int parsed = parseNonnegativeInteger(value, option);
  if (parsed == 0)
    throw std::invalid_argument(std::string(option) + " must be positive.");
  return parsed;
}

std::string_view requireValue(int argc, char **argv, std::string_view prefix)
{
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    if (argument.starts_with(prefix))
      return argument.substr(prefix.size());
  }
  throw std::invalid_argument("Missing required option " + std::string(prefix));
}

Arguments parseArguments(int argc, char **argv)
{
  return {
    parseMklExecutorCase(requireValue(argc, argv, "--policy=")),
    parsePositiveInteger(
      requireValue(argc, argv, "--concurrency="), "--concurrency"),
    parsePositiveInteger(requireValue(argc, argv, "--matrix-n="), "--matrix-n"),
    parseNonnegativeInteger(
      requireValue(argc, argv, "--warmup-iterations="), "--warmup-iterations"),
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

  executor.execute([&] { runIterations(workload, arguments.warmupIterations); });

  const std::clock_t cpuStart = std::clock();
  const auto wallStart = std::chrono::steady_clock::now();

  std::cout << "PGO_MKL_TASK_PROFILE_BEGIN"
            << " policy=" << mklExecutorCaseName(arguments.policy)
            << " configured_global_concurrency=" << arguments.concurrency
            << " effective_global_concurrency=" << effectiveConcurrency
            << " configured_arena_concurrency=" << spec.arenaConcurrency
            << " configured_mkl_local_budget=" << spec.mklLocalThreadBudget
            << " matrix_n=" << arguments.matrixN
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
