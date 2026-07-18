#include "../eigen_mkl_common/eigen_mkl_executor_cases.h"
#include "../eigen_mkl_common/eigen_mkl_gemm_workload.h"
#include "../benchmark_argument_parser.h"

#include "parallel/arenaThreadingExecutor.h"
#include "parallel/parallelControl.h"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/partitioner.h>

#include <climits>
#include <cstdlib>
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
using pgo::benchmark_helpers::parsePositiveInteger;
using pgo::benchmark_helpers::requireValue;

struct Arguments
{
  MklExecutorCase policy;
  int concurrency;
  int outerTasks;
  int matrixN;
};

Arguments parseArguments(int argc, char **argv)
{
  return {
    parseMklExecutorCase(requireValue(argc, argv, "--policy=")),
    parsePositiveInteger(requireValue(argc, argv, "--concurrency="), "--concurrency"),
    parsePositiveInteger(requireValue(argc, argv, "--outer-tasks="), "--outer-tasks"),
    parsePositiveInteger(requireValue(argc, argv, "--matrix-n="), "--matrix-n"),
  };
}

void run(const Arguments &arguments)
{
  P::GlobalTbbControl control(arguments.concurrency);
  const int effectiveConcurrency = static_cast<int>(tbb::global_control::active_value(
    tbb::global_control::max_allowed_parallelism));
  const auto spec = mklExecutorSpec(arguments.policy, arguments.concurrency);
  P::ArenaThreadingExecutor executor(spec.arenaConcurrency,
    { .mklLocalThreadBudget = spec.mklLocalThreadBudget });
  EigenMklGemmWorkload workload(arguments.outerTasks, arguments.matrixN);

  std::cout << "PGO_MKL_VERBOSE_PROBE_BEGIN policy="
            << mklExecutorCaseName(arguments.policy)
            << " configured_global_concurrency=" << arguments.concurrency
            << " effective_global_concurrency=" << effectiveConcurrency
            << " configured_arena_concurrency=" << spec.arenaConcurrency
            << " configured_mkl_local_budget=" << spec.mklLocalThreadBudget
            << " outer_tasks=" << arguments.outerTasks
            << " matrix_n=" << arguments.matrixN << '\n';

  executor.execute([&] {
    tbb::parallel_for(
      tbb::blocked_range<int>(0, arguments.outerTasks, 1),
      [&](const tbb::blocked_range<int> &range) {
        for (int taskIndex = range.begin(); taskIndex < range.end(); ++taskIndex)
          workload.run(taskIndex);
      },
      tbb::auto_partitioner{});
  });

  std::cout << "PGO_MKL_VERBOSE_PROBE_END policy="
            << mklExecutorCaseName(arguments.policy)
            << " checksum=" << workload.checksum() << '\n';
}

}  // namespace

int main(int argc, char **argv)
{
  try {
    run(parseArguments(argc, argv));
    return 0;
  }
  catch (const std::exception &error) {
    std::cerr << "eigen_mkl_control_matrix_probe: " << error.what() << '\n';
    return 1;
  }
}
