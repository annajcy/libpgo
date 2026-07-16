#include "../eigen_mkl_nested/eigen_mkl_nested_kernel.h"

#include "parallel/arenaThreadingExecutor.h"
#include "parallel/parallelControl.h"

#include <mkl.h>

#include <tbb/partitioner.h>

#include <climits>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

namespace
{

namespace P = pgo::parallel;
using pgo::benchmark_helpers::NestedEigenMklWorkload;

enum class Policy
{
  DefaultArenaGlobal,
  Local1ArenaGlobal,
  DefaultArena1,
  Local1Arena1,
};

struct Arguments
{
  Policy policy;
  int concurrency;
  int outerTasks;
  int matrixN;
};

const char *policyName(Policy policy) noexcept
{
  switch (policy) {
  case Policy::DefaultArenaGlobal:
    return "DefaultArenaGlobal";
  case Policy::Local1ArenaGlobal:
    return "Local1ArenaGlobal";
  case Policy::DefaultArena1:
    return "DefaultArena1";
  case Policy::Local1Arena1:
    return "Local1Arena1";
  }
  return "Unknown";
}

Policy parsePolicy(std::string_view value)
{
  if (value == "DefaultArenaGlobal")
    return Policy::DefaultArenaGlobal;
  if (value == "Local1ArenaGlobal")
    return Policy::Local1ArenaGlobal;
  if (value == "DefaultArena1")
    return Policy::DefaultArena1;
  if (value == "Local1Arena1")
    return Policy::Local1Arena1;
  throw std::invalid_argument("Unknown --policy value.");
}

bool usesMklLocalOne(Policy policy) noexcept
{
  return policy == Policy::Local1ArenaGlobal || policy == Policy::Local1Arena1;
}

bool usesSingleParticipantArena(Policy policy) noexcept
{
  return policy == Policy::DefaultArena1 || policy == Policy::Local1Arena1;
}

int parsePositiveInteger(std::string_view value, std::string_view option)
{
  char *end = nullptr;
  const long parsed = std::strtol(value.data(), &end, 10);
  if (end == value.data() || *end != '\0' || parsed <= 0 || parsed > INT_MAX)
    throw std::invalid_argument(std::string(option) + " must be a positive integer.");
  return static_cast<int>(parsed);
}

std::string_view requireValue(int argc, char **argv, std::string_view prefix)
{
  for (int i = 1; i < argc; ++i) {
    const std::string_view argument(argv[i]);
    if (argument.size() >= prefix.size() &&
      argument.substr(0, prefix.size()) == prefix) {
      return argument.substr(prefix.size());
    }
  }
  throw std::invalid_argument("Missing required option " + std::string(prefix));
}

Arguments parseArguments(int argc, char **argv)
{
  return {
    parsePolicy(requireValue(argc, argv, "--policy=")),
    parsePositiveInteger(requireValue(argc, argv, "--concurrency="), "--concurrency"),
    parsePositiveInteger(requireValue(argc, argv, "--outer-tasks="), "--outer-tasks"),
    parsePositiveInteger(requireValue(argc, argv, "--matrix-n="), "--matrix-n"),
  };
}

template<class Fn>
void withMklApiSetting(Policy policy, Fn &&fn)
{
  if (!usesMklLocalOne(policy)) {
    fn();
    return;
  }

  const int previous = mkl_set_num_threads_local(1);
  try {
    fn();
  }
  catch (...) {
    mkl_set_num_threads_local(previous);
    throw;
  }
  mkl_set_num_threads_local(previous);
}

void run(const Arguments &arguments)
{
  P::GlobalTbbControl control(arguments.concurrency);
  const int effectiveConcurrency = static_cast<int>(tbb::global_control::active_value(
    tbb::global_control::max_allowed_parallelism));
  P::ArenaThreadingExecutor outerExecutor(arguments.concurrency,
    { .mklLocalThreadBudget = 0 });
  tbb::task_arena singleArena(1, 1);
  NestedEigenMklWorkload workload(arguments.outerTasks, arguments.matrixN);

  std::cout << "PGO_MKL_VERBOSE_PROBE_BEGIN policy=" << policyName(arguments.policy)
            << " configured_concurrency=" << arguments.concurrency
            << " effective_concurrency=" << effectiveConcurrency
            << " outer_tasks=" << arguments.outerTasks
            << " matrix_n=" << arguments.matrixN << '\n';

  outerExecutor.execute([&] {
    tbb::parallel_for(
      tbb::blocked_range<int>(0, arguments.outerTasks, 1),
      [&](const tbb::blocked_range<int> &range) {
        for (int taskIndex = range.begin(); taskIndex < range.end(); ++taskIndex) {
          const auto runKernel = [&] {
            withMklApiSetting(arguments.policy, [&] { workload.run(taskIndex); });
          };
          if (usesSingleParticipantArena(arguments.policy))
            singleArena.execute(runKernel);
          else
            runKernel();
        }
      },
      tbb::auto_partitioner{});
  });

  std::cout << "PGO_MKL_VERBOSE_PROBE_END policy=" << policyName(arguments.policy)
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
