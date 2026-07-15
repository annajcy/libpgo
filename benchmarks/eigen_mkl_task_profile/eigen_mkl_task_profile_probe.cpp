#include "../eigen_mkl_nested/eigen_mkl_nested_kernel.h"

#include "parallel/parallelControl.h"

#include <mkl.h>

#include <chrono>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace
{

namespace P = pgo::parallel;
using pgo::benchmark_helpers::NestedEigenMklWorkload;

enum class Mode
{
  Default,
  Local1,
  Local2,
  Local4,
};

struct Arguments
{
  Mode mode;
  int concurrency;
  int matrixN;
  int warmupIterations;
  int profileIterations;
};

const char *modeName(Mode mode) noexcept
{
  switch (mode) {
  case Mode::Default:
    return "Default";
  case Mode::Local1:
    return "Local1";
  case Mode::Local2:
    return "Local2";
  case Mode::Local4:
    return "Local4";
  }
  return "Unknown";
}

std::optional<int> localThreadLimit(Mode mode) noexcept
{
  switch (mode) {
  case Mode::Default:
    return std::nullopt;
  case Mode::Local1:
    return 1;
  case Mode::Local2:
    return 2;
  case Mode::Local4:
    return 4;
  }
  return std::nullopt;
}

Mode parseMode(std::string_view value)
{
  if (value == "Default")
    return Mode::Default;
  if (value == "Local1")
    return Mode::Local1;
  if (value == "Local2")
    return Mode::Local2;
  if (value == "Local4")
    return Mode::Local4;
  throw std::invalid_argument(
    "--mode must be Default, Local1, Local2, or Local4.");
}

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
    parseMode(requireValue(argc, argv, "--mode=")),
    parsePositiveInteger(
      requireValue(argc, argv, "--concurrency="), "--concurrency"),
    parsePositiveInteger(requireValue(argc, argv, "--matrix-n="), "--matrix-n"),
    parseNonnegativeInteger(
      requireValue(argc, argv, "--warmup-iterations="), "--warmup-iterations"),
    parsePositiveInteger(
      requireValue(argc, argv, "--profile-iterations="), "--profile-iterations"),
  };
}

class ScopedMklLocalThreads
{
public:
  explicit ScopedMklLocalThreads(std::optional<int> limit)
  {
    if (limit.has_value())
      previous_ = mkl_set_num_threads_local(*limit);
  }

  ~ScopedMklLocalThreads()
  {
    if (previous_.has_value())
      mkl_set_num_threads_local(*previous_);
  }

  ScopedMklLocalThreads(const ScopedMklLocalThreads &) = delete;
  ScopedMklLocalThreads &operator=(const ScopedMklLocalThreads &) = delete;

private:
  std::optional<int> previous_;
};

void runIterations(NestedEigenMklWorkload &workload, int iterations)
{
  for (int iteration = 0; iteration < iterations; ++iteration)
    workload.run(0);
}

void run(const Arguments &arguments)
{
  const int effectiveConcurrency = P::initialize(arguments.concurrency);
  NestedEigenMklWorkload workload(1, arguments.matrixN);

  P::withGlobalTbbConcurrency([&] {
    ScopedMklLocalThreads localThreads(localThreadLimit(arguments.mode));
    runIterations(workload, arguments.warmupIterations);
  });

  const std::clock_t cpuStart = std::clock();
  const auto wallStart = std::chrono::steady_clock::now();

  std::cout << "PGO_MKL_TASK_PROFILE_BEGIN"
            << " mode=" << modeName(arguments.mode)
            << " configured_concurrency=" << arguments.concurrency
            << " effective_concurrency=" << effectiveConcurrency
            << " matrix_n=" << arguments.matrixN
            << " iterations=" << arguments.profileIterations << std::endl;

  P::withGlobalTbbConcurrency([&] {
    ScopedMklLocalThreads localThreads(localThreadLimit(arguments.mode));
    runIterations(workload, arguments.profileIterations);
  });

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
            << " mode=" << modeName(arguments.mode)
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
