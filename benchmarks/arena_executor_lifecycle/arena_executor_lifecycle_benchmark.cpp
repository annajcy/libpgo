#include "../benchmark_argument_parser.h"
#include "../timed_workload.h"
#include "../workload_warmup.h"

#include "parallel/arenaThreadingExecutor.h"
#include "parallel/arenaThreadingExecutorInternal.h"
#include "parallel/parallelControl.h"

#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

#include <chrono>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace
{

namespace P = pgo::parallel;
using pgo::benchmark_helpers::parseNonnegativeDouble;
using pgo::benchmark_helpers::parseNonnegativeInteger;
using pgo::benchmark_helpers::parsePositiveInteger;
using pgo::benchmark_helpers::parseTimedMeasurementArguments;
using pgo::benchmark_helpers::requireValue;
using pgo::benchmark_helpers::runTimedMeasurement;
using pgo::benchmark_helpers::runWorkloadWarmup;
using pgo::benchmark_helpers::TimedMeasurementArguments;

enum class LifecycleCase
{
  Reuse,
  Recreate,
  RecreateRetainedAttachment,
};

const char *caseName(LifecycleCase testCase) noexcept
{
  switch (testCase) {
  case LifecycleCase::Reuse:
    return "reuse";
  case LifecycleCase::Recreate:
    return "recreate";
  case LifecycleCase::RecreateRetainedAttachment:
    return "recreate_retained_attachment";
  }
  return "unknown";
}

LifecycleCase parseCase(std::string_view value)
{
  if (value == "reuse")
    return LifecycleCase::Reuse;
  if (value == "recreate")
    return LifecycleCase::Recreate;
  if (value == "recreate_retained_attachment")
    return LifecycleCase::RecreateRetainedAttachment;
  throw std::invalid_argument(
    "--case must be reuse, recreate, or recreate_retained_attachment.");
}

struct Arguments
{
  LifecycleCase testCase;
  int concurrency;
  int outerTasks;
  double warmupSeconds;
  int warmupMinOperations;
  TimedMeasurementArguments measurement;
  int drainTimeoutMilliseconds;
};

Arguments parseArguments(int argc, char **argv)
{
  return {
    parseCase(requireValue(argc, argv, "--case=")),
    parsePositiveInteger(
      requireValue(argc, argv, "--concurrency="), "--concurrency"),
    parsePositiveInteger(
      requireValue(argc, argv, "--outer-tasks="), "--outer-tasks"),
    parseNonnegativeDouble(
      requireValue(argc, argv, "--warmup-seconds="), "--warmup-seconds"),
    parseNonnegativeInteger(requireValue(argc, argv, "--warmup-min-operations="),
      "--warmup-min-operations"),
    parseTimedMeasurementArguments(argc, argv),
    parseNonnegativeInteger(requireValue(argc, argv, "--drain-timeout-ms="),
      "--drain-timeout-ms"),
  };
}

P::ThreadingPolicy outerPolicy()
{
  return {
    .mklLocalThreadBudget = 2,
    .accelerate = P::AccelerateThreading::multi,
  };
}

P::ThreadingPolicy innerPolicy()
{
  return {
    .mklLocalThreadBudget = 1,
    .accelerate = P::AccelerateThreading::single,
  };
}

struct alignas(64) Counter
{
  std::uint64_t value = 0;
  std::uint64_t constructions = 0;
};

using ExecutorList = std::vector<std::unique_ptr<P::ArenaThreadingExecutor>>;
using AttachmentList =
  std::vector<std::vector<std::unique_ptr<tbb::task_arena>>>;

ExecutorList makeInnerExecutors(int outerTasks)
{
  ExecutorList executors;
  executors.reserve(static_cast<std::size_t>(outerTasks));
  for (int task = 0; task < outerTasks; ++task) {
    executors.emplace_back(
      std::make_unique<P::ArenaThreadingExecutor>(1, innerPolicy()));
  }
  return executors;
}

class LifecycleWorkload
{
public:
  LifecycleWorkload(P::ArenaThreadingExecutor &outer, LifecycleCase testCase,
    int outerTasks):
    outer_(outer),
    testCase_(testCase),
    outerTasks_(outerTasks),
    counters_(static_cast<std::size_t>(outerTasks)),
    retainedAttachments_(static_cast<std::size_t>(outerTasks))
  {
    if (testCase_ == LifecycleCase::Reuse)
      reusedExecutors_ = makeInnerExecutors(outerTasks_);
  }

  void runBatch()
  {
    outer_.execute([&] {
      tbb::parallel_for(0, outerTasks_, [&](int task) {
        if (testCase_ == LifecycleCase::Reuse) {
          invoke(*reusedExecutors_[static_cast<std::size_t>(task)], task, false);
          return;
        }

        ++counters_[static_cast<std::size_t>(task)].constructions;
        P::ArenaThreadingExecutor inner(1, innerPolicy());
        invoke(inner, task,
          testCase_ == LifecycleCase::RecreateRetainedAttachment);
      });
    });
  }

  void resetCounters() noexcept
  {
    for (Counter &counter : counters_)
      counter.value = 0;
    for (Counter &counter : counters_)
      counter.constructions = 0;
  }

  std::uint64_t checksum() const noexcept
  {
    std::uint64_t sum = 0;
    for (const Counter &counter : counters_)
      sum += counter.value;
    return sum;
  }

  std::uint64_t constructionCount() const noexcept
  {
    std::uint64_t sum = 0;
    for (const Counter &counter : counters_)
      sum += counter.constructions;
    return sum;
  }

  std::size_t retainedAttachmentCount() const noexcept
  {
    std::size_t count = 0;
    for (const auto &attachments : retainedAttachments_)
      count += attachments.size();
    return count;
  }

  void releaseAttachments()
  {
    for (auto &attachments : retainedAttachments_)
      attachments.clear();
  }

  void releaseReusedExecutors()
  {
    reusedExecutors_.clear();
  }

private:
  void invoke(P::ArenaThreadingExecutor &inner, int task,
    bool retainAttachment)
  {
    inner.execute([&] {
      ++counters_[static_cast<std::size_t>(task)].value;
      if (retainAttachment) {
        retainedAttachments_[static_cast<std::size_t>(task)].emplace_back(
          std::make_unique<tbb::task_arena>(tbb::attach{}));
      }
    });
  }

  P::ArenaThreadingExecutor &outer_;
  LifecycleCase testCase_;
  int outerTasks_;
  std::vector<Counter> counters_;
  ExecutorList reusedExecutors_;
  AttachmentList retainedAttachments_;
};

bool drain(std::chrono::milliseconds timeout)
{
  return P::drainRetiredArenaThreadingExecutorStates(timeout);
}

void run(const Arguments &arguments)
{
  if (arguments.testCase == LifecycleCase::RecreateRetainedAttachment &&
    arguments.measurement.minimumSeconds > 0.0) {
    throw std::invalid_argument(
      "recreate_retained_attachment requires --measurement-min-seconds=0 "
      "to bound retained arena growth.");
  }

  const auto drainTimeout =
    std::chrono::milliseconds(arguments.drainTimeoutMilliseconds);
  if (!drain(drainTimeout))
    throw std::runtime_error("Retirement registry was not empty before warmup.");

  P::GlobalTbbControl control(arguments.concurrency);
  const int effectiveConcurrency = static_cast<int>(
    tbb::global_control::active_value(
      tbb::global_control::max_allowed_parallelism));
  P::ArenaThreadingExecutor outer(arguments.concurrency, outerPolicy());
  LifecycleWorkload workload(
    outer, arguments.testCase, arguments.outerTasks);

  const auto warmup = runWorkloadWarmup([&] { workload.runBatch(); },
    arguments.warmupSeconds, arguments.warmupMinOperations);
  const std::size_t retiredAfterWarmup =
    P::detail::retiredArenaThreadingExecutorStateCount();
  workload.releaseAttachments();
  if (!drain(drainTimeout))
    throw std::runtime_error("Retirement registry did not drain after warmup.");
  workload.resetCounters();

  const std::clock_t cpuStart = std::clock();
  const auto measurement =
    runTimedMeasurement([&] { workload.runBatch(); }, arguments.measurement);
  const std::clock_t cpuEnd = std::clock();

  const std::size_t retiredAfterMeasurement =
    P::detail::retiredArenaThreadingExecutorStateCount();
  const std::size_t retainedAttachmentCount =
    workload.retainedAttachmentCount();

  const auto releaseStart = std::chrono::steady_clock::now();
  workload.releaseAttachments();
  const auto releaseEnd = std::chrono::steady_clock::now();
  const std::size_t retiredAfterAttachmentRelease =
    P::detail::retiredArenaThreadingExecutorStateCount();

  workload.releaseReusedExecutors();
  const std::size_t retiredAfterReuseRelease =
    P::detail::retiredArenaThreadingExecutorStateCount();

  const auto drainStart = std::chrono::steady_clock::now();
  const bool drained = drain(drainTimeout);
  const auto drainEnd = std::chrono::steady_clock::now();
  const std::size_t retiredAfterDrain =
    P::detail::retiredArenaThreadingExecutorStateCount();

  const std::uint64_t measuredInnerCalls =
    static_cast<std::uint64_t>(measurement.completedOperations) *
    static_cast<std::uint64_t>(arguments.outerTasks);
  const std::uint64_t checksum = workload.checksum();
  const std::uint64_t timedInnerConstructions = workload.constructionCount();
  if (checksum != measuredInnerCalls)
    throw std::runtime_error("Nested operation checksum mismatch.");
  if (!drained || retiredAfterDrain != 0)
    throw std::runtime_error("Retirement registry did not drain after timing.");

  const double cpuSeconds =
    static_cast<double>(cpuEnd - cpuStart) / static_cast<double>(CLOCKS_PER_SEC);
  const double releaseSeconds =
    std::chrono::duration<double>(releaseEnd - releaseStart).count();
  const double drainSeconds =
    std::chrono::duration<double>(drainEnd - drainStart).count();

  std::cout << std::setprecision(17)
            << "PGO_ARENA_EXECUTOR_LIFECYCLE_RESULT"
            << " case=" << caseName(arguments.testCase)
            << " configured_concurrency=" << arguments.concurrency
            << " effective_concurrency=" << effectiveConcurrency
            << " outer_tasks=" << arguments.outerTasks
            << " configured_inner_concurrency=1"
            << " configured_warmup_seconds=" << arguments.warmupSeconds
            << " configured_warmup_min_operations="
            << arguments.warmupMinOperations
            << " actual_warmup_seconds=" << warmup.elapsedSeconds
            << " actual_warmup_operations=" << warmup.completedOperations
            << " configured_measurement_min_seconds="
            << arguments.measurement.minimumSeconds
            << " configured_measurement_min_operations="
            << arguments.measurement.minimumOperations
            << " measurement_operations=" << measurement.completedOperations
            << " measurement_inner_calls=" << measuredInnerCalls
            << " timed_inner_constructions=" << timedInnerConstructions
            << " retained_attachments=" << retainedAttachmentCount
            << " retired_after_warmup=" << retiredAfterWarmup
            << " retired_after_measurement=" << retiredAfterMeasurement
            << " retired_after_attachment_release="
            << retiredAfterAttachmentRelease
            << " retired_after_reuse_release=" << retiredAfterReuseRelease
            << " retired_after_drain=" << retiredAfterDrain
            << " drain_succeeded=" << (drained ? 1 : 0)
            << " checksum=" << checksum
            << " measurement_wall_seconds=" << measurement.elapsedSeconds
            << " process_cpu_seconds=" << cpuSeconds
            << " attachment_release_seconds=" << releaseSeconds
            << " drain_seconds=" << drainSeconds << '\n';
}

}  // namespace

int main(int argc, char **argv)
{
  try {
    run(parseArguments(argc, argv));
    return 0;
  }
  catch (const std::exception &exception) {
    std::cerr << "arena_executor_lifecycle_benchmark: " << exception.what()
              << '\n';
    return 1;
  }
}
