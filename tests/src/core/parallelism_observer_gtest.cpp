#include <gtest/gtest.h>

#include "parallelism/parallelFor.h"

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

namespace P = pgo::parallel;

namespace
{

bool waitFor(const std::atomic<int> &value, int target)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (value.load(std::memory_order_acquire) < target &&
    std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  return value.load(std::memory_order_acquire) >= target;
}

}  // namespace

TEST(ParallelObserverTest, ClassifiesExternalAndWorkerParticipants)
{
  if (P::defaultConcurrency() < 2)
    GTEST_SKIP() << "This environment cannot supply a TBB worker.";

  auto &runtime = P::initializeRuntime({ .maxTbbConcurrency = 2 });
  const auto executor = runtime.createExecutor({ .maxConcurrency = 2 });
  std::atomic<int> entered = 0;
  std::atomic<bool> release = false;
  std::thread caller([&] {
    P::Options options;
    options.grainSize = 1;
    P::parallelFor(executor, 0, 2, options, [&](int) {
      entered.fetch_add(1, std::memory_order_release);
      while (!release.load(std::memory_order_acquire))
        std::this_thread::yield();
    });
  });

  if (!waitFor(entered, 2)) {
    release.store(true, std::memory_order_release);
    caller.join();
    GTEST_SKIP() << "The scheduler did not provide a worker before timeout.";
  }

  const P::RuntimeInfo info = runtime.info();
  EXPECT_GE(info.currentWorkerParticipants, 1);
  EXPECT_GE(info.currentExternalParticipants, 1);
  EXPECT_EQ(info.currentTotalParticipants,
    info.currentWorkerParticipants + info.currentExternalParticipants);
  EXPECT_GE(info.peakTotalParticipants, info.currentTotalParticipants);
  EXPECT_LE(info.currentWorkerParticipants, info.tbbWorkerCeiling);

  release.store(true, std::memory_order_release);
  caller.join();
}

TEST(ParallelObserverTest, ExecutorTeardownUnregistersLocalParticipants)
{
  auto &runtime = P::initializeRuntime({ .maxTbbConcurrency = 2 });
  const int baseline = runtime.info().currentTotalParticipants;
  {
    const auto executor = runtime.createExecutor({ .maxConcurrency = 2 });
    P::parallelFor(executor, 0, 64, [](int) {});
  }
  const P::RuntimeInfo info = runtime.info();
  EXPECT_EQ(info.currentTotalParticipants, baseline);
  EXPECT_GE(info.currentWorkerParticipants, 0);
  EXPECT_GE(info.currentExternalParticipants, 0);

  const auto next = runtime.createExecutor({ .maxConcurrency = 2 });
  EXPECT_NO_THROW(P::parallelFor(next, 0, 4, [](int) {}));
}

TEST(ParallelObserverTest, SnapshotsRemainConsistentUnderConcurrentCalls)
{
  auto &runtime = P::initializeRuntime({ .maxTbbConcurrency = 2 });
  const auto executor = runtime.createExecutor({ .maxConcurrency = 2 });
  constexpr int numCallers = 6;
  constexpr int repetitions = 200;
  std::atomic<bool> start = false;
  std::atomic<int> completed = 0;
  std::vector<std::thread> callers;
  for (int caller = 0; caller < numCallers; ++caller) {
    callers.emplace_back([&] {
      while (!start.load(std::memory_order_acquire))
        std::this_thread::yield();
      for (int repetition = 0; repetition < repetitions; ++repetition)
        P::parallelFor(executor, 0, 16, [](int) {});
      completed.fetch_add(1, std::memory_order_release);
    });
  }

  start.store(true, std::memory_order_release);
  while (completed.load(std::memory_order_acquire) != numCallers) {
    const P::RuntimeInfo info = runtime.info();
    EXPECT_GE(info.currentWorkerParticipants, 0);
    EXPECT_GE(info.currentExternalParticipants, 0);
    EXPECT_EQ(info.currentTotalParticipants,
      info.currentWorkerParticipants + info.currentExternalParticipants);
    EXPECT_GE(info.peakTotalParticipants, info.currentTotalParticipants);
  }
  for (auto &caller : callers)
    caller.join();
}
