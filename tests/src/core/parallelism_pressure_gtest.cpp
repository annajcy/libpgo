#include <gtest/gtest.h>

#include "parallelism/parallelFor.h"

#include <atomic>
#include <chrono>
#include <thread>

namespace P = pgo::parallel;

TEST(ParallelPressureTest, IndependentExternalCallersSetStructuredPressureFlag)
{
  auto &runtime = P::initializeRuntime({ .maxTbbConcurrency = 1 });
  const auto first = runtime.createExecutor({ .maxConcurrency = 1 });
  const auto second = runtime.createExecutor({ .maxConcurrency = 1 });
  std::atomic<int> entered = 0;
  std::atomic<bool> release = false;

  auto call = [&](const P::ParallelExecutor &executor) {
    P::parallelFor(executor, 0, 1, [&](int) {
      entered.fetch_add(1, std::memory_order_release);
      while (!release.load(std::memory_order_acquire))
        std::this_thread::yield();
    });
  };
  std::thread a(call, std::cref(first));
  std::thread b(call, std::cref(second));

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (entered.load(std::memory_order_acquire) < 2 &&
    std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }

  if (entered.load(std::memory_order_acquire) != 2) {
    release.store(true, std::memory_order_release);
    a.join();
    b.join();
    FAIL() << "Both application threads did not enter their arenas before timeout.";
  }
  const P::RuntimeInfo info = runtime.info();
  EXPECT_EQ(info.effectiveTbbMaxAllowedParallelism, 1);
  EXPECT_EQ(info.currentWorkerParticipants, 0);
  EXPECT_EQ(info.currentExternalParticipants, 2);
  EXPECT_EQ(info.currentTotalParticipants, 2);
  EXPECT_GE(info.peakTotalParticipants, 2);
  EXPECT_TRUE(info.participantPressureObserved);

  release.store(true, std::memory_order_release);
  a.join();
  b.join();
}
