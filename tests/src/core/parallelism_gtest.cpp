#include <gtest/gtest.h>

#include "parallelism/parallelFor.h"

#include <atomic>
#include <limits>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

#ifdef PGO_PARALLELISM_HAS_TBB
#  include <tbb/task_arena.h>
#endif

#ifdef PGO_PARALLELISM_HAS_MKL
#  include <mkl.h>
#endif

namespace P = pgo::parallel;

TEST(ParallelForTest, VisitsEveryIndexAndHandlesRanges)
{
  std::vector<std::atomic<int>> visits(103);
  P::Options options;
  options.grainSize = 7;
  P::parallelFor(0, static_cast<int>(visits.size()), options, [&](int i) {
    visits[static_cast<std::size_t>(i)].fetch_add(1, std::memory_order_relaxed);
  });

  for (const auto &visit : visits)
    EXPECT_EQ(visit.load(std::memory_order_relaxed), 1);

  int calls = 0;
  P::parallelFor(5, 5, [&](int) { ++calls; });
  P::parallelFor(7, 2, [&](int) { ++calls; });
  EXPECT_EQ(calls, 0);
}

TEST(ParallelForTest, ParallelFor3DVisitsEveryCoordinateAndValidatesCapacity)
{
  constexpr int nx = 3;
  constexpr int ny = 4;
  constexpr int nz = 5;
  std::vector<std::atomic<int>> visits(nx * ny * nz);
  P::parallelFor3D(nx, ny, nz, [&](int x, int y, int z) {
    visits[static_cast<std::size_t>(x + nx * (y + ny * z))].fetch_add(1, std::memory_order_relaxed);
  });
  for (const auto &visit : visits)
    EXPECT_EQ(visit.load(std::memory_order_relaxed), 1);

  int calls = 0;
  P::parallelFor3D(0, 4, 5, [&](int, int, int) { ++calls; });
  EXPECT_EQ(calls, 0);
  EXPECT_THROW(P::parallelFor3D(std::numeric_limits<int>::max(), 2, 2, [](int, int, int) {}),
    std::runtime_error);
}

TEST(ParallelExecutorTest, ExplicitExecutorCopiesAndRemainsReusableAfterException)
{
  auto executor = P::runtime().createExecutor({ .maxConcurrency = 2 });
  auto copy = executor;
  EXPECT_EQ(copy.maxConcurrency(), 2);

  EXPECT_THROW(P::parallelFor(copy, 0, 16, [](int i) {
    if (i == 3)
      throw std::runtime_error("body failure");
  }),
    std::runtime_error);

  std::atomic<int> count = 0;
  executor = copy;
  P::parallelFor(executor, 0, 29, [&](int) {
    count.fetch_add(1, std::memory_order_relaxed);
  });
  EXPECT_EQ(count.load(std::memory_order_relaxed), 29);
}

TEST(ParallelExecutorTest, NestedImplicitAndSameExecutorInheritArena)
{
  const int arenaConcurrency = std::min(3, P::defaultConcurrency());
  const auto executor = P::runtime().createExecutor({ .maxConcurrency = arenaConcurrency });
  std::atomic<int> implicitCalls = 0;
  std::atomic<int> sameCalls = 0;
  std::atomic<int> observedConcurrency = arenaConcurrency;

  P::parallelFor(executor, 0, 4, [&](int) {
    P::parallelFor(0, 5, [&](int) {
      implicitCalls.fetch_add(1, std::memory_order_relaxed);
#ifdef PGO_PARALLELISM_HAS_TBB
      observedConcurrency.store(tbb::this_task_arena::max_concurrency(), std::memory_order_relaxed);
#endif

    });
    P::parallelFor(executor, 0, 3, [&](int) {
      sameCalls.fetch_add(1, std::memory_order_relaxed);
    });
  });

  EXPECT_EQ(implicitCalls.load(std::memory_order_relaxed), 20);
  EXPECT_EQ(sameCalls.load(std::memory_order_relaxed), 12);
  EXPECT_EQ(observedConcurrency.load(std::memory_order_relaxed), arenaConcurrency);
}

TEST(ParallelExecutorTest, NestedDifferentExecutorIsRejectedAndContextRestores)
{
  auto first = P::runtime().createExecutor({ .maxConcurrency = 2 });
  auto second = P::runtime().createExecutor({ .maxConcurrency = 2 });
  std::atomic<int> secondCalls = 0;

  EXPECT_THROW(P::parallelFor(first, 0, 1, [&](int) {
    P::parallelFor(second, 0, 1, [&](int) {
      secondCalls.fetch_add(1, std::memory_order_relaxed);
    });
  }),
    std::logic_error);
  EXPECT_EQ(secondCalls.load(std::memory_order_relaxed), 0);

  P::parallelFor(second, 0, 7, [&](int) {
    secondCalls.fetch_add(1, std::memory_order_relaxed);
  });
  EXPECT_EQ(secondCalls.load(std::memory_order_relaxed), 7);
}

TEST(ParallelExecutorTest, SharedExecutorAcceptsConcurrentCallers)
{
  const int arenaConcurrency = std::min(4, P::defaultConcurrency());
  const auto executor = P::runtime().createExecutor({ .maxConcurrency = arenaConcurrency });
  constexpr int numCallers = 6;
  constexpr int workPerCaller = 101;
  std::atomic<int> count = 0;
  std::vector<std::thread> callers;
  for (int caller = 0; caller < numCallers; ++caller) {
    callers.emplace_back([&] {
      P::parallelFor(executor, 0, workPerCaller, [&](int) {
        count.fetch_add(1, std::memory_order_relaxed);
      });
    });
  }
  for (auto &caller : callers)
    caller.join();

  EXPECT_EQ(count.load(std::memory_order_relaxed), numCallers * workPerCaller);
  const P::RuntimeInfo info = P::runtimeInfo();
  EXPECT_GE(info.peakTotalParticipants, 1);
  EXPECT_EQ(info.currentTotalParticipants,
    info.currentWorkerParticipants + info.currentExternalParticipants);
}

TEST(ParallelExecutorTest, ActiveCallRetainsExecutorStateAfterHandleDestruction)
{
  std::optional<P::ParallelExecutor> executor(
    P::runtime().createExecutor({ .maxConcurrency = 1 }));
  std::atomic<bool> entered = false;
  std::atomic<bool> release = false;
  std::thread caller([&] {
    P::parallelFor(*executor, 0, 1, [&](int) {
      entered.store(true, std::memory_order_release);
      while (!release.load(std::memory_order_acquire))
        std::this_thread::yield();
    });
  });

  while (!entered.load(std::memory_order_acquire))
    std::this_thread::yield();
  executor.reset();
  release.store(true, std::memory_order_release);
  caller.join();

  const auto next = P::runtime().createExecutor({ .maxConcurrency = 1 });
  EXPECT_NO_THROW(P::parallelFor(next, 0, 1, [](int) {}));
}

#ifdef PGO_PARALLELISM_HAS_MKL
TEST(ParallelNestedKernelTest, MklSuppressAndInheritRestoreLocalThreads)
{
  const int initial = mkl_get_max_threads();
  const auto executor = P::runtime().createExecutor({ .maxConcurrency = 2 });

  std::atomic<int> suppressed = -1;
  P::parallelFor(executor, 0, 4, [&](int) {
    suppressed.store(mkl_get_max_threads(), std::memory_order_relaxed);
  });
  EXPECT_EQ(suppressed.load(std::memory_order_relaxed), 1);
  EXPECT_EQ(mkl_get_max_threads(), initial);

  P::Options inherit;
  inherit.nestedKernelPolicy = P::NestedKernelPolicy::Inherit;
  std::atomic<int> inherited = -1;
  P::parallelFor(executor, 0, 4, inherit, [&](int) {
    inherited.store(mkl_get_max_threads(), std::memory_order_relaxed);
  });
  EXPECT_EQ(inherited.load(std::memory_order_relaxed), initial);
  EXPECT_EQ(mkl_get_max_threads(), initial);
}
#endif

#ifdef PGO_PARALLELISM_HAS_ACCELERATE_THREADING
#  include <Accelerate/Accelerate.h>

TEST(ParallelNestedKernelTest, AccelerateSuppressAndInheritRestoreThreading)
{
  const auto initial = BLASGetThreading();
  if (BLASSetThreading(BLAS_THREADING_MULTI_THREADED) != 0)
    GTEST_SKIP() << "Accelerate threading control is unavailable.";

  const auto executor = P::runtime().createExecutor({ .maxConcurrency = 2 });
  std::atomic<int> suppressed = -1;
  P::parallelFor(executor, 0, 4, [&](int) {
    suppressed.store(static_cast<int>(BLASGetThreading()), std::memory_order_relaxed);
  });
  EXPECT_EQ(suppressed.load(std::memory_order_relaxed),
    static_cast<int>(BLAS_THREADING_SINGLE_THREADED));
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);

  P::Options inherit;
  inherit.nestedKernelPolicy = P::NestedKernelPolicy::Inherit;
  std::atomic<int> inherited = -1;
  P::parallelFor(executor, 0, 4, inherit, [&](int) {
    inherited.store(static_cast<int>(BLASGetThreading()), std::memory_order_relaxed);
  });
  EXPECT_EQ(inherited.load(std::memory_order_relaxed),
    static_cast<int>(BLAS_THREADING_MULTI_THREADED));
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);
  BLASSetThreading(initial);
}
#endif
