#include <gtest/gtest.h>

#include "parallel/parallel.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

#if defined(__APPLE__)
#  include <Accelerate/Accelerate.h>
#endif

namespace P = pgo::parallel;

TEST(ParallelControlTest, ConfigurationIsRepeatableAndGlobalArenaIsAligned)
{
  const int first = P::initialize(2);
  EXPECT_GE(first, 1);
  EXPECT_LE(first, 2);

  const int second = P::initialize(4);
  EXPECT_GE(second, 1);
  EXPECT_LE(second, 4);
  EXPECT_EQ(P::withGlobalTbbConcurrency(
              [] { return tbb::this_task_arena::max_concurrency(); }),
    second);

  const int restored = P::initialize();
  EXPECT_GE(restored, 1);
}

TEST(ParallelControlTest, LexicalBoundsAreMonotonicAndInheritIntoPgoAndTbb)
{
  const int global = P::initialize(4);
  const int outer = std::min(2, global);
  const int nested = P::withTbbConcurrencyLimit(outer, [&] {
    EXPECT_EQ(tbb::this_task_arena::max_concurrency(), outer);
    return P::withTbbConcurrencyLimit(global + 1, [] {
      return tbb::this_task_arena::max_concurrency();
    });
  });
  EXPECT_EQ(nested, outer);

  std::atomic<int> pgoArena{ 0 };
  std::atomic<int> directTbbArena{ 0 };
  P::withTbbConcurrencyLimit(outer, [&] {
    P::parallelFor(0, 8, [&](int) { pgoArena.store(tbb::this_task_arena::max_concurrency(), std::memory_order_relaxed); }, std::optional<int>{ 1 });
    tbb::parallel_for(0, 8, [&](int) {
      directTbbArena.store(tbb::this_task_arena::max_concurrency(), std::memory_order_relaxed);
    });
  });
  EXPECT_EQ(pgoArena.load(std::memory_order_relaxed), outer);
  EXPECT_EQ(directTbbArena.load(std::memory_order_relaxed), outer);
}

TEST(ParallelControlTest, BoundsValidateAndRestoreAfterExceptions)
{
  EXPECT_THROW(P::withTbbConcurrencyLimit(0, [] {}), std::invalid_argument);
  EXPECT_THROW(P::withTbbConcurrencyLimit(-1, [] {}), std::invalid_argument);

  const int before = tbb::this_task_arena::max_concurrency();
  EXPECT_THROW(P::withSingleThreadedTbb([] { throw std::runtime_error("expected"); }),
    std::runtime_error);
  EXPECT_EQ(tbb::this_task_arena::max_concurrency(), before);
}

TEST(ParallelFacadeTest, SupportsChunksPartitionersReductionAndSort)
{
  std::vector<std::atomic<int>> visits(97);
  for (bool useStatic : { false, true }) {
    for (auto &visit : visits)
      visit.store(0, std::memory_order_relaxed);
    auto body = [&](std::size_t begin, std::size_t end) {
      for (std::size_t i = begin; i < end; ++i)
        visits[i].fetch_add(1, std::memory_order_relaxed);
    };
    if (useStatic)
      P::parallelForChunks(std::size_t{ 0 }, visits.size(), body,
        std::optional<int>{ 7 }, tbb::static_partitioner{});
    else
      P::parallelForChunks(std::size_t{ 0 }, visits.size(), body, std::optional<int>{ 7 });
    for (const auto &visit : visits)
      EXPECT_EQ(visit.load(std::memory_order_relaxed), 1);
  }

  EXPECT_EQ(P::parallelReduce(0, 100, 0, [](int begin, int end, int local) {
                for (int i = begin; i < end; ++i)
                  local += i;
                return local; }, std::plus<int>{}), 4950);

  std::vector<int> values(2048);
  for (std::size_t i = 0; i < values.size(); ++i)
    values[i] = static_cast<int>(values.size() - i);
  P::parallelSort(values.begin(), values.end());
  EXPECT_TRUE(std::is_sorted(values.begin(), values.end()));

  std::atomic<std::uint64_t> integerIndexTotal{ 0 };
  P::parallelFor(std::uint64_t{ 0 }, std::uint64_t{ 64 }, [&](std::uint64_t i) {
    integerIndexTotal.fetch_add(i, std::memory_order_relaxed);
  });
  EXPECT_EQ(integerIndexTotal.load(std::memory_order_relaxed), 2016);

  EXPECT_THROW(P::parallelFor(0, 1, [](int) {}, std::optional<int>{ -1 }), std::invalid_argument);
}

#if defined(__APPLE__)
namespace
{

class RestoreAccelerateThreading
{
public:
  RestoreAccelerateThreading(): previous_(BLASGetThreading()) {}
  ~RestoreAccelerateThreading()
  {
    BLASSetThreading(previous_);
  }

private:
  BLAS_THREADING previous_;
};

}  // namespace

TEST(ParallelAccelerateTest, InitializeAndPgoCallbacksLeaveCurrentThreadSingle)
{
  RestoreAccelerateThreading restore;
  ASSERT_EQ(BLASSetThreading(BLAS_THREADING_MULTI_THREADED), 0);

  P::initialize(2);
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_SINGLE_THREADED);

  ASSERT_EQ(BLASSetThreading(BLAS_THREADING_MULTI_THREADED), 0);

  P::withTbbConcurrencyLimit(1, [] {
    EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);
  });

  std::atomic<int> singleThreadedCallbacks{ 0 };
  P::withTbbConcurrencyLimit(1, [&] {
    P::parallelFor(0, 4, [&](int) {
      if (BLASGetThreading() == BLAS_THREADING_SINGLE_THREADED)
        singleThreadedCallbacks.fetch_add(1, std::memory_order_relaxed);
      P::parallelFor(0, 2, [&](int) {
        if (BLASGetThreading() == BLAS_THREADING_SINGLE_THREADED)
          singleThreadedCallbacks.fetch_add(1, std::memory_order_relaxed);
      });
    });
  });
  EXPECT_EQ(singleThreadedCallbacks.load(std::memory_order_relaxed), 12);
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_SINGLE_THREADED);

  ASSERT_EQ(BLASSetThreading(BLAS_THREADING_MULTI_THREADED), 0);
  std::atomic<bool> reduceCallbacksWereSingle{ true };
  const int total = P::withTbbConcurrencyLimit(2, [&] {
    return P::parallelReduce(
      0, 128, 0,
      [&](int begin, int end, int local) {
        if (BLASGetThreading() != BLAS_THREADING_SINGLE_THREADED)
          reduceCallbacksWereSingle.store(false, std::memory_order_relaxed);
        for (int i = begin; i < end; ++i)
          local += i;
        return local;
      },
      [&](int left, int right) {
        if (BLASGetThreading() != BLAS_THREADING_SINGLE_THREADED)
          reduceCallbacksWereSingle.store(false, std::memory_order_relaxed);
        return left + right;
      },
      std::optional<int>{ 1 }, tbb::static_partitioner{});
  });
  EXPECT_EQ(total, 8128);
  EXPECT_TRUE(reduceCallbacksWereSingle.load(std::memory_order_relaxed));
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_SINGLE_THREADED);

  ASSERT_EQ(BLASSetThreading(BLAS_THREADING_MULTI_THREADED), 0);
  EXPECT_THROW(P::withTbbConcurrencyLimit(1, [] {
    P::parallelFor(0, 1, [](int) { throw std::runtime_error("expected"); });
  }),
    std::runtime_error);
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_SINGLE_THREADED);
}

TEST(ParallelAccelerateTest, ExperimentalMultiRestoresSingleAfterReturnAndException)
{
  RestoreAccelerateThreading restore;
  ASSERT_EQ(BLASSetThreading(BLAS_THREADING_SINGLE_THREADED), 0);

  const int result = P::experimental::withMultiThreadedAccelerate([] {
    EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);
    return P::experimental::withMultiThreadedAccelerate([] {
      EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);
      return 42;
    });
  });
  EXPECT_EQ(result, 42);
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_SINGLE_THREADED);

  EXPECT_THROW(P::experimental::withMultiThreadedAccelerate([] {
    EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);
    throw std::runtime_error("expected");
  }),
    std::runtime_error);
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_SINGLE_THREADED);

  ASSERT_EQ(BLASSetThreading(BLAS_THREADING_MULTI_THREADED), 0);
  P::experimental::withMultiThreadedAccelerate([] {
    EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);
  });
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);
}

TEST(ParallelAccelerateTest, TbbControlsAndSortComparatorsAreBlasNeutral)
{
  RestoreAccelerateThreading restore;
  ASSERT_EQ(BLASSetThreading(BLAS_THREADING_MULTI_THREADED), 0);

  std::vector<int> values(2048);
  for (std::size_t i = 0; i < values.size(); ++i)
    values[i] = static_cast<int>(values.size() - i);

  std::atomic<int> comparisons{ 0 };
  std::atomic<bool> comparatorsKeptAmbient{ true };
  P::withTbbConcurrencyLimit(1, [&] {
    P::parallelSort(values.begin(), values.end(), [&](int left, int right) {
      comparisons.fetch_add(1, std::memory_order_relaxed);
      if (BLASGetThreading() != BLAS_THREADING_MULTI_THREADED)
        comparatorsKeptAmbient.store(false, std::memory_order_relaxed);
      return left < right;
    });
  });

  EXPECT_TRUE(std::is_sorted(values.begin(), values.end()));
  EXPECT_GT(comparisons.load(std::memory_order_relaxed), 0);
  EXPECT_TRUE(comparatorsKeptAmbient.load(std::memory_order_relaxed));
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);
}
#endif
