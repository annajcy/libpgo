#include <gtest/gtest.h>

#include "parallel/parallel.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include <tbb/enumerable_thread_specific.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>

#if defined(PGO_HAS_MKL)
#  include <mkl.h>
#elif defined(__APPLE__)
#  include <Accelerate/Accelerate.h>
#endif

namespace P = pgo::parallel;

namespace
{

P::ThreadingPolicy completeThreadingPolicy()
{
#if defined(PGO_HAS_MKL)
  return { .mklLocalThreadBudget = 1 };
#elif defined(__APPLE__)
  return { .accelerate = P::AccelerateThreading::single };
#else
  return {};
#endif
}

}  // namespace

TEST(GlobalTbbControlTest, ValidatesAndUsesOneTbbRaiiSemantics)
{
  EXPECT_THROW(P::GlobalTbbControl(0), std::invalid_argument);
  EXPECT_THROW(P::GlobalTbbControl(-1), std::invalid_argument);

  P::GlobalTbbControl outer(4);
  EXPECT_LE(tbb::global_control::active_value(
              tbb::global_control::max_allowed_parallelism),
    4U);
  {
    P::GlobalTbbControl inner(2);
    EXPECT_LE(tbb::global_control::active_value(
                tbb::global_control::max_allowed_parallelism),
      2U);
  }
  EXPECT_LE(tbb::global_control::active_value(
              tbb::global_control::max_allowed_parallelism),
    4U);
}

TEST(ParallelControlTest, NonExpandingResolutionUsesCurrentArenaOnly)
{
  EXPECT_THROW(P::resolveNonExpandingTbbConcurrency(0), std::invalid_argument);
  EXPECT_THROW(P::resolveNonExpandingTbbConcurrency(-1), std::invalid_argument);

  tbb::task_arena arena(2, 1);
  arena.execute([] {
    EXPECT_EQ(P::resolveNonExpandingTbbConcurrency(1), 1);
    EXPECT_EQ(P::resolveNonExpandingTbbConcurrency(8), 2);
  });
}

TEST(ThreadingPolicyTest, RejectsNegativeMklBudget)
{
  EXPECT_THROW(P::setThreadingPolicy({ .mklLocalThreadBudget = -1 }),
    std::invalid_argument);
  EXPECT_NO_THROW(P::setThreadingPolicy({ .mklLocalThreadBudget = 0 }));
}

#if defined(PGO_HAS_MKL)
namespace
{

class RestoreMklLocalThreadBudget
{
public:
  RestoreMklLocalThreadBudget(): previous_(mkl_set_num_threads_local(0)) {}
  ~RestoreMklLocalThreadBudget() { mkl_set_num_threads_local(previous_); }

private:
  int previous_;
};

}  // namespace

TEST(ParallelMklTest, CurrentThreadSetterIsStickyAndZeroClearsOverride)
{
  RestoreMklLocalThreadBudget restore;

  P::setThreadingPolicy({ .mklLocalThreadBudget = 3 });
  EXPECT_EQ(mkl_set_num_threads_local(0), 3);

  P::setThreadingPolicy({ .mklLocalThreadBudget = 2 });
  P::setThreadingPolicy({ .mklLocalThreadBudget = 0 });
  EXPECT_EQ(mkl_set_num_threads_local(0), 0);
}

TEST(ParallelMklTest, ArenaExecutorAppliesBudgetToEveryObservedParticipant)
{
  RestoreMklLocalThreadBudget restore;
  P::ArenaThreadingExecutor executor(
    2, { .mklLocalThreadBudget = 1 });

  std::atomic<bool> mismatch{ false };
  tbb::enumerable_thread_specific<bool> checked(false);
  executor.execute([&] {
    tbb::parallel_for(0, 1024, [&](int) {
      bool &wasChecked = checked.local();
      if (wasChecked)
        return;
      wasChecked = true;
      if (mkl_set_num_threads_local(0) != 1)
        mismatch.store(true, std::memory_order_relaxed);
    });
  });

  EXPECT_FALSE(mismatch.load(std::memory_order_relaxed));
  EXPECT_GE(checked.size(), 1U);
}
#endif

TEST(ArenaThreadingExecutorTest, OwnsAndReusesArenaForNativeTbbAlgorithms)
{
  P::ArenaThreadingExecutor executor(2, completeThreadingPolicy());
  EXPECT_EQ(executor.maxConcurrency(), 2);

  std::vector<int> values(128, 0);
  for (int iteration = 0; iteration < 2; ++iteration) {
    executor.execute([&] {
      EXPECT_EQ(tbb::this_task_arena::max_concurrency(), 2);
      tbb::parallel_for(std::size_t{ 0 }, values.size(), [&](std::size_t i) {
        values[i] += 1;
      });
    });
  }
  EXPECT_TRUE(std::all_of(values.begin(), values.end(),
    [](int value) { return value == 2; }));

  const int total = executor.execute([] {
    return tbb::parallel_reduce(
      tbb::blocked_range<int>(0, 100), 0,
      [](const tbb::blocked_range<int> &range, int local) {
        for (int i = range.begin(); i < range.end(); ++i)
          local += i;
        return local;
      },
      std::plus<int>{});
  });
  EXPECT_EQ(total, 4950);

  executor.execute([&] { tbb::parallel_sort(values.begin(), values.end()); });
  EXPECT_TRUE(std::is_sorted(values.begin(), values.end()));
}

#if defined(__APPLE__)
namespace
{

class RestoreAccelerateThreading
{
public:
  RestoreAccelerateThreading(): previous_(BLASGetThreading()) {}
  ~RestoreAccelerateThreading() { BLASSetThreading(previous_); }

private:
  BLAS_THREADING previous_;
};

}  // namespace

TEST(ParallelAccelerateTest, CurrentThreadSetterIsSticky)
{
  RestoreAccelerateThreading restore;
  ASSERT_EQ(BLASSetThreading(BLAS_THREADING_MULTI_THREADED), 0);

  P::setThreadingPolicy({ .accelerate = P::AccelerateThreading::single });
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_SINGLE_THREADED);

  P::setThreadingPolicy({ .accelerate = P::AccelerateThreading::multi });
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);
}

TEST(ParallelAccelerateTest, ArenaExecutorAppliesPolicyWithoutRestore)
{
  RestoreAccelerateThreading restore;
  ASSERT_EQ(BLASSetThreading(BLAS_THREADING_MULTI_THREADED), 0);

  {
    P::ArenaThreadingExecutor singleExecutor(
      2, { .accelerate = P::AccelerateThreading::single });
    singleExecutor.execute([] {
      EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_SINGLE_THREADED);
      tbb::parallel_for(0, 32, [](int) {
        EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_SINGLE_THREADED);
      });
    });
  }
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_SINGLE_THREADED);

  {
    P::ArenaThreadingExecutor multiExecutor(
      2, { .accelerate = P::AccelerateThreading::multi });
    multiExecutor.execute([] {
      EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);
    });
  }
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);
}

#endif
