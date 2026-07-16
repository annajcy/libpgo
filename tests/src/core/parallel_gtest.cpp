#include <gtest/gtest.h>

#include "parallel/parallel.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <thread>
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

#if defined(PGO_HAS_MKL) || defined(__APPLE__)

int currentBackendThreadingValue()
{
#  if defined(PGO_HAS_MKL)
  const int current = mkl_set_num_threads_local(0);
  mkl_set_num_threads_local(current);
  return current;
#  else
  return static_cast<int>(BLASGetThreading());
#  endif
}

void setBackendThreadingValue(int value)
{
#  if defined(PGO_HAS_MKL)
  mkl_set_num_threads_local(value);
#  else
  if (BLASSetThreading(static_cast<BLAS_THREADING>(value)) != 0)
    throw std::runtime_error("Accelerate rejected a test threading mode.");
#  endif
}

P::ThreadingPolicy backendPolicy(int value)
{
#  if defined(PGO_HAS_MKL)
  return { .mklLocalThreadBudget = value };
#  else
  return { .accelerate = value == static_cast<int>(BLAS_THREADING_SINGLE_THREADED) ?
      P::AccelerateThreading::single :
      P::AccelerateThreading::multi };
#  endif
}

int baselineBackendThreadingValue()
{
#  if defined(PGO_HAS_MKL)
  return 3;
#  else
  return static_cast<int>(BLAS_THREADING_MULTI_THREADED);
#  endif
}

int outerBackendThreadingValue()
{
#  if defined(PGO_HAS_MKL)
  return 1;
#  else
  return static_cast<int>(BLAS_THREADING_SINGLE_THREADED);
#  endif
}

int innerBackendThreadingValue()
{
#  if defined(PGO_HAS_MKL)
  return 2;
#  else
  return static_cast<int>(BLAS_THREADING_MULTI_THREADED);
#  endif
}

bool waitUntilTrue(const std::atomic<bool> &value,
  std::chrono::steady_clock::duration timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!value.load(std::memory_order_acquire)) {
    if (std::chrono::steady_clock::now() >= deadline)
      return false;
    std::this_thread::yield();
  }
  return true;
}

class RestoreBackendThreadingValue
{
public:
  RestoreBackendThreadingValue(): previous_(currentBackendThreadingValue()) {}
  ~RestoreBackendThreadingValue() { setBackendThreadingValue(previous_); }

private:
  int previous_;
};

#endif

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
      if (currentBackendThreadingValue() != 1)
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

#if defined(PGO_HAS_MKL) || defined(__APPLE__)

TEST(ArenaThreadingExecutorTest, RepeatedExecuteScopesAndRestoresCallingThreadPolicy)
{
  RestoreBackendThreadingValue restore;
  const int baseline = baselineBackendThreadingValue();
  const int policyValue = outerBackendThreadingValue();
  setBackendThreadingValue(baseline);

  {
    P::ArenaThreadingExecutor executor(2, backendPolicy(policyValue));
    for (int iteration = 0; iteration < 3; ++iteration) {
      executor.execute([&] {
        EXPECT_EQ(currentBackendThreadingValue(), policyValue);
        tbb::parallel_for(0, 128, [&](int) {
          EXPECT_EQ(currentBackendThreadingValue(), policyValue);
        });
      });
      EXPECT_EQ(currentBackendThreadingValue(), baseline);
    }
  }

  EXPECT_EQ(currentBackendThreadingValue(), baseline);
  EXPECT_TRUE(P::drainRetiredArenaThreadingExecutorStates(
    std::chrono::seconds(5)));
}

TEST(ArenaThreadingExecutorTest, NestedExecutorsRestorePoliciesInLifoOrder)
{
  RestoreBackendThreadingValue restore;
  const int baseline = baselineBackendThreadingValue();
  const int outerValue = outerBackendThreadingValue();
  const int innerValue = innerBackendThreadingValue();
  setBackendThreadingValue(baseline);

  {
    P::ArenaThreadingExecutor outer(2, backendPolicy(outerValue));
    outer.execute([&] {
      EXPECT_EQ(currentBackendThreadingValue(), outerValue);
      {
        P::ArenaThreadingExecutor inner(2, backendPolicy(innerValue));
        inner.execute([&] {
          EXPECT_EQ(currentBackendThreadingValue(), innerValue);
          tbb::parallel_for(0, 128, [&](int) {
            EXPECT_EQ(currentBackendThreadingValue(), innerValue);
          });
        });
      }
      EXPECT_EQ(currentBackendThreadingValue(), outerValue);
    });
  }

  EXPECT_EQ(currentBackendThreadingValue(), baseline);
  EXPECT_TRUE(P::drainRetiredArenaThreadingExecutorStates(
    std::chrono::seconds(5)));
}

TEST(ArenaThreadingExecutorTest, ConcurrentNestedParticipantsRestoreOuterPolicy)
{
  RestoreBackendThreadingValue restore;
  const int baseline = baselineBackendThreadingValue();
  const int outerValue = outerBackendThreadingValue();
  const int innerValue = innerBackendThreadingValue();
  setBackendThreadingValue(baseline);

  {
    P::ArenaThreadingExecutor outer(4, backendPolicy(outerValue));
    P::ArenaThreadingExecutor inner(2, backendPolicy(innerValue));
    tbb::enumerable_thread_specific<bool> visited(false);
    std::atomic<bool> mismatch{ false };

    outer.execute([&] {
      tbb::parallel_for(0, 4096, [&](int) {
        if (currentBackendThreadingValue() != outerValue)
          mismatch.store(true, std::memory_order_relaxed);

        bool &didVisitInner = visited.local();
        if (didVisitInner)
          return;
        didVisitInner = true;

        inner.execute([&] {
          if (currentBackendThreadingValue() != innerValue)
            mismatch.store(true, std::memory_order_relaxed);
        });
        if (currentBackendThreadingValue() != outerValue)
          mismatch.store(true, std::memory_order_relaxed);
      });
    });

    EXPECT_FALSE(mismatch.load(std::memory_order_relaxed));
    EXPECT_GE(visited.size(), 1U);
  }

  EXPECT_EQ(currentBackendThreadingValue(), baseline);
  EXPECT_TRUE(P::drainRetiredArenaThreadingExecutorStates(
    std::chrono::seconds(5)));
}

TEST(ArenaThreadingExecutorTest, ExceptionRestoresPolicyAndExecutorRemainsReusable)
{
  RestoreBackendThreadingValue restore;
  const int baseline = baselineBackendThreadingValue();
  const int policyValue = outerBackendThreadingValue();
  setBackendThreadingValue(baseline);

  {
    P::ArenaThreadingExecutor executor(2, backendPolicy(policyValue));
    EXPECT_THROW(executor.execute([&] {
      EXPECT_EQ(currentBackendThreadingValue(), policyValue);
      throw std::runtime_error("expected test exception");
    }),
      std::runtime_error);
    EXPECT_EQ(currentBackendThreadingValue(), baseline);

    executor.execute([&] {
      EXPECT_EQ(currentBackendThreadingValue(), policyValue);
    });
    EXPECT_EQ(currentBackendThreadingValue(), baseline);
  }

  EXPECT_TRUE(P::drainRetiredArenaThreadingExecutorStates(
    std::chrono::seconds(5)));
}

TEST(ArenaThreadingExecutorTest, RetiredObserverCoversAttachedArenaUntilFinalDetach)
{
  RestoreBackendThreadingValue restore;
  const int baseline = baselineBackendThreadingValue();
  const int policyValue = outerBackendThreadingValue();
  setBackendThreadingValue(baseline);
  ASSERT_TRUE(P::drainRetiredArenaThreadingExecutorStates(
    std::chrono::seconds(5)));

  std::unique_ptr<tbb::task_arena> attached;
  {
    P::ArenaThreadingExecutor executor(2, backendPolicy(policyValue));
    executor.execute([&] {
      attached = std::make_unique<tbb::task_arena>(tbb::attach{});
    });
  }

  ASSERT_NE(attached, nullptr);
  EXPECT_EQ(P::retiredArenaThreadingExecutorStateCount(), 1U);
  EXPECT_FALSE(P::drainRetiredArenaThreadingExecutorStates(
    std::chrono::milliseconds(0)));
  attached->execute([&] {
    EXPECT_EQ(currentBackendThreadingValue(), policyValue);
    tbb::parallel_for(0, 128, [&](int) {
      EXPECT_EQ(currentBackendThreadingValue(), policyValue);
    });
  });
  EXPECT_EQ(currentBackendThreadingValue(), baseline);

  attached.reset();
  EXPECT_TRUE(P::drainRetiredArenaThreadingExecutorStates(
    std::chrono::seconds(5)));
  EXPECT_EQ(P::retiredArenaThreadingExecutorStateCount(), 0U);
}

TEST(ArenaThreadingExecutorTest, DestructionDefersObserverUntilActiveParticipantExits)
{
  RestoreBackendThreadingValue restore;
  const int baseline = baselineBackendThreadingValue();
  const int policyValue = outerBackendThreadingValue();
  setBackendThreadingValue(baseline);
  ASSERT_TRUE(P::drainRetiredArenaThreadingExecutorStates(
    std::chrono::seconds(5)));

  std::atomic<bool> release{ false };
  std::atomic<bool> started{ false };
  std::atomic<bool> finished{ false };
  std::atomic<bool> mismatch{ false };
  auto executor = std::make_unique<P::ArenaThreadingExecutor>(
    2, backendPolicy(policyValue));
  std::unique_ptr<tbb::task_arena> attached;
  executor->execute([&] {
    attached = std::make_unique<tbb::task_arena>(tbb::attach{});
  });
  attached->enqueue([&] {
    if (currentBackendThreadingValue() != policyValue)
      mismatch.store(true, std::memory_order_relaxed);
    started.store(true, std::memory_order_release);
    while (!release.load(std::memory_order_acquire))
      std::this_thread::yield();
    if (currentBackendThreadingValue() != policyValue)
      mismatch.store(true, std::memory_order_relaxed);
    finished.store(true, std::memory_order_release);
  });

  if (!waitUntilTrue(started, std::chrono::seconds(5))) {
    release.store(true, std::memory_order_release);
    attached.reset();
    executor.reset();
    EXPECT_TRUE(P::drainRetiredArenaThreadingExecutorStates(
      std::chrono::seconds(5)));
    FAIL() << "The enqueued arena participant did not start.";
  }

  attached.reset();
  executor.reset();
  EXPECT_EQ(P::retiredArenaThreadingExecutorStateCount(), 1U);
  EXPECT_FALSE(P::drainRetiredArenaThreadingExecutorStates(
    std::chrono::milliseconds(0)));

  release.store(true, std::memory_order_release);
  EXPECT_TRUE(waitUntilTrue(finished, std::chrono::seconds(5)));
  EXPECT_FALSE(mismatch.load(std::memory_order_relaxed));
  EXPECT_TRUE(P::drainRetiredArenaThreadingExecutorStates(
    std::chrono::seconds(5)));
  EXPECT_EQ(P::retiredArenaThreadingExecutorStateCount(), 0U);
  EXPECT_EQ(currentBackendThreadingValue(), baseline);
}

#endif

TEST(ArenaThreadingExecutorTest, RepeatedRetirementEventuallyDrainsWithoutBacklog)
{
  ASSERT_TRUE(P::drainRetiredArenaThreadingExecutorStates(
    std::chrono::seconds(5)));

  for (int iteration = 0; iteration < 1000; ++iteration) {
    P::ArenaThreadingExecutor executor(2, completeThreadingPolicy());
    executor.execute([] {
      tbb::parallel_for(0, 8, [](int) {});
    });
  }

  EXPECT_TRUE(P::drainRetiredArenaThreadingExecutorStates(
    std::chrono::seconds(5)));
  EXPECT_EQ(P::retiredArenaThreadingExecutorStateCount(), 0U);
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

TEST(ParallelAccelerateTest, ArenaExecutorAppliesAndRestoresPolicy)
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
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);

  {
    P::ArenaThreadingExecutor multiExecutor(
      2, { .accelerate = P::AccelerateThreading::multi });
    multiExecutor.execute([] {
      EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);
    });
  }
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);
  EXPECT_TRUE(P::drainRetiredArenaThreadingExecutorStates(
    std::chrono::seconds(5)));
}

#endif
