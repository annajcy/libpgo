#include <gtest/gtest.h>

#include "parallelism/parallelFor.h"

#include <atomic>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef PGO_PARALLELISM_HAS_TBB
#  include <tbb/global_control.h>
#endif

#ifdef PGO_PARALLELISM_HAS_EIGEN
#  include <Eigen/Core>
#endif

#ifdef PGO_PARALLELISM_HAS_MKL
#  include <mkl.h>
#endif

#ifdef PGO_PARALLELISM_HAS_ACCELERATE_THREADING
#  include <Accelerate/Accelerate.h>
#endif

namespace P = pgo::parallel;

namespace
{
bool hasObservableNestedRuntime()
{
#if defined(PGO_PARALLELISM_HAS_MKL) && !defined(PGO_PARALLELISM_MKL_TBB_THREADING)
  return true;
#else
  return false;
#endif
}

int observableNestedRuntimeThreads()
{
#if defined(PGO_PARALLELISM_HAS_MKL) && !defined(PGO_PARALLELISM_MKL_TBB_THREADING)
  return mkl_get_max_threads();
#else
  return 0;
#endif
}

P::Options optionsWithNestedPolicy(P::NestedKernelPolicy nestedKernelPolicy)
{
  P::Options options;
  options.nestedKernelPolicy = nestedKernelPolicy;
  return options;
}
}  // namespace

TEST(ParallelForTest, SerialLoopVisitsEveryIndex)
{
  std::vector<int> values(16, -1);

  P::parallelFor(0, static_cast<int>(values.size()), P::Options{}, [&](int i) {
    values[i] = i * i;
  });

  for (int i = 0; i < static_cast<int>(values.size()); ++i)
    EXPECT_EQ(values[i], i * i);
}

TEST(ParallelForTest, ParallelLoopVisitsEveryIndex)
{
  std::vector<int> values(1024, 0);

  P::parallelFor(0, static_cast<int>(values.size()), P::Options{}, [&](int i) {
    values[i] = i + 1;
  });

  for (int i = 0; i < static_cast<int>(values.size()); ++i)
    EXPECT_EQ(values[i], i + 1);
}

TEST(ParallelForTest, EmptyRangeDoesNothing)
{
  int calls = 0;
  P::parallelFor(5, 5, {}, [&](int) {
    ++calls;
  });
  EXPECT_EQ(calls, 0);
}

#ifdef PGO_PARALLELISM_HAS_TBB
TEST(ParallelForTest, DefaultSuppressesNestedRuntimeInTBBWorker)
{
  if (!hasObservableNestedRuntime())
    GTEST_SKIP() << "No observable nested runtime is available in this build.";

  P::ScopedWorkerLimit workerLimit(4);
  std::atomic<int> observed = 0;

  P::parallelFor(0, 8, P::Options{}, [&](int) {
    observed.store(observableNestedRuntimeThreads(), std::memory_order_relaxed);
  });

  EXPECT_EQ(observed.load(std::memory_order_relaxed), 1);
  EXPECT_EQ(observableNestedRuntimeThreads(), 4);
}

TEST(ParallelForTest, InheritLeavesNestedRuntimeInTBBWorker)
{
  if (!hasObservableNestedRuntime())
    GTEST_SKIP() << "No observable nested runtime is available in this build.";

  P::ScopedWorkerLimit workerLimit(4);
  std::atomic<int> observed = 0;

  P::parallelFor(0, 8, optionsWithNestedPolicy(P::NestedKernelPolicy::Inherit), [&](int) {
    observed.store(observableNestedRuntimeThreads(), std::memory_order_relaxed);
  });

  EXPECT_EQ(observed.load(std::memory_order_relaxed), 4);
  EXPECT_EQ(observableNestedRuntimeThreads(), 4);
}
#endif

#if defined(PGO_PARALLELISM_HAS_TBB) && defined(PGO_PARALLELISM_HAS_ACCELERATE_THREADING)
TEST(ParallelForTest, DefaultSuppressesAccelerateThreadingInTBBWorker)
{
  const auto initial = BLASGetThreading();
  if (BLASSetThreading(BLAS_THREADING_MULTI_THREADED) != 0)
    GTEST_SKIP() << "Accelerate threading control is not supported on this platform.";

  std::atomic<int> observed = -1;
  P::parallelFor(0, 8, P::Options{}, [&](int) {
    observed.store(static_cast<int>(BLASGetThreading()), std::memory_order_relaxed);
  });

  EXPECT_EQ(observed.load(std::memory_order_relaxed), static_cast<int>(BLAS_THREADING_SINGLE_THREADED));
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);

  BLASSetThreading(initial);
}

TEST(ParallelForTest, InheritLeavesAccelerateThreadingInTBBWorker)
{
  const auto initial = BLASGetThreading();
  if (BLASSetThreading(BLAS_THREADING_MULTI_THREADED) != 0)
    GTEST_SKIP() << "Accelerate threading control is not supported on this platform.";

  std::atomic<int> observed = -1;
  testing::internal::CaptureStderr();
  P::parallelFor(0, 8, optionsWithNestedPolicy(P::NestedKernelPolicy::Inherit), [&](int) {
    observed.store(static_cast<int>(BLASGetThreading()), std::memory_order_relaxed);
  });
  const std::string warning = testing::internal::GetCapturedStderr();

  EXPECT_EQ(observed.load(std::memory_order_relaxed), static_cast<int>(BLAS_THREADING_MULTI_THREADED));
  EXPECT_EQ(BLASGetThreading(), BLAS_THREADING_MULTI_THREADED);
  EXPECT_NE(warning.find("Accelerate threading enabled inside TBB workers"), std::string::npos);

  BLASSetThreading(initial);
}
#endif

TEST(ParallelFor3DTest, VisitsEveryCoordinateOnce)
{
  constexpr int nx = 3;
  constexpr int ny = 4;
  constexpr int nz = 2;
  std::vector<int> counts(nx * ny * nz, 0);

  P::parallelFor3D(nx, ny, nz, P::Options{}, [&](int x, int y, int z) {
    ASSERT_GE(x, 0);
    ASSERT_LT(x, nx);
    ASSERT_GE(y, 0);
    ASSERT_LT(y, ny);
    ASSERT_GE(z, 0);
    ASSERT_LT(z, nz);
    ++counts[x + nx * (y + ny * z)];
  });

  for (int count : counts)
    EXPECT_EQ(count, 1);
}

TEST(ParallelRuntimeTest, WorkerLimitControlsOptions)
{
  P::setWorkerLimit(std::nullopt);
  EXPECT_EQ(P::workerLimit(), std::nullopt);

  P::setWorkerLimit(3);
  EXPECT_EQ(P::workerLimit(), std::optional<int>(3));

  int calls = 0;
  P::parallelFor(0, 4, [&](int) {
    ++calls;
  });
  EXPECT_EQ(calls, 4);

  P::setWorkerLimit(std::nullopt);
}

TEST(ParallelRuntimeTest, RejectsInvalidLimits)
{
  EXPECT_THROW(P::setWorkerLimit(0), std::invalid_argument);
  EXPECT_THROW(P::setWorkerLimit(-1), std::invalid_argument);
  EXPECT_THROW(P::setCpuAffinityLimit(0), std::invalid_argument);
  EXPECT_THROW(P::setCpuAffinityLimit(-1), std::invalid_argument);
}

TEST(ParallelRuntimeTest, CpuAffinityLimitRestoresPreviousLimit)
{
  if (!P::supportsCpuAffinityLimit())
    GTEST_SKIP() << "CPU affinity limits are not supported on this platform.";

  P::setCpuAffinityLimit(std::nullopt);
  EXPECT_EQ(P::cpuAffinityLimit(), std::nullopt);

  P::setCpuAffinityLimit(1);
  EXPECT_EQ(P::cpuAffinityLimit(), std::optional<int>(1));

  {
    P::ScopedCpuAffinityLimit limit(std::nullopt);
    EXPECT_EQ(P::cpuAffinityLimit(), std::nullopt);
  }
  EXPECT_EQ(P::cpuAffinityLimit(), std::optional<int>(1));

  P::setCpuAffinityLimit(std::nullopt);
  EXPECT_EQ(P::cpuAffinityLimit(), std::nullopt);
}

#ifdef PGO_PARALLELISM_HAS_TBB
TEST(ParallelRuntimeTest, GlobalLimitControlsDirectTBBAndRestoresPreviousValue)
{
  const auto initial = tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism);

  P::setWorkerLimit(1);
  const auto limited = tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism);
  P::setWorkerLimit(std::nullopt);
  const auto restored = tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism);

  EXPECT_EQ(limited, 1u);
  EXPECT_EQ(restored, initial);
}

TEST(ParallelRuntimeTest, PerCallTBBLimitDoesNotChangeGlobalLimit)
{
  const auto initial = tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism);
  if (initial <= 1)
    GTEST_SKIP() << "TBB runtime exposes only one execution slot.";

  std::atomic<std::size_t> observed = 0;
  P::parallelFor(0, 4, P::Options{}, [&](int) {
    observed.store(tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism));
  });

  EXPECT_EQ(observed.load(), initial);
}
#endif

TEST(ParallelRuntimeTest, ScopedWorkerLimitRestoresPreviousLimit)
{
  P::setWorkerLimit(4);
  {
    P::ScopedWorkerLimit limit(1);
    EXPECT_EQ(P::workerLimit(), std::optional<int>(1));
  }
  EXPECT_EQ(P::workerLimit(), std::optional<int>(4));

  {
    P::ScopedWorkerLimit limit(std::nullopt);
    EXPECT_EQ(P::workerLimit(), std::nullopt);
  }
  EXPECT_EQ(P::workerLimit(), std::optional<int>(4));

  P::setWorkerLimit(std::nullopt);
}
