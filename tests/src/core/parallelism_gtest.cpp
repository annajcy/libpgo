#include <gtest/gtest.h>

#include "parallelism/parallelFor.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifdef __linux__
#  include <dirent.h>
#  include <unistd.h>
#endif

#ifdef __APPLE__
#  include <mach/mach.h>
#  include <unistd.h>
#endif

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

#if defined(__linux__) || defined(__APPLE__)
int currentProcessThreadCount()
{
#ifdef __linux__
  DIR *dir = opendir("/proc/self/task");
  if (!dir)
    return 0;

  int count = 0;
  while (dirent *entry = readdir(dir)) {
    if (entry->d_name[0] != '.')
      ++count;
  }
  closedir(dir);
  return count;
#else
  thread_act_array_t threads = nullptr;
  mach_msg_type_number_t count = 0;
  if (task_threads(mach_task_self(), &threads, &count) != KERN_SUCCESS)
    return 0;

  vm_deallocate(mach_task_self(), reinterpret_cast<vm_address_t>(threads), count * sizeof(thread_t));
  return static_cast<int>(count);
#endif
}

template<class Fn>
int peakThreadCountWhile(Fn &&fn)
{
  std::atomic<bool> done = false;
  std::atomic<int> peak = currentProcessThreadCount();
  std::thread sampler([&] {
    while (!done.load(std::memory_order_acquire)) {
      peak.store(std::max(peak.load(std::memory_order_relaxed), currentProcessThreadCount()),
        std::memory_order_relaxed);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  fn();
  done.store(true, std::memory_order_release);
  sampler.join();
  return peak.load(std::memory_order_relaxed);
}
#endif

#ifdef PGO_PARALLELISM_HAS_ACCELERATE_THREADING
void runAccelerateDgemm()
{
  constexpr int n = 384;
  std::vector<double> a(n * n, 1.0);
  std::vector<double> b(n * n, 2.0);
  std::vector<double> c(n * n, 0.0);
  for (int repeat = 0; repeat < 3; ++repeat) {
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#endif
    cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
      n, n, n, 1.0, a.data(), n, b.data(), n, 0.0, c.data(), n);
#ifdef __clang__
#pragma clang diagnostic pop
#endif
  }
}
#endif

#ifdef PGO_PARALLELISM_HAS_MKL
void runMklDgemm()
{
  constexpr int n = 384;
  std::vector<double> a(n * n, 1.0);
  std::vector<double> b(n * n, 2.0);
  std::vector<double> c(n * n, 0.0);
  for (int repeat = 0; repeat < 3; ++repeat) {
    cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
      n, n, n, 1.0, a.data(), n, b.data(), n, 0.0, c.data(), n);
  }
}
#endif
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

TEST(ParallelForTest, SuppressAvoidsAccelerateThreadOversubscriptionInTBBWorker)
{
  const auto initial = BLASGetThreading();
  if (BLASSetThreading(BLAS_THREADING_MULTI_THREADED) != 0)
    GTEST_SKIP() << "Accelerate threading control is not supported on this platform.";

  P::ScopedWorkerLimit workerLimit(2);
  const int baseline = currentProcessThreadCount();
  const int peak = peakThreadCountWhile([] {
    P::parallelFor(0, 2, P::Options{}, [](int) {
      runAccelerateDgemm();
    });
  });

  EXPECT_LE(peak, baseline + 6);

  BLASSetThreading(initial);
}
#endif

#if defined(__linux__) && defined(PGO_PARALLELISM_HAS_TBB) && defined(PGO_PARALLELISM_HAS_MKL) && defined(PGO_PARALLELISM_MKL_TBB_THREADING)
TEST(ParallelForTest, MklTbbThreadingDoesNotOversubscribeInSuppressOrInherit)
{
  P::ScopedWorkerLimit workerLimit(2);
  const int baseline = currentProcessThreadCount();

  const int suppressPeak = peakThreadCountWhile([] {
    P::parallelFor(0, 2, P::Options{}, [](int) {
      runMklDgemm();
    });
  });
  EXPECT_LE(suppressPeak, baseline + 6);

  const int inheritPeak = peakThreadCountWhile([] {
    P::parallelFor(0, 2, optionsWithNestedPolicy(P::NestedKernelPolicy::Inherit), [](int) {
      runMklDgemm();
    });
  });
  EXPECT_LE(inheritPeak, baseline + 6);
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
