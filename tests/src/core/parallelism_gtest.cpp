#include <gtest/gtest.h>

#include "parallelism/parallelFor.h"

#include <atomic>
#include <optional>
#include <stdexcept>
#include <vector>

#ifdef PGO_PARALLELISM_HAS_TBB
#  include <tbb/global_control.h>
#endif

#ifdef PGO_PARALLELISM_HAS_EIGEN
#  include <Eigen/Core>
#endif

#ifdef PGO_PARALLELISM_HAS_OPENMP
#  include <omp.h>
#endif

#ifdef PGO_PARALLELISM_HAS_MKL
#  include <mkl.h>
#endif

#ifdef PGO_PARALLELISM_HAS_OPENBLAS
extern "C" int openblas_get_num_threads();
#endif

namespace P = pgo::parallel;

namespace
{
bool hasObservableNestedRuntime()
{
#if defined(PGO_PARALLELISM_HAS_OPENMP) || (defined(PGO_PARALLELISM_HAS_MKL) && !defined(PGO_PARALLELISM_MKL_TBB_THREADING))
  return true;
#else
  return false;
#endif
}

int observableNestedRuntimeThreads()
{
#ifdef PGO_PARALLELISM_HAS_OPENMP
  return omp_get_max_threads();
#elif defined(PGO_PARALLELISM_HAS_MKL) && !defined(PGO_PARALLELISM_MKL_TBB_THREADING)
  return mkl_get_max_threads();
#else
  return 0;
#endif
}

P::Options backendOptions(P::Backend backend, P::NestedKernelPolicy nestedKernelPolicy)
{
  P::Options options;
  options.backend = backend;
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

  P::ScopedThreadLimit threadLimit(4);
  std::atomic<int> observed = 0;

  P::Options options;
  options.backend = P::Backend::TBB;
  P::parallelFor(0, 8, options, [&](int) {
    observed.store(observableNestedRuntimeThreads(), std::memory_order_relaxed);
  });

  EXPECT_EQ(observed.load(std::memory_order_relaxed), 1);
  EXPECT_EQ(observableNestedRuntimeThreads(), 4);
}

TEST(ParallelForTest, InheritLeavesNestedRuntimeInTBBWorker)
{
  if (!hasObservableNestedRuntime())
    GTEST_SKIP() << "No observable nested runtime is available in this build.";

  P::ScopedThreadLimit threadLimit(4);
  std::atomic<int> observed = 0;

  P::parallelFor(0, 8, backendOptions(P::Backend::TBB, P::NestedKernelPolicy::Inherit), [&](int) {
    observed.store(observableNestedRuntimeThreads(), std::memory_order_relaxed);
  });

  EXPECT_EQ(observed.load(std::memory_order_relaxed), 4);
  EXPECT_EQ(observableNestedRuntimeThreads(), 4);
}
#endif

#ifdef PGO_PARALLELISM_HAS_OPENMP
TEST(ParallelForTest, DefaultSuppressesNestedRuntimeInOpenMPWorker)
{
  P::ScopedThreadLimit threadLimit(4);
  std::atomic<int> observed = 0;

  P::parallelFor(0, 8, backendOptions(P::Backend::OpenMP, P::NestedKernelPolicy::Suppress), [&](int) {
    observed.store(observableNestedRuntimeThreads(), std::memory_order_relaxed);
  });

  EXPECT_EQ(observed.load(std::memory_order_relaxed), 1);
  EXPECT_EQ(observableNestedRuntimeThreads(), 4);
}

TEST(ParallelForTest, InheritLeavesNestedRuntimeInOpenMPWorker)
{
  P::ScopedThreadLimit threadLimit(4);
  std::atomic<int> observed = 0;

  P::parallelFor(0, 8, backendOptions(P::Backend::OpenMP, P::NestedKernelPolicy::Inherit), [&](int) {
    observed.store(observableNestedRuntimeThreads(), std::memory_order_relaxed);
  });

  EXPECT_EQ(observed.load(std::memory_order_relaxed), 4);
  EXPECT_EQ(observableNestedRuntimeThreads(), 4);
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

TEST(ParallelBackendTest, ExplicitUnavailableBackendThrows)
{
  const P::Backend backend = P::isBackendAvailable(P::Backend::OpenMP) ? P::Backend::TBB : P::Backend::OpenMP;
  if (P::isBackendAvailable(backend))
    GTEST_SKIP() << "Both TBB and OpenMP are available in this build.";

  EXPECT_THROW(
    P::parallelFor(0, 4, { .backend = backend }, [](int) {}),
    std::runtime_error);
}

TEST(ParallelRuntimeTest, ThreadLimitControlsOptions)
{
  P::setThreadLimit(std::nullopt);
  EXPECT_EQ(P::threadLimit(), std::nullopt);

  P::setThreadLimit(3);
  EXPECT_EQ(P::threadLimit(), std::optional<int>(3));

  int calls = 0;
  P::parallelFor(0, 4, [&](int) {
    ++calls;
  });
  EXPECT_EQ(calls, 4);

  P::setThreadLimit(std::nullopt);
}

TEST(ParallelRuntimeTest, RejectsInvalidLimits)
{
  EXPECT_THROW(P::setThreadLimit(0), std::invalid_argument);
  EXPECT_THROW(P::setThreadLimit(-1), std::invalid_argument);
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

  P::setThreadLimit(1);
  const auto limited = tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism);
  P::setThreadLimit(std::nullopt);
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

#ifdef PGO_PARALLELISM_HAS_EIGEN
TEST(ParallelRuntimeTest, GlobalLimitControlsEigenAndRestoresPreviousValue)
{
  const int initial = Eigen::nbThreads();
  if (initial <= 1)
    GTEST_SKIP() << "Eigen runtime exposes only one execution slot.";
  const int requested = 1;

  P::setThreadLimit(requested);
  const int limited = Eigen::nbThreads();
  P::setThreadLimit(std::nullopt);

  EXPECT_EQ(limited, requested);
  EXPECT_EQ(Eigen::nbThreads(), initial);
}
#endif

#ifdef PGO_PARALLELISM_HAS_OPENMP
TEST(ParallelRuntimeTest, GlobalLimitControlsOpenMPAndRestoresPreviousValue)
{
  const int initial = omp_get_max_threads();
  const int requested = initial == 1 ? 2 : 1;

  P::setThreadLimit(requested);
  const int limited = omp_get_max_threads();
  P::setThreadLimit(std::nullopt);

  EXPECT_EQ(limited, requested);
  EXPECT_EQ(omp_get_max_threads(), initial);
}
#endif

#ifdef PGO_PARALLELISM_HAS_MKL
TEST(ParallelRuntimeTest, GlobalLimitControlsMKLAndRestoresPreviousValue)
{
#ifdef PGO_PARALLELISM_MKL_TBB_THREADING
  const auto initial = P::runtimeInfo().mklEffectiveThreadLimit;
  if (!initial.has_value() || *initial <= 1)
    GTEST_SKIP() << "MKL TBB runtime exposes only one execution slot.";

  P::setThreadLimit(1);
  const auto limited = P::runtimeInfo().mklEffectiveThreadLimit;
  P::setThreadLimit(std::nullopt);
  const auto restored = P::runtimeInfo().mklEffectiveThreadLimit;

  EXPECT_EQ(limited, std::optional<int>(1));
  EXPECT_EQ(restored, initial);
#else
  const int initial = mkl_get_max_threads();
  if (initial <= 1)
    GTEST_SKIP() << "MKL runtime exposes only one execution slot.";
  const int requested = 1;

  P::setThreadLimit(requested);
  const int limited = mkl_get_max_threads();
  P::setThreadLimit(std::nullopt);

  EXPECT_EQ(limited, requested);
  EXPECT_EQ(mkl_get_max_threads(), initial);
#endif
}
#endif

TEST(ParallelRuntimeTest, ScopedNestedThreadLimitRestrictsNestedRuntimeAndRestoresPreviousValue)
{
#if !defined(PGO_PARALLELISM_HAS_OPENMP) && !defined(PGO_PARALLELISM_HAS_MKL)
  GTEST_SKIP() << "No nested native runtime is available in this build.";
#else
  P::setThreadLimit(4);

#ifdef PGO_PARALLELISM_HAS_OPENMP
  EXPECT_EQ(omp_get_max_threads(), 4);
#endif
#ifdef PGO_PARALLELISM_HAS_MKL
#ifdef PGO_PARALLELISM_MKL_TBB_THREADING
  EXPECT_EQ(P::runtimeInfo().mklEffectiveThreadLimit, std::optional<int>(4));
#else
  EXPECT_EQ(mkl_get_max_threads(), 4);
#endif
#endif

  {
    P::ScopedNestedThreadLimit nestedThreadLimit;
#ifdef PGO_PARALLELISM_HAS_OPENMP
    EXPECT_EQ(omp_get_max_threads(), 1);
#endif
#ifdef PGO_PARALLELISM_HAS_MKL
#ifdef PGO_PARALLELISM_MKL_TBB_THREADING
    EXPECT_EQ(P::runtimeInfo().mklEffectiveThreadLimit, std::optional<int>(4));
#else
    EXPECT_EQ(mkl_get_max_threads(), 1);
#endif
#endif
  }

#ifdef PGO_PARALLELISM_HAS_OPENMP
  EXPECT_EQ(omp_get_max_threads(), 4);
#endif
#ifdef PGO_PARALLELISM_HAS_MKL
#ifdef PGO_PARALLELISM_MKL_TBB_THREADING
  EXPECT_EQ(P::runtimeInfo().mklEffectiveThreadLimit, std::optional<int>(4));
#else
  EXPECT_EQ(mkl_get_max_threads(), 4);
#endif
#endif

  P::setThreadLimit(std::nullopt);
#endif
}

#ifdef PGO_PARALLELISM_HAS_OPENBLAS
TEST(ParallelRuntimeTest, GlobalLimitControlsOpenBLASAndRestoresPreviousValue)
{
  const int initial = openblas_get_num_threads();
  if (initial <= 1)
    GTEST_SKIP() << "OpenBLAS runtime exposes only one execution slot.";
  const int requested = 1;

  P::setThreadLimit(requested);
  const int limited = openblas_get_num_threads();
  P::setThreadLimit(std::nullopt);

  EXPECT_EQ(limited, requested);
  EXPECT_EQ(openblas_get_num_threads(), initial);
}
#endif

TEST(ParallelRuntimeTest, ScopedThreadLimitRestoresPreviousLimit)
{
  P::setThreadLimit(4);
  {
    P::ScopedThreadLimit limit(1);
    EXPECT_EQ(P::threadLimit(), std::optional<int>(1));
  }
  EXPECT_EQ(P::threadLimit(), std::optional<int>(4));

  {
    P::ScopedThreadLimit limit(std::nullopt);
    EXPECT_EQ(P::threadLimit(), std::nullopt);
  }
  EXPECT_EQ(P::threadLimit(), std::optional<int>(4));

  P::setThreadLimit(std::nullopt);
}
