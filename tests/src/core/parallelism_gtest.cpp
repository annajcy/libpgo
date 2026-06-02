#include <gtest/gtest.h>

#include "parallelism/parallelFor.h"

#include <atomic>
#include <optional>
#include <stdexcept>
#include <vector>

namespace P = pgo::parallel;

TEST(ParallelForTest, SerialLoopVisitsEveryIndex)
{
  std::vector<int> values(16, -1);

  P::parallelFor(0, static_cast<int>(values.size()), { .numThreads = 1 }, [&](int i) {
    values[i] = i * i;
  });

  for (int i = 0; i < static_cast<int>(values.size()); ++i)
    EXPECT_EQ(values[i], i * i);
}

TEST(ParallelForTest, ParallelLoopVisitsEveryIndex)
{
  std::vector<int> values(1024, 0);

  P::parallelFor(0, static_cast<int>(values.size()), { .numThreads = 2 }, [&](int i) {
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

TEST(ParallelFor3DTest, VisitsEveryCoordinateOnce)
{
  constexpr int nx = 3;
  constexpr int ny = 4;
  constexpr int nz = 2;
  std::vector<int> counts(nx * ny * nz, 0);

  P::parallelFor3D(nx, ny, nz, { .numThreads = 1 }, [&](int x, int y, int z) {
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

TEST(ParallelRuntimeTest, DefaultNumThreadsControlsOptions)
{
  P::setDefaultNumThreads(std::nullopt);
  EXPECT_EQ(P::defaultNumThreads(), std::nullopt);

  P::setDefaultNumThreads(3);
  EXPECT_EQ(P::defaultNumThreads(), std::optional<int>(3));

  int calls = 0;
  P::parallelFor(0, 4, [&](int) {
    ++calls;
  });
  EXPECT_EQ(calls, 4);

  P::setDefaultNumThreads(std::nullopt);
}

TEST(ParallelRuntimeTest, ScopedThreadLimitRestoresPreviousDefault)
{
  P::setDefaultNumThreads(4);
  {
    P::ScopedThreadLimit limit(1);
    EXPECT_EQ(P::defaultNumThreads(), std::optional<int>(1));
  }
  EXPECT_EQ(P::defaultNumThreads(), std::optional<int>(4));

  {
    P::ScopedThreadLimit limit(std::nullopt);
    EXPECT_EQ(P::defaultNumThreads(), std::nullopt);
  }
  EXPECT_EQ(P::defaultNumThreads(), std::optional<int>(4));

  P::setDefaultNumThreads(std::nullopt);
}
