#include <gtest/gtest.h>

#include "ipc/ipcActiveSetCache.h"

namespace
{
namespace ES = pgo::EigenSupport;
namespace IPC = pgo::Contact::IPC;

IPC::SurfaceIPCActiveSet makeActiveSet(ES::ConstRefVecXd x)
{
  IPC::SurfaceIPCActiveSet activeSet;
  activeSet.positions = x;
  return activeSet;
}
}  // namespace

TEST(IPCActiveSetCacheGTest, ReusesExactStateAndRebuildsChangedState)
{
  IPC::IPCActiveSetCache cache;
  int builds = 0;
  auto build = [&](ES::ConstRefVecXd x) {
    ++builds;
    return makeActiveSet(x);
  };

  const ES::VXd x0 = ES::VXd::Zero(3);
  const ES::VXd x1 = ES::VXd::Ones(3);

  const IPC::SurfaceIPCActiveSet &first = cache.forEvaluation(x0, build);
  const IPC::SurfaceIPCActiveSet &second = cache.forEvaluation(x0, build);
  EXPECT_EQ(&first, &second);
  EXPECT_EQ(builds, 1);

  cache.forEvaluation(x1, build);
  EXPECT_EQ(builds, 2);
}

TEST(IPCActiveSetCacheGTest, LineSearchOverridesExactUntilEnded)
{
  IPC::IPCActiveSetCache cache;
  int builds = 0;
  auto build = [&](ES::ConstRefVecXd x) {
    ++builds;
    return makeActiveSet(x);
  };

  const ES::VXd x0 = ES::VXd::Zero(3);
  const ES::VXd x1 = ES::VXd::Ones(3);
  cache.prepareExact(x0, build);

  IPC::SurfaceIPCActiveSet superset = makeActiveSet(x0);
  cache.beginLineSearch(std::move(superset));

  const IPC::SurfaceIPCActiveSet &lineSearch = cache.forEvaluation(x1, build);
  EXPECT_EQ(lineSearch.positions, x1);
  EXPECT_EQ(builds, 1);

  cache.endLineSearch();
  cache.forEvaluation(x1, build);
  EXPECT_EQ(builds, 2);
}
