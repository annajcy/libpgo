#include <gtest/gtest.h>

#include "sampled_penalty/sampledPenaltyActiveSetCache.h"

namespace
{
namespace ES = pgo::EigenSupport;
namespace SP = pgo::Contact::SampledPenalty;
}  // namespace

TEST(SampledPenaltyInternalsGTest, ActiveSetCacheReusesExactState)
{
  SP::SampledPenaltyActiveSetCache cache;
  int builds = 0;
  auto build = [&](ES::ConstRefVecXd) {
    ++builds;
    return std::make_unique<SP::SampledPenaltyActiveSet>();
  };

  const ES::VXd x0 = ES::VXd::Zero(3);
  const ES::VXd x1 = ES::VXd::Ones(3);

  const SP::SampledPenaltyActiveSet &first = cache.forEvaluation(x0, build);
  const SP::SampledPenaltyActiveSet &second = cache.forEvaluation(x0, build);
  EXPECT_EQ(&first, &second);
  EXPECT_EQ(builds, 1);

  cache.forEvaluation(x1, build);
  EXPECT_EQ(builds, 2);
}

TEST(SampledPenaltyInternalsGTest, LineSearchOverridesExactState)
{
  SP::SampledPenaltyActiveSetCache cache;
  int builds = 0;
  auto build = [&](ES::ConstRefVecXd) {
    ++builds;
    return std::make_unique<SP::SampledPenaltyActiveSet>();
  };

  const ES::VXd x0 = ES::VXd::Zero(3);
  const ES::VXd x1 = ES::VXd::Ones(3);

  cache.forEvaluation(x0, build);
  cache.beginLineSearch(x0, build);
  cache.forEvaluation(x1, build);
  EXPECT_EQ(builds, 2);

  cache.endLineSearch();
  cache.forEvaluation(x1, build);
  EXPECT_EQ(builds, 3);
}
