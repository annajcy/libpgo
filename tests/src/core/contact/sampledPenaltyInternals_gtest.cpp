#include <gtest/gtest.h>

#include "sampled_penalty/sampledPenaltyContactEvaluator.h"
#include "sampled_penalty/sampledPenaltyEvaluationBundle.h"

#include <type_traits>

namespace
{
namespace ES = pgo::EigenSupport;
namespace SP = pgo::Contact::SampledPenalty;

static_assert(!std::is_copy_constructible_v<SP::SampledPenaltyEvaluationBundle>);
static_assert(!std::is_copy_assignable_v<SP::SampledPenaltyEvaluationBundle>);
static_assert(!std::is_move_constructible_v<SP::SampledPenaltyEvaluationBundle>);
static_assert(!std::is_move_assignable_v<SP::SampledPenaltyEvaluationBundle>);
}  // namespace

TEST(SampledPenaltyInternalsGTest, EvaluationBundleStartsEmptyAndClears)
{
  SP::SampledPenaltyEvaluationBundle bundle;
  EXPECT_TRUE(bundle.empty());
  EXPECT_NO_THROW(bundle.clear());
  EXPECT_TRUE(bundle.empty());
}

TEST(SampledPenaltyInternalsGTest, EvaluatorHandlesEmptyBundle)
{
  SP::SampledPenaltyEvaluationBundle bundle;
  SP::SampledPenaltyContactEvaluator evaluator;

  const ES::VXd x = ES::VXd::Zero(6);
  ES::VXd g = ES::VXd::Ones(6);
  ES::SpMatD H;

  EXPECT_DOUBLE_EQ(evaluator.func(bundle, x), 0.0);
  evaluator.gradient(bundle, x, g);
  evaluator.hessian(bundle, x, H);

  EXPECT_EQ(g.norm(), 0.0);
  EXPECT_EQ(H.rows(), x.size());
  EXPECT_EQ(H.cols(), x.size());
  EXPECT_EQ(H.nonZeros(), 0);
}
