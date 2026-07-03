#include "solver/service/optimizationProblem.h"
#include "solver/service/optimizerUtils.h"

#include <gtest/gtest.h>

namespace OPT = pgo::NonlinearOptimization::Optimization;
namespace ES = pgo::EigenSupport;

TEST(OptimizationProblem, MaterializeMissingBounds)
{
  OPT::Bounds bounds;
  auto materialized = OPT::materializeBounds(bounds, 3, -1e20, 1e20);
  EXPECT_EQ(materialized.lower.size(), 3);
  EXPECT_EQ(materialized.upper.size(), 3);
  EXPECT_DOUBLE_EQ(materialized.lower[0], -1e20);
  EXPECT_DOUBLE_EQ(materialized.upper[2], 1e20);
}

TEST(OptimizationProblem, RejectsMismatchedBoundsSize)
{
  OPT::Bounds bounds;
  bounds.lower = ES::VXd::Zero(2);
  EXPECT_THROW(OPT::materializeBounds(bounds, 3, -1e20, 1e20), std::invalid_argument);
}

TEST(OptimizationProblem, RejectsLowerGreaterThanUpper)
{
  OPT::Bounds bounds;
  bounds.lower = ES::VXd::Constant(2, 2.0);
  bounds.upper = ES::VXd::Constant(2, 1.0);
  EXPECT_THROW(OPT::materializeBounds(bounds, 2, -1e20, 1e20), std::invalid_argument);
}

TEST(OptimizationProblem, FixVariablesWritesEqualityBounds)
{
  OPT::OptimizationProblem problem;
  const std::vector<int> dofs = { 2, 0 };
  ES::VXd values(2);
  values << 4.0, -3.0;

  OPT::fixVariables(problem, dofs, values, 4);

  ASSERT_TRUE(problem.variableBounds.lower.has_value());
  ASSERT_TRUE(problem.variableBounds.upper.has_value());
  EXPECT_DOUBLE_EQ((*problem.variableBounds.lower)[0], -3.0);
  EXPECT_DOUBLE_EQ((*problem.variableBounds.upper)[2], 4.0);
}

TEST(OptimizationProblem, ExtractFixedDofsDetectsGeneralBounds)
{
  OPT::Bounds bounds;
  bounds.lower = ES::VXd::Constant(3, -1.0);
  bounds.upper = ES::VXd::Constant(3, 1.0);
  (*bounds.lower)[1] = 2.0;
  (*bounds.upper)[1] = 2.0;

  const OPT::FixedDofsFromBounds fixed = OPT::extractFixedDofsFromVariableBounds(bounds, 3);
  ASSERT_EQ(fixed.dofs.size(), 1);
  EXPECT_EQ(fixed.dofs[0], 1);
  EXPECT_DOUBLE_EQ(fixed.values[0], 2.0);
  EXPECT_TRUE(fixed.hasGeneralBounds);
}

TEST(OptimizationProblem, ExtractFixedDofsIgnoresInfiniteUnboundedEntries)
{
  OPT::OptimizationProblem problem;
  const std::vector<int> dofs = { 1 };
  ES::VXd values(1);
  values << 2.0;
  OPT::fixVariables(problem, dofs, values, 3);

  const OPT::FixedDofsFromBounds fixed = OPT::extractFixedDofsFromVariableBounds(problem.variableBounds, 3);
  ASSERT_EQ(fixed.dofs.size(), 1);
  EXPECT_EQ(fixed.dofs[0], 1);
  EXPECT_DOUBLE_EQ(fixed.values[0], 2.0);
  EXPECT_FALSE(fixed.hasGeneralBounds);
}
