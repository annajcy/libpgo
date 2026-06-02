#include <gtest/gtest.h>

#include "constraints/constraintEvaluation.h"
#include "constraints/linearConstraintFunctions.h"

namespace
{
namespace ES = pgo::EigenSupport;
using pgo::NonlinearOptimization::LinearConstraintFunctions;
using pgo::NonlinearOptimization::evaluateConstraintHessian;
using pgo::NonlinearOptimization::evaluateConstraintJacobian;
using pgo::NonlinearOptimization::evaluateConstraintValues;

ES::SpMatD makeTestMatrix()
{
  ES::SpMatD C(2, 3);
  std::vector<ES::TripletD> entries;
  entries.emplace_back(0, 0, 2.0);
  entries.emplace_back(0, 2, -1.0);
  entries.emplace_back(1, 1, 3.0);
  C.setFromTriplets(entries.begin(), entries.end());
  return C;
}
}  // namespace

TEST(LinearConstraintFunctionsOwnershipGTest, OwnsSparseMatrixAndOffset)
{
  std::shared_ptr<LinearConstraintFunctions> constraints;
  {
    ES::SpMatD C = makeTestMatrix();
    ES::VXd offset(2);
    offset << 5.0, -4.0;
    constraints = std::make_shared<LinearConstraintFunctions>(C, offset);

    C.coeffRef(0, 0) = 99.0;
    offset[0] = 99.0;
  }

  ES::VXd x(3);
  x << 1.0, 2.0, 3.0;
  ES::VXd values = evaluateConstraintValues(*constraints, x);

  EXPECT_DOUBLE_EQ(values[0], 2.0 * 1.0 - 3.0 + 5.0);
  EXPECT_DOUBLE_EQ(values[1], 3.0 * 2.0 - 4.0);

  ES::SpMatD J = evaluateConstraintJacobian(*constraints, x);
  EXPECT_DOUBLE_EQ(J.coeff(0, 0), 2.0);
  EXPECT_DOUBLE_EQ(J.coeff(0, 2), -1.0);
  EXPECT_DOUBLE_EQ(J.coeff(1, 1), 3.0);
}

TEST(LinearConstraintFunctionsOwnershipGTest, SetOffsetCopiesInput)
{
  ES::SpMatD C = makeTestMatrix();
  ES::VXd offset = ES::VXd::Zero(2);
  LinearConstraintFunctions constraints(C, offset);

  ES::VXd newOffset(2);
  newOffset << -2.0, 7.0;
  constraints.setOffset(newOffset);
  newOffset[0] = 100.0;

  ES::VXd x(3);
  x << 0.0, 1.0, 0.0;
  ES::VXd values = evaluateConstraintValues(constraints, x);

  EXPECT_DOUBLE_EQ(values[0], -2.0);
  EXPECT_DOUBLE_EQ(values[1], 10.0);
}

TEST(LinearConstraintFunctionsOwnershipGTest, LinearHessianAndHessianVectorAreZero)
{
  ES::SpMatD C = makeTestMatrix();
  ES::VXd offset = ES::VXd::Zero(2);
  LinearConstraintFunctions constraints(C, offset);

  ES::VXd x(3);
  x << 1.0, 2.0, 3.0;
  ES::VXd lambda(2);
  lambda << 4.0, 5.0;

  ES::SpMatD H = evaluateConstraintHessian(constraints, x, lambda);
  EXPECT_EQ(H.rows(), 3);
  EXPECT_EQ(H.cols(), 3);
  EXPECT_EQ(H.nonZeros(), 0);

  ES::VXd vec(3);
  vec << 1.0, -1.0, 2.0;
  ES::VXd hessVec(3);
  hessVec.setConstant(42.0);
  constraints.hessianVector(x, lambda, vec, hessVec);
  EXPECT_TRUE(hessVec.isZero(0.0));
}
