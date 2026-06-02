#include <gtest/gtest.h>

#include "constraints/constraintEvaluation.h"
#include "constraints/constraintSet.h"
#include "constraints/linearConstraintFunctions.h"
#include "constraints/potentialEnergyFromConstraintFunctions.h"

namespace
{
namespace ES = pgo::EigenSupport;
using pgo::NonlinearOptimization::ConstraintSet;
using pgo::NonlinearOptimization::ConstraintFunctions;
using pgo::NonlinearOptimization::LinearConstraintFunctions;
using pgo::NonlinearOptimization::PotentialEnergyBoundedConstraintFunctions;
using pgo::NonlinearOptimization::evaluateConstraintHessian;
using pgo::NonlinearOptimization::evaluateConstraintJacobian;
using pgo::NonlinearOptimization::evaluateConstraintValues;

std::shared_ptr<LinearConstraintFunctions> makeLinear(
  int rows,
  int cols,
  std::initializer_list<ES::TripletD> triplets,
  std::initializer_list<double> offsetValues)
{
  ES::SpMatD C(rows, cols);
  std::vector<ES::TripletD> entries(triplets);
  C.setFromTriplets(entries.begin(), entries.end());

  ES::VXd offset(rows);
  int i = 0;
  for (double value : offsetValues)
    offset[i++] = value;

  return std::make_shared<LinearConstraintFunctions>(C, offset);
}

class NoHessianVectorConstraint : public ConstraintFunctions
{
public:
  NoHessianVectorConstraint() : ConstraintFunctions(2)
  {
    jacobianTemplate.resize(1, 2);
    jacobianTemplate.insert(0, 0) = 1.0;
    jacobianTemplate.makeCompressed();
    lambdahTemplate.resize(2, 2);
  }

  void func(ES::ConstRefVecXd x, ES::RefVecXd g) const override { g[0] = x[0]; }
  void jacobian(ES::ConstRefVecXd, ES::SpMatD &jac) const override { jac = jacobianTemplate; }
  void hessianInPlace(ES::ConstRefVecXd, ES::ConstRefVecXd, ES::SpMatD &) const override {}
};
}  // namespace

TEST(ConstraintEvaluationGTest, HelpersMatchDirectOutParamApi)
{
  auto constraints = makeLinear(
    2, 3,
    { ES::TripletD(0, 0, 1.0), ES::TripletD(1, 2, 2.0) },
    { 3.0, -1.0 });

  ES::VXd x(3);
  x << 4.0, 5.0, 6.0;

  ES::VXd directValues(2);
  constraints->func(x, directValues);
  ES::SpMatD directJac;
  constraints->createJacobian(directJac);
  constraints->jacobian(x, directJac);
  ES::SpMatD directHess;
  constraints->hessianAlloc(directHess);
  constraints->hessianInPlace(x, ES::VXd::Ones(2), directHess);

  EXPECT_TRUE(evaluateConstraintValues(*constraints, x).isApprox(directValues));
  EXPECT_TRUE(evaluateConstraintJacobian(*constraints, x).isApprox(directJac));
  EXPECT_TRUE(evaluateConstraintHessian(*constraints, x, ES::VXd::Ones(2)).isApprox(directHess));
}

TEST(ConstraintEvaluationGTest, HelpersValidateInputShapes)
{
  auto constraints = makeLinear(
    2, 3,
    { ES::TripletD(0, 0, 1.0), ES::TripletD(1, 2, 2.0) },
    { 0.0, 0.0 });

  ES::VXd wrongX = ES::VXd::Zero(2);
  ES::VXd goodX = ES::VXd::Zero(3);
  ES::VXd wrongLambda = ES::VXd::Zero(1);

  EXPECT_THROW(evaluateConstraintValues(*constraints, wrongX), std::invalid_argument);
  EXPECT_THROW(evaluateConstraintJacobian(*constraints, wrongX), std::invalid_argument);
  EXPECT_THROW(evaluateConstraintHessian(*constraints, wrongX, ES::VXd::Zero(2)), std::invalid_argument);
  EXPECT_THROW(evaluateConstraintHessian(*constraints, goodX, wrongLambda), std::invalid_argument);
}

TEST(ConstraintEvaluationGTest, DefaultHessianVectorThrowsWhenUnsupported)
{
  NoHessianVectorConstraint constraints;
  ES::VXd x = ES::VXd::Zero(2);
  ES::VXd lambda = ES::VXd::Ones(1);
  ES::VXd vec = ES::VXd::Ones(2);
  ES::VXd hessVec = ES::VXd::Zero(2);

  EXPECT_FALSE(constraints.hasHessianVector());
  EXPECT_THROW(constraints.hessianVector(x, lambda, vec, hessVec), std::logic_error);
}

TEST(ConstraintSetGTest, EvaluatesImmediatelyAndPreservesRowOrder)
{
  auto first = makeLinear(
    2, 3,
    { ES::TripletD(0, 0, 1.0), ES::TripletD(1, 1, 2.0) },
    { 10.0, 20.0 });
  auto second = makeLinear(
    1, 3,
    { ES::TripletD(0, 2, -3.0) },
    { 30.0 });

  ConstraintSet set(3, {
    ConstraintSet::Term{ first },
    ConstraintSet::Term{ second },
  });

  ES::VXd x(3);
  x << 1.0, 2.0, 3.0;
  ES::VXd values = evaluateConstraintValues(set, x);

  ASSERT_EQ(values.size(), 3);
  EXPECT_DOUBLE_EQ(values[0], 11.0);
  EXPECT_DOUBLE_EQ(values[1], 24.0);
  EXPECT_DOUBLE_EQ(values[2], 21.0);

  ES::SpMatD J = evaluateConstraintJacobian(set, x);
  EXPECT_EQ(J.rows(), 3);
  EXPECT_EQ(J.cols(), 3);
  EXPECT_DOUBLE_EQ(J.coeff(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(J.coeff(1, 1), 2.0);
  EXPECT_DOUBLE_EQ(J.coeff(2, 2), -3.0);
}

TEST(ConstraintSetGTest, OwnsChildSharedPointers)
{
  ConstraintSet set(3, {
    ConstraintSet::Term{ makeLinear(1, 3, { ES::TripletD(0, 0, 4.0) }, { 1.0 }) },
  });

  ES::VXd x(3);
  x << 2.0, 0.0, 0.0;
  ES::VXd values = evaluateConstraintValues(set, x);

  ASSERT_EQ(values.size(), 1);
  EXPECT_DOUBLE_EQ(values[0], 9.0);
}

TEST(ConstraintSetGTest, RejectsInvalidTerms)
{
  EXPECT_THROW(ConstraintSet(3, {}), std::invalid_argument);
  EXPECT_THROW(ConstraintSet(3, { ConstraintSet::Term{ nullptr } }), std::invalid_argument);

  auto wrongDofs = makeLinear(1, 2, { ES::TripletD(0, 0, 1.0) }, { 0.0 });
  EXPECT_THROW(ConstraintSet(3, { ConstraintSet::Term{ wrongDofs } }), std::invalid_argument);
}

TEST(ConstraintSetGTest, AllLinearSetHasZeroHessian)
{
  ConstraintSet set(3, {
    ConstraintSet::Term{ makeLinear(1, 3, { ES::TripletD(0, 0, 4.0) }, { 1.0 }) },
    ConstraintSet::Term{ makeLinear(1, 3, { ES::TripletD(0, 2, 5.0) }, { 2.0 }) },
  });

  ES::VXd x = ES::VXd::Zero(3);
  ES::VXd lambda = ES::VXd::Ones(2);
  ES::SpMatD H = evaluateConstraintHessian(set, x, lambda);

  EXPECT_EQ(H.rows(), 3);
  EXPECT_EQ(H.cols(), 3);
  EXPECT_EQ(H.nonZeros(), 0);
  EXPECT_TRUE(set.isLinear());
}

TEST(ConstraintViolationPenaltyGTest, LinearBoundsValueGradientAndHessian)
{
  auto constraints = makeLinear(
    3, 3,
    { ES::TripletD(0, 0, 1.0), ES::TripletD(1, 1, 1.0), ES::TripletD(2, 2, 1.0) },
    { 0.0, 0.0, 0.0 });

  ES::VXd lower(3), upper(3);
  lower << 2.0, 0.0, -1.0;
  upper << 2.0, std::numeric_limits<double>::infinity(), 1.0;

  PotentialEnergyBoundedConstraintFunctions penalty(3, constraints, lower, upper);

  ES::VXd x(3);
  x << 1.5, -0.5, 0.25;

  EXPECT_DOUBLE_EQ(penalty.func(x), 0.25);

  ES::VXd grad(3);
  penalty.gradient(x, grad);
  EXPECT_TRUE(grad.isApprox((ES::VXd(3) << -0.5, -0.5, 0.0).finished()));

  ES::SpMatD hess;
  penalty.hessianAlloc(hess);
  penalty.hessianInPlace(x, hess);
  EXPECT_DOUBLE_EQ(hess.coeff(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(hess.coeff(1, 1), 1.0);
  EXPECT_DOUBLE_EQ(hess.coeff(2, 2), 0.0);
}
