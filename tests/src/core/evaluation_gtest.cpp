#include <gtest/gtest.h>

#include "evaluation.h"
#include "potentialEnergy.h"

#include <numeric>

namespace
{
namespace ES = pgo::EigenSupport;
using pgo::NonlinearOptimization::PotentialEnergy;
using pgo::NonlinearOptimization::EnergyStateKind;
using pgo::NonlinearOptimization::evaluateValue;
using pgo::NonlinearOptimization::evaluateGradient;
using pgo::NonlinearOptimization::evaluateHessian;
using pgo::NonlinearOptimization::evaluateMaxStep;
using pgo::NonlinearOptimization::dofsOf;

// ── Fixed-topology fake energy ──────────────────────────────────
// func = 0.5 * (a*x0^2 + b*x1^2), grad = [a*x0, b*x1], Hessian = diag(a, b)
// Does NOT override hessian() — relies on base default dispatch:
//   hessianAlloc + hessianInPlace.
class FixedTopoEnergy : public PotentialEnergy
{
public:
  FixedTopoEnergy(int n, double a, double b) : n_(n), a_(a), b_(b) {}

  double func(ES::ConstRefVecXd x) const override
  {
    return 0.5 * (a_ * x[0] * x[0] + b_ * x[1] * x[1]);
  }

  void gradient(ES::ConstRefVecXd x, ES::RefVecXd grad) const override
  {
    grad[0] = a_ * x[0];
    grad[1] = b_ * x[1];
  }

  void hessianInPlace(ES::ConstRefVecXd /*x*/, ES::SpMatD &hess) const override
  {
    hessianInPlaceCalls_++;
    hess.coeffRef(0, 0) = a_;
    hess.coeffRef(1, 1) = b_;
  }

  void hessianAlloc(ES::SpMatD &hess) const override
  {
    hessianAllocCalls_++;
    hess.resize(n_, n_);
    hess.setZero();
    hess.coeffRef(0, 0) = a_;
    hess.coeffRef(1, 1) = b_;
  }

  // hessian() NOT overridden — uses base default = hessianAlloc + hessianInPlace

  void getDOFs(std::vector<int> &dofs) const override
  {
    dofs.resize(n_);
    std::iota(dofs.begin(), dofs.end(), 0);
  }

  int getNumDOFs() const override { return n_; }

  int isHessianTopologyFixed() const override { return 1; }

  mutable int hessianAllocCalls_ = 0;
  mutable int hessianInPlaceCalls_ = 0;

private:
  int n_;
  double a_, b_;
};

// ── Non-fixed-topology fake energy ──────────────────────────────
// Same energy but isHessianTopologyFixed() == 0 AND overrides hessian()
// to fill H with a tridiagonal pattern that differs from the diag pattern
// in hessianInPlace. This proves evaluateHessian dispatches through
// the override, not the base default.
class NonFixedTopoEnergy : public PotentialEnergy
{
public:
  explicit NonFixedTopoEnergy(int n) : n_(n) {}

  double func(ES::ConstRefVecXd x) const override
  {
    return 0.5 * x.squaredNorm();
  }

  void gradient(ES::ConstRefVecXd x, ES::RefVecXd grad) const override
  {
    grad = x;
  }

  // hessian() override: tridiagonal [2, -1, ...], NOT pure diagonal.
  void hessian(ES::ConstRefVecXd /*x*/, ES::SpMatD &hess) const override
  {
    hessianOverrideCalls_++;
    hess.resize(n_, n_);
    hess.setZero();
    for (int i = 0; i < n_; i++) {
      hess.coeffRef(i, i) = 2.0;
      if (i > 0)
        hess.coeffRef(i, i - 1) = -1.0;
      if (i < n_ - 1)
        hess.coeffRef(i, i + 1) = -1.0;
    }
  }

  void hessianInPlace(ES::ConstRefVecXd /*x*/, ES::SpMatD &hess) const override
  {
    hessianInPlaceCalls_++;
    hess.setZero();
    for (int i = 0; i < n_; i++)
      hess.coeffRef(i, i) = 1.0;
  }

  void hessianAlloc(ES::SpMatD &hess) const override
  {
    hessianAllocCalls_++;
    hess.resize(n_, n_);
    hess.setZero();
    for (int i = 0; i < n_; i++)
      hess.coeffRef(i, i) = 1.0;
  }

  void getDOFs(std::vector<int> &dofs) const override
  {
    dofs.resize(n_);
    std::iota(dofs.begin(), dofs.end(), 0);
  }

  int getNumDOFs() const override { return n_; }

  int isHessianTopologyFixed() const override { return 0; }

  mutable int hessianOverrideCalls_ = 0;
  mutable int hessianAllocCalls_ = 0;
  mutable int hessianInPlaceCalls_ = 0;

private:
  int n_;
};
}  // namespace

// ── evaluateValue ───────────────────────────────────────────────

TEST(EvaluationGTest, EvaluateValueCallsFunc)
{
  FixedTopoEnergy energy(2, 3.0, 5.0);
  ES::VXd x(2);
  x << 1.0, 2.0;

  double e = evaluateValue(energy, x);

  double expected = 0.5 * (3.0 * 1.0 + 5.0 * 4.0);  // = 11.5
  EXPECT_DOUBLE_EQ(e, expected);
}

TEST(EvaluationGTest, EvaluateValueThrowsOnSizeMismatch)
{
  FixedTopoEnergy energy(2, 3.0, 5.0);
  ES::VXd x(3);
  x.setZero();

  EXPECT_THROW(evaluateValue(energy, x), std::invalid_argument);
}

// ── evaluateGradient ────────────────────────────────────────────

TEST(EvaluationGTest, EvaluateGradientReturnsByValue)
{
  FixedTopoEnergy energy(2, 3.0, 5.0);
  ES::VXd x(2);
  x << 2.0, 3.0;

  ES::VXd g = evaluateGradient(energy, x);

  EXPECT_DOUBLE_EQ(g[0], 3.0 * 2.0);
  EXPECT_DOUBLE_EQ(g[1], 5.0 * 3.0);
}

// ── evaluateHessian: fixed topology ─────────────────────────────

TEST(EvaluationGTest, EvaluateHessianFixedTopologyUsesBaseDefault)
{
  FixedTopoEnergy energy(2, 3.0, 5.0);
  ES::VXd x(2);
  x << 1.0, 2.0;

  // Before calling evaluateHessian, counters should be zero.
  EXPECT_EQ(energy.hessianAllocCalls_, 0);
  EXPECT_EQ(energy.hessianInPlaceCalls_, 0);

  ES::SpMatD H = evaluateHessian(energy, x);

  // Base default hessian() calls hessianAlloc once + hessianInPlace once.
  EXPECT_EQ(energy.hessianAllocCalls_, 1);
  EXPECT_EQ(energy.hessianInPlaceCalls_, 1);

  // Verify the returned matrix has correct values.
  EXPECT_EQ(H.rows(), 2);
  EXPECT_EQ(H.cols(), 2);
  EXPECT_DOUBLE_EQ(H.coeff(0, 0), 3.0);
  EXPECT_DOUBLE_EQ(H.coeff(1, 1), 5.0);
  EXPECT_DOUBLE_EQ(H.coeff(0, 1), 0.0);
  EXPECT_DOUBLE_EQ(H.coeff(1, 0), 0.0);
}

// ── evaluateHessian: non-fixed topology ─────────────────────────

TEST(EvaluationGTest, EvaluateHessianNonFixedTopologyUsesOverride)
{
  NonFixedTopoEnergy energy(4);

  ES::VXd x(4);
  x << 1.0, 2.0, 3.0, 4.0;

  EXPECT_EQ(energy.hessianOverrideCalls_, 0);
  EXPECT_EQ(energy.hessianAllocCalls_, 0);
  EXPECT_EQ(energy.hessianInPlaceCalls_, 0);

  ES::SpMatD H = evaluateHessian(energy, x);

  // The override hessian() was called, NOT the base default.
  EXPECT_EQ(energy.hessianOverrideCalls_, 1);
  // hessianAlloc / hessianInPlace should NOT be called by evaluateHessian
  // when the override bypasses the base default.
  EXPECT_EQ(energy.hessianAllocCalls_, 0);
  EXPECT_EQ(energy.hessianInPlaceCalls_, 0);

  // Verify tridiagonal pattern.
  EXPECT_EQ(H.rows(), 4);
  EXPECT_EQ(H.cols(), 4);
  EXPECT_DOUBLE_EQ(H.coeff(0, 0), 2.0);
  EXPECT_DOUBLE_EQ(H.coeff(1, 0), -1.0);
  EXPECT_DOUBLE_EQ(H.coeff(0, 1), -1.0);
  EXPECT_DOUBLE_EQ(H.coeff(2, 1), -1.0);
  EXPECT_DOUBLE_EQ(H.coeff(1, 2), -1.0);
  EXPECT_DOUBLE_EQ(H.coeff(3, 2), -1.0);
  EXPECT_DOUBLE_EQ(H.coeff(2, 3), -1.0);
}

// ── evaluateMaxStep ─────────────────────────────────────────────

TEST(EvaluationGTest, EvaluateMaxStepUnconstrainedByDefault)
{
  FixedTopoEnergy energy(2, 3.0, 5.0);
  ES::VXd x(2);
  x << 1.0, 2.0;
  ES::VXd dx(2);
  dx << 0.1, 0.2;

  auto result = evaluateMaxStep(energy, x, dx);

  EXPECT_DOUBLE_EQ(result.alpha, 1.0);
  EXPECT_FALSE(result.materialClamped);
  EXPECT_FALSE(result.contactClamped);
}

// ── dofsOf ──────────────────────────────────────────────────────

TEST(EvaluationGTest, DofsOfReturnsByValue)
{
  FixedTopoEnergy energy(3, 1.0, 1.0);

  std::vector<int> dofs = dofsOf(energy);

  ASSERT_EQ(dofs.size(), 3u);
  EXPECT_EQ(dofs[0], 0);
  EXPECT_EQ(dofs[1], 1);
  EXPECT_EQ(dofs[2], 2);
}

// ── EnergyStateKind ─────────────────────────────────────────────

TEST(EvaluationGTest, EnergyStateKindDefaultIsGeneric)
{
  FixedTopoEnergy energy(2, 3.0, 5.0);

  EXPECT_EQ(energy.stateKind(), EnergyStateKind::Generic);
}
