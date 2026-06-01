#include <gtest/gtest.h>

#include "quadraticPotentialEnergy.h"
#include "evaluation.h"

#include <numeric>

namespace
{
namespace ES = pgo::EigenSupport;
using pgo::PredefinedPotentialEnergies::QuadraticPotentialEnergy;
using pgo::PredefinedPotentialEnergies::makeLeastSquaresEnergy;
using pgo::NonlinearOptimization::evaluateValue;
using pgo::NonlinearOptimization::evaluateGradient;
using pgo::NonlinearOptimization::evaluateHessian;

ES::SpMatD makeDiag2(int n, double a, double b)
{
  ES::SpMatD M(n, n);
  M.coeffRef(0, 0) = a;
  M.coeffRef(1, 1) = b;
  return M;
}

// ── A ownership ─────────────────────────────────────────────────

TEST(QuadraticPotentialEnergyOwnershipGTest, AOwnershipSurvivesInputDestruction)
{
  const int n = 2;
  ES::VXd x(n);
  x << 2.0, 3.0;

  std::shared_ptr<QuadraticPotentialEnergy> energy;

  {
    ES::SpMatD A = makeDiag2(n, 4.0, 6.0);
    energy = std::make_shared<QuadraticPotentialEnergy>(std::move(A));
  }
  // A destroyed; energy owns its copy.

  // func = 0.5 * (4*4 + 6*9) = 0.5 * (16 + 54) = 35
  EXPECT_DOUBLE_EQ(evaluateValue(*energy, x), 35.0);

  ES::VXd g = evaluateGradient(*energy, x);
  EXPECT_DOUBLE_EQ(g[0], 4.0 * 2.0);
  EXPECT_DOUBLE_EQ(g[1], 6.0 * 3.0);

  ES::SpMatD H = evaluateHessian(*energy, x);
  EXPECT_DOUBLE_EQ(H.coeff(0, 0), 4.0);
  EXPECT_DOUBLE_EQ(H.coeff(1, 1), 6.0);
}

// ── b ownership ─────────────────────────────────────────────────

TEST(QuadraticPotentialEnergyOwnershipGTest, BOwnershipSurvivesInputDestruction)
{
  const int n = 2;
  ES::VXd x(n);
  x << 1.0, -1.0;

  double c_b = 0.0;
  std::shared_ptr<QuadraticPotentialEnergy> energy;

  {
    ES::SpMatD A = makeDiag2(n, 2.0, 2.0);
    ES::VXd b(n);
    b << 3.0, -4.0;
    c_b = b.dot(x);  // expected b^T x = 3 + 4 = 7
    energy = std::make_shared<QuadraticPotentialEnergy>(std::move(A), std::move(b));
  }

  // func = 0.5 * x^T A x + b^T x
  //      = 0.5 * (2*1 + 2*1) + 7 = 2 + 7 = 9
  EXPECT_DOUBLE_EQ(evaluateValue(*energy, x), 9.0);

  ES::VXd g = evaluateGradient(*energy, x);
  EXPECT_DOUBLE_EQ(g[0], 2.0 * 1.0 + 3.0);
  EXPECT_DOUBLE_EQ(g[1], 2.0 * (-1.0) - 4.0);
}

// ── W ownership (parentheses + diag weight) ─────────────────────

TEST(QuadraticPotentialEnergyOwnershipGTest, WOwnershipSurvivesInputDestruction)
{
  const int n = 3;
  ES::VXd x(n);
  x << 1.0, 2.0, 0.0;

  std::shared_ptr<QuadraticPotentialEnergy> energy;

  {
    ES::SpMatD A(n, n);
    A.coeffRef(0, 0) = 1.0;
    A.coeffRef(1, 1) = 2.0;
    A.coeffRef(2, 2) = 1.0;

    double W[3] = { 2.0, 3.0, 1.0 };

    energy = makeLeastSquaresEnergy(A, W);
  }

  // energy = 1/2 x^T A^T W A x
  // diag(A) = [1,2,1], W = [2,3,1] → diag(A^T W A) = [2, 12, 1]
  // func = 0.5 * (2*1 + 12*4 + 1*0) = 0.5 * (2+48) = 25
  EXPECT_DOUBLE_EQ(evaluateValue(*energy, x), 25.0);
}

// ── Parentheses + b ownership ───────────────────────────────────

TEST(QuadraticPotentialEnergyOwnershipGTest, ParenthesesBOwnership)
{
  const int n = 2;
  ES::VXd x(n);
  x << 0.5, -0.5;

  std::shared_ptr<QuadraticPotentialEnergy> energy;

  {
    ES::SpMatD A(n, n);
    A.coeffRef(0, 0) = 1.0;
    A.coeffRef(1, 1) = 1.0;
    ES::VXd b(n);
    b << 1.0, 2.0;
    energy = makeLeastSquaresEnergy(std::move(A), std::move(b));
  }

  // energy = 1/2 (I x + b)^2 = 1/2 * ||x + b||^2
  // x + b = [1.5, 1.5] → squared norm = 4.5 → energy = 2.25
  EXPECT_DOUBLE_EQ(evaluateValue(*energy, x), 2.25);

  // gradient = x + b = [1.5, 1.5]
  ES::VXd g = evaluateGradient(*energy, x);
  EXPECT_DOUBLE_EQ(g[0], 1.5);
  EXPECT_DOUBLE_EQ(g[1], 1.5);

  // Hessian = I
  ES::SpMatD H = evaluateHessian(*energy, x);
  EXPECT_DOUBLE_EQ(H.coeff(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(H.coeff(1, 1), 1.0);
}

// ── Empty b (no linear term) ────────────────────────────────────

TEST(QuadraticPotentialEnergyOwnershipGTest, NullBIsSafe)
{
  ES::SpMatD A = makeDiag2(2, 5.0, 7.0);
  QuadraticPotentialEnergy energy(std::move(A));

  ES::VXd x(2);
  x << 1.0, 1.0;

  // func = 0.5 * (5 + 7) = 6
  EXPECT_DOUBLE_EQ(evaluateValue(energy, x), 6.0);
}

// ── State kind default ──────────────────────────────────────────

TEST(QuadraticPotentialEnergyOwnershipGTest, StateKindIsGeneric)
{
  ES::SpMatD A = makeDiag2(2, 1.0, 1.0);
  QuadraticPotentialEnergy energy(std::move(A));

  EXPECT_EQ(energy.stateKind(), pgo::NonlinearOptimization::EnergyStateKind::Generic);
}
}  // namespace
