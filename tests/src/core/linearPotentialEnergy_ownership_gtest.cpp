#include <gtest/gtest.h>

#include "linearPotentialEnergy.h"
#include "energy/evaluation.h"

#include <numeric>

namespace
{
namespace ES = pgo::EigenSupport;
using pgo::PredefinedPotentialEnergies::LinearPotentialEnergy;
using pgo::NonlinearOptimization::evaluateValue;
using pgo::NonlinearOptimization::evaluateGradient;

// After construction, the energy owns its b vector.
// The original input can be destroyed without affecting results.
TEST(LinearPotentialEnergyOwnershipGTest, SurvivesInputDestruction)
{
  const int n = 5;
  ES::VXd x(n);
  for (int i = 0; i < n; i++)
    x[i] = double(i + 1);

  std::shared_ptr<LinearPotentialEnergy> energy;

  {
    // Construct b in a nested scope so it is destroyed after.
    ES::VXd b(n);
    for (int i = 0; i < n; i++)
      b[i] = double(2 * i - 3);

    const double expectedValue = x.dot(b);
    const ES::VXd expectedGrad = b;

    energy = std::make_shared<LinearPotentialEnergy>(std::move(b));

    // Verify results while b is (moved-from but) still in scope.
    EXPECT_DOUBLE_EQ(evaluateValue(*energy, x), expectedValue);

    ES::VXd g = evaluateGradient(*energy, x);
    EXPECT_EQ(g.size(), n);
    for (int i = 0; i < n; i++)
      EXPECT_DOUBLE_EQ(g[i], expectedGrad[i]);
  }
  // b is now out of scope. energy must still own its copy.

  // Recompute expected values (b = [2*i - 3]).
  double expectedValue = 0.0;
  ES::VXd expectedGrad(n);
  for (int i = 0; i < n; i++) {
    expectedGrad[i] = double(2 * i - 3);
    expectedValue += x[i] * expectedGrad[i];
  }

  EXPECT_DOUBLE_EQ(evaluateValue(*energy, x), expectedValue);

  ES::VXd g = evaluateGradient(*energy, x);
  EXPECT_EQ(g.size(), n);
  for (int i = 0; i < n; i++)
    EXPECT_DOUBLE_EQ(g[i], expectedGrad[i]);
}

TEST(LinearPotentialEnergyOwnershipGTest, HessianIsEmpty)
{
  ES::VXd b(3);
  b << 1.0, 2.0, 3.0;
  LinearPotentialEnergy energy(std::move(b));

  ES::VXd x(3);
  x << 1.0, -1.0, 0.0;
  ES::SpMatD H;
  energy.hessian(x, H);

  // A linear energy has no second-order term, but hessianAlloc sizes the matrix to
  // the DOF count so it stays composable with other energies' Hessians in the
  // solver; hessianInPlace adds nothing, leaving a 3x3 matrix with no nonzeros.
  EXPECT_EQ(H.rows(), 3);
  EXPECT_EQ(H.cols(), 3);
  EXPECT_EQ(H.nonZeros(), 0);
}

TEST(LinearPotentialEnergyOwnershipGTest, DofsAndSize)
{
  ES::VXd b(4);
  b.setZero();
  LinearPotentialEnergy energy(std::move(b));

  EXPECT_EQ(energy.getNumDOFs(), 4);

  std::vector<int> dofs;
  energy.getDOFs(dofs);
  ASSERT_EQ(dofs.size(), 4u);
  EXPECT_EQ(dofs[0], 0);
  EXPECT_EQ(dofs[3], 3);
}
}  // namespace
