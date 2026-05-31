#include "gtest/gtest.h"
#include "formulations/basis/tetP1Basis.h"
#include "formulations/basis/hexTrilinearBasis.h"
#include "formulations/quadrature/tetP1DefaultQuadrature.h"
#include "formulations/quadrature/gaussLegendreHexQuadrature.h"
#include "formulations/kernels/volumetricKernel.h"

#include "EigenSupport.h"

#include <cmath>

namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

// ============================================================
// Tet kernel tests
// ============================================================

TEST(VolumetricKernelGTest, TetKernelRestStateFrefIsIdentity)
{
  double rest[12] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
  TetP1Basis basis;
  TetP1DefaultQuadrature quad;
  VolumetricKernel kernel(rest, basis, quad);

  // At rest (x = rest), Fref should be identity.
  double F[9];
  kernel.computeFref(rest, 0, F);
  ES::M3d FMat = Eigen::Map<ES::M3d>(F);
  EXPECT_TRUE(FMat.isApprox(ES::M3d::Identity(), 1e-12));
}

TEST(VolumetricKernelGTest, TetKernelUniformTranslationLeavesFrefUnchanged)
{
  double rest[12] = {
    0.0, 0.0, 0.0,
    2.0, 0.0, 0.0,
    0.0, 3.0, 0.0,
    0.0, 0.0, 4.0,
  };
  double x[12];
  for (int i = 0; i < 12; i++) {
    x[i] = rest[i] + 5.0;
  }

  TetP1Basis basis;
  TetP1DefaultQuadrature quad;
  VolumetricKernel kernel(rest, basis, quad);

  double Frest[9], Ftrans[9];
  kernel.computeFref(rest, 0, Frest);
  kernel.computeFref(x, 0, Ftrans);

  for (int i = 0; i < 9; i++) {
    EXPECT_NEAR(Frest[i], Ftrans[i], 1e-12);
  }
}

TEST(VolumetricKernelGTest, TetKernelAffineDeformationGivesExactF)
{
  double rest[12] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };

  ES::M3d A;
  A << 2.0, 1.0, 0.0,
       0.0, 3.0, 0.0,
       0.0, 0.0, 4.0;
  ES::V3d b(1.0, 2.0, 3.0);

  double x[12];
  for (int vi = 0; vi < 4; vi++) {
    ES::V3d X(rest[vi * 3], rest[vi * 3 + 1], rest[vi * 3 + 2]);
    ES::V3d deformed = A * X + b;
    for (int c = 0; c < 3; c++) {
      x[vi * 3 + c] = deformed[c];
    }
  }

  TetP1Basis basis;
  TetP1DefaultQuadrature quad;
  VolumetricKernel kernel(rest, basis, quad);
  double F[9];
  kernel.computeFref(x, 0, F);
  ES::M3d FMat = Eigen::Map<ES::M3d>(F);

  EXPECT_TRUE(FMat.isApprox(A, 1e-12));
}

TEST(VolumetricKernelGTest, TetKernelWeightDetJEqualsVolume)
{
  double rest[12] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
  TetP1Basis basis;
  TetP1DefaultQuadrature quad;
  VolumetricKernel kernel(rest, basis, quad);
  EXPECT_NEAR(kernel.weightDetJ(0), 1.0 / 6.0, 1e-12);
}

TEST(VolumetricKernelGTest, TetKernelComputedFrefdxMatchesFiniteDifference)
{
  double rest[12] = {
    0.0, 0.0, 0.0,
    2.0, 0.0, 0.0,
    0.0, 3.0, 0.0,
    0.0, 0.0, 4.0,
  };

  TetP1Basis basis;
  TetP1DefaultQuadrature quad;
  VolumetricKernel kernel(rest, basis, quad);

  double dFdx_flat[9 * 12];
  kernel.computedFrefdx(0, dFdx_flat);
  Eigen::Map<Eigen::Matrix<double, 9, Eigen::Dynamic>> dFdx_ana(dFdx_flat, 9, 12);

  const double eps = 1e-7;
  for (int dof = 0; dof < 12; dof++) {
    double xPlus[12], xMinus[12];
    for (int i = 0; i < 12; i++) {
      xPlus[i] = rest[i];
      xMinus[i] = rest[i];
    }
    xPlus[dof] += eps;
    xMinus[dof] -= eps;

    double Fplus[9], Fminus[9];
    kernel.computeFref(xPlus, 0, Fplus);
    kernel.computeFref(xMinus, 0, Fminus);

    for (int r = 0; r < 9; r++) {
      double fd = (Fplus[r] - Fminus[r]) / (2.0 * eps);
      EXPECT_NEAR(dFdx_ana(r, dof), fd, 1e-4);
    }
  }
}

// ============================================================
// Hex kernel tests
// ============================================================

TEST(VolumetricKernelGTest, HexKernelRestStateFrefIsIdentity)
{
  double rest[24] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    1.0, 1.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
    1.0, 0.0, 1.0,
    1.0, 1.0, 1.0,
    0.0, 1.0, 1.0,
  };
  HexTrilinearBasis basis;
  GaussLegendreHexQuadrature2 quad;
  VolumetricKernel kernel(rest, basis, quad);

  for (int q = 0; q < GaussLegendreHexQuadrature2::kNumPoints; q++) {
    double F[9];
    kernel.computeFref(rest, q, F);
    ES::M3d FMat = Eigen::Map<ES::M3d>(F);
    EXPECT_TRUE(FMat.isApprox(ES::M3d::Identity(), 1e-12));
  }
}

TEST(VolumetricKernelGTest, HexKernelAffineDeformationGivesExactF)
{
  double rest[24] = {
    0.0, 0.0, 0.0,
    2.0, 0.0, 0.0,
    2.0, 3.0, 0.0,
    0.0, 3.0, 0.0,
    0.0, 0.0, 4.0,
    2.0, 0.0, 4.0,
    2.0, 3.0, 4.0,
    0.0, 3.0, 4.0,
  };

  ES::M3d A;
  A << 1.5, 0.2, 0.0,
       0.0, 2.0, 0.0,
       0.0, 0.0, 1.0;
  ES::V3d b(1.0, 2.0, 3.0);

  double x[24];
  for (int vi = 0; vi < 8; vi++) {
    ES::V3d X(rest[vi * 3], rest[vi * 3 + 1], rest[vi * 3 + 2]);
    ES::V3d deformed = A * X + b;
    for (int c = 0; c < 3; c++) {
      x[vi * 3 + c] = deformed[c];
    }
  }

  HexTrilinearBasis basis;
  GaussLegendreHexQuadrature2 quad;
  VolumetricKernel kernel(rest, basis, quad);
  for (int q = 0; q < GaussLegendreHexQuadrature2::kNumPoints; q++) {
    double F[9];
    kernel.computeFref(x, q, F);
    ES::M3d FMat = Eigen::Map<ES::M3d>(F);
    EXPECT_TRUE(FMat.isApprox(A, 1e-12));
  }
}

TEST(VolumetricKernelGTest, HexKernelWeightDetJSumEqualsVolume)
{
  double rest[24] = {
    0.0, 0.0, 0.0,  1.0, 0.0, 0.0,  1.0, 1.0, 0.0,  0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,  1.0, 0.0, 1.0,  1.0, 1.0, 1.0,  0.0, 1.0, 1.0,
  };
  HexTrilinearBasis basis;
  GaussLegendreHexQuadrature2 quad;
  VolumetricKernel kernel(rest, basis, quad);

  double volSum = 0.0;
  for (int q = 0; q < GaussLegendreHexQuadrature2::kNumPoints; q++) {
    volSum += kernel.weightDetJ(q);
  }
  EXPECT_NEAR(volSum, 1.0, 1e-12);
}

TEST(VolumetricKernelGTest, HexKernelComputedFrefdxMatchesFiniteDifference)
{
  double rest[24] = {
    0.0, 0.0, 0.0,  2.0, 0.0, 0.0,  2.0, 3.0, 0.0,  0.0, 3.0, 0.0,
    0.0, 0.0, 4.0,  2.0, 0.0, 4.0,  2.0, 3.0, 4.0,  0.0, 3.0, 4.0,
  };

  HexTrilinearBasis basis;
  GaussLegendreHexQuadrature2 quad;
  VolumetricKernel kernel(rest, basis, quad);

  for (int q = 0; q < GaussLegendreHexQuadrature2::kNumPoints; q++) {
    double dFdx_flat[9 * 24];
    kernel.computedFrefdx(q, dFdx_flat);
    Eigen::Map<Eigen::Matrix<double, 9, Eigen::Dynamic>> dFdx_ana(dFdx_flat, 9, 24);

    const double eps = 1e-7;
    for (int dof = 0; dof < 24; dof++) {
      double xPlus[24], xMinus[24];
      for (int i = 0; i < 24; i++) {
        xPlus[i] = rest[i];
        xMinus[i] = rest[i];
      }
      xPlus[dof] += eps;
      xMinus[dof] -= eps;

      double Fplus[9], Fminus[9];
      kernel.computeFref(xPlus, q, Fplus);
      kernel.computeFref(xMinus, q, Fminus);

      for (int r = 0; r < 9; r++) {
        double fd = (Fplus[r] - Fminus[r]) / (2.0 * eps);
        EXPECT_NEAR(dFdx_ana(r, dof), fd, 1e-4);
      }
    }
  }
}

TEST(VolumetricKernelGTest, HexKernelNumNodesAndDofs)
{
  EXPECT_EQ(HexTrilinearBasis::kNumNodes, 8);
  EXPECT_EQ(HexTrilinearBasis::kLocalDofs, 24);
  EXPECT_EQ(GaussLegendreHexQuadrature2::kNumPoints, 8);
}
