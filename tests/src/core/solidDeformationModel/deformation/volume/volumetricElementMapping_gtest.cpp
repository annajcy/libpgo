#include "gtest/gtest.h"
#include "formulations/shapeFunction/tetLinearShapeFunction.h"
#include "formulations/shapeFunction/cubicLinearShapeFunction.h"
#include "formulations/shapeFunction/cubicTricubicHermiteShapeFunction.h"
#include "formulations/quadrature/tetLinearDefaultQuadrature.h"
#include "formulations/quadrature/gaussLegendreHexQuadrature.h"
#include "deformation/volume/volumetricElementMapping.h"

#include "EigenSupport.h"

#include <array>
#include <cmath>

namespace ES = pgo::EigenSupport;
using namespace pgo::SolidDeformationModel;

namespace
{
std::array<double, 192> makeUnitCubeHermiteRestDofs()
{
  constexpr int kCorners = 8;
  constexpr int kModesPerCorner = 8;
  static const double P[kCorners][3] = {
    { 0.0, 0.0, 0.0 },
    { 1.0, 0.0, 0.0 },
    { 1.0, 1.0, 0.0 },
    { 0.0, 1.0, 0.0 },
    { 0.0, 0.0, 1.0 },
    { 1.0, 0.0, 1.0 },
    { 1.0, 1.0, 1.0 },
    { 0.0, 1.0, 1.0 },
  };
  static const double dXi[3] = { 1.0, 0.0, 0.0 };
  static const double dEta[3] = { 0.0, 1.0, 0.0 };
  static const double dZeta[3] = { 0.0, 0.0, 1.0 };

  std::array<double, 192> rest{};
  for (int c = 0; c < kCorners; c++) {
    const double *mode[4] = { P[c], dXi, dEta, dZeta };
    for (int m = 0; m < 4; m++) {
      const int node = c * kModesPerCorner + m;
      for (int coord = 0; coord < 3; coord++)
        rest[node * 3 + coord] = mode[m][coord];
    }
  }
  return rest;
}
}  // namespace

// ============================================================
// Tet mapping tests
// ============================================================

TEST(VolumetricElementMappingGTest, TetElementMappingRestStateFrefIsIdentity)
{
  double rest[12] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
  TetLinearShapeFunction basis;
  TetLinearDefaultQuadrature quad;
  VolumetricElementMapping mapping(rest, basis, quad);

  // At rest (x = rest), Fref should be identity.
  const ES::M3d FMat = mapping.computeFref(rest, 0);
  EXPECT_TRUE(FMat.isApprox(ES::M3d::Identity(), 1e-12));
}

TEST(VolumetricElementMappingGTest, TetElementMappingUniformTranslationLeavesFrefUnchanged)
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

  TetLinearShapeFunction basis;
  TetLinearDefaultQuadrature quad;
  VolumetricElementMapping mapping(rest, basis, quad);

  const ES::M3d Frest = mapping.computeFref(rest, 0);
  const ES::M3d Ftrans = mapping.computeFref(x, 0);
  EXPECT_TRUE(Frest.isApprox(Ftrans, 1e-12));
}

TEST(VolumetricElementMappingGTest, TetElementMappingAffineDeformationGivesExactF)
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

  TetLinearShapeFunction basis;
  TetLinearDefaultQuadrature quad;
  VolumetricElementMapping mapping(rest, basis, quad);
  const ES::M3d FMat = mapping.computeFref(x, 0);

  EXPECT_TRUE(FMat.isApprox(A, 1e-12));
}

TEST(VolumetricElementMappingGTest, TetElementMappingWeightDetJEqualsVolume)
{
  double rest[12] = {
    0.0, 0.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };
  TetLinearShapeFunction basis;
  TetLinearDefaultQuadrature quad;
  VolumetricElementMapping mapping(rest, basis, quad);
  EXPECT_NEAR(mapping.weightDetJ(0), 1.0 / 6.0, 1e-12);
}

TEST(VolumetricElementMappingGTest, TetElementMappingComputedFrefdxMatchesFiniteDifference)
{
  double rest[12] = {
    0.0, 0.0, 0.0,
    2.0, 0.0, 0.0,
    0.0, 3.0, 0.0,
    0.0, 0.0, 4.0,
  };

  TetLinearShapeFunction basis;
  TetLinearDefaultQuadrature quad;
  VolumetricElementMapping mapping(rest, basis, quad);

  Eigen::Matrix<double, 9, Eigen::Dynamic> dFdx_ana(9, 12);
  mapping.computedFrefdx(0, dFdx_ana);

  const double eps = 1e-7;
  for (int dof = 0; dof < 12; dof++) {
    double xPlus[12], xMinus[12];
    for (int i = 0; i < 12; i++) {
      xPlus[i] = rest[i];
      xMinus[i] = rest[i];
    }
    xPlus[dof] += eps;
    xMinus[dof] -= eps;

    const ES::M3d Fplus = mapping.computeFref(xPlus, 0);
    const ES::M3d Fminus = mapping.computeFref(xMinus, 0);

    for (int r = 0; r < 9; r++) {
      double fd = (Fplus.data()[r] - Fminus.data()[r]) / (2.0 * eps);
      EXPECT_NEAR(dFdx_ana(r, dof), fd, 1e-4);
    }
  }
}

// ============================================================
// Hex mapping tests
// ============================================================

TEST(VolumetricElementMappingGTest, HexElementMappingRestStateFrefIsIdentity)
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
  CubicLinearShapeFunction basis;
  GaussLegendreHexQuadrature2 quad;
  VolumetricElementMapping mapping(rest, basis, quad);

  for (int q = 0; q < GaussLegendreHexQuadrature2::kNumPoints; q++) {
    const ES::M3d FMat = mapping.computeFref(rest, q);
    EXPECT_TRUE(FMat.isApprox(ES::M3d::Identity(), 1e-12));
  }
}

TEST(VolumetricElementMappingGTest, HexElementMappingAffineDeformationGivesExactF)
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

  CubicLinearShapeFunction basis;
  GaussLegendreHexQuadrature2 quad;
  VolumetricElementMapping mapping(rest, basis, quad);
  for (int q = 0; q < GaussLegendreHexQuadrature2::kNumPoints; q++) {
    const ES::M3d FMat = mapping.computeFref(x, q);
    EXPECT_TRUE(FMat.isApprox(A, 1e-12));
  }
}

TEST(VolumetricElementMappingGTest, HexElementMappingWeightDetJSumEqualsVolume)
{
  double rest[24] = {
    0.0, 0.0, 0.0,  1.0, 0.0, 0.0,  1.0, 1.0, 0.0,  0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,  1.0, 0.0, 1.0,  1.0, 1.0, 1.0,  0.0, 1.0, 1.0,
  };
  CubicLinearShapeFunction basis;
  GaussLegendreHexQuadrature2 quad;
  VolumetricElementMapping mapping(rest, basis, quad);

  double volSum = 0.0;
  for (int q = 0; q < GaussLegendreHexQuadrature2::kNumPoints; q++) {
    volSum += mapping.weightDetJ(q);
  }
  EXPECT_NEAR(volSum, 1.0, 1e-12);
}

TEST(VolumetricElementMappingGTest, HexElementMappingComputedFrefdxMatchesFiniteDifference)
{
  double rest[24] = {
    0.0, 0.0, 0.0,  2.0, 0.0, 0.0,  2.0, 3.0, 0.0,  0.0, 3.0, 0.0,
    0.0, 0.0, 4.0,  2.0, 0.0, 4.0,  2.0, 3.0, 4.0,  0.0, 3.0, 4.0,
  };

  CubicLinearShapeFunction basis;
  GaussLegendreHexQuadrature2 quad;
  VolumetricElementMapping mapping(rest, basis, quad);

  for (int q = 0; q < GaussLegendreHexQuadrature2::kNumPoints; q++) {
    Eigen::Matrix<double, 9, Eigen::Dynamic> dFdx_ana(9, 24);
    mapping.computedFrefdx(q, dFdx_ana);

    const double eps = 1e-7;
    for (int dof = 0; dof < 24; dof++) {
      double xPlus[24], xMinus[24];
      for (int i = 0; i < 24; i++) {
        xPlus[i] = rest[i];
        xMinus[i] = rest[i];
      }
      xPlus[dof] += eps;
      xMinus[dof] -= eps;

      const ES::M3d Fplus = mapping.computeFref(xPlus, q);
      const ES::M3d Fminus = mapping.computeFref(xMinus, q);

      for (int r = 0; r < 9; r++) {
        double fd = (Fplus.data()[r] - Fminus.data()[r]) / (2.0 * eps);
        EXPECT_NEAR(dFdx_ana(r, dof), fd, 1e-4);
      }
    }
  }
}

TEST(VolumetricElementMappingGTest, HexElementMappingNumNodesAndDofs)
{
  EXPECT_EQ(CubicLinearShapeFunction::kNumNodes, 8);
  EXPECT_EQ(CubicLinearShapeFunction::kLocalDofs, 24);
  EXPECT_EQ(GaussLegendreHexQuadrature2::kNumPoints, 8);
}

TEST(VolumetricElementMappingGTest, CubicTricubicHermiteElementMappingUsesAll192LocalDofs)
{
  const std::array<double, 192> rest = makeUnitCubeHermiteRestDofs();
  CubicTricubicHermiteShapeFunction basis;
  GaussLegendreHexQuadrature4 quad;
  VolumetricElementMapping mapping(rest, basis, quad);

  EXPECT_EQ(mapping.numNodes(), 64);
  EXPECT_EQ(mapping.localDofs(), 192);
  EXPECT_EQ(mapping.rest_dFdx(0).rows(), 9);
  EXPECT_EQ(mapping.rest_dFdx(0).cols(), 192);
}

TEST(VolumetricElementMappingGTest, CubicTricubicHermiteComputedFrefdxMatchesFiniteDifference)
{
  const std::array<double, 192> rest = makeUnitCubeHermiteRestDofs();
  CubicTricubicHermiteShapeFunction basis;
  GaussLegendreHexQuadrature4 quad;
  VolumetricElementMapping mapping(rest, basis, quad);

  const double eps = 1e-7;
  for (int q = 0; q < mapping.numQuadraturePoints(); q++) {
    Eigen::Matrix<double, 9, Eigen::Dynamic> dFdxAna = mapping.rest_dFdx(q);
    ASSERT_EQ(dFdxAna.cols(), 192);

    for (int dof = 0; dof < 192; dof++) {
      std::array<double, 192> xPlus = rest;
      std::array<double, 192> xMinus = rest;
      xPlus[dof] += eps;
      xMinus[dof] -= eps;

      const ES::M3d Fplus = mapping.computeFref(xPlus, q);
      const ES::M3d Fminus = mapping.computeFref(xMinus, q);

      for (int r = 0; r < 9; r++) {
        const double fd = (Fplus.data()[r] - Fminus.data()[r]) / (2.0 * eps);
        EXPECT_NEAR(dFdxAna(r, dof), fd, 1e-4)
          << "q=" << q << " dof=" << dof << " F entry=" << r;
      }
    }
  }
}
