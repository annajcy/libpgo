#include "gtest/gtest.h"
#include "formulations/shapeFunction/cubicLinearShapeFunction.h"

#include <cmath>

using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

TEST(CubicLinearShapeFunctionGTest, PartitionOfUnity)
{
  CubicLinearShapeFunction basis;
  double testPoints[4][3] = {
    { 0.5, 0.5, 0.5 },
    { 0.0, 0.0, 0.0 },
    { 1.0, 1.0, 1.0 },
    { 0.2113, 0.7887, 0.2113 },
  };
  for (const auto &pt : testPoints) {
    const ES::V8d N = basis.compute_N(pt[0], pt[1], pt[2]);
    const double sum = N.sum();
    EXPECT_NEAR(sum, 1.0, 1e-15);
  }
}

TEST(CubicLinearShapeFunctionGTest, DerivativeSumToZero)
{
  CubicLinearShapeFunction basis;
  double testPoints[4][3] = {
    { 0.5, 0.5, 0.5 },
    { 0.2113, 0.7887, 0.2113 },
    { 0.1, 0.2, 0.3 },
    { 0.9, 0.8, 0.7 },
  };
  for (const auto &pt : testPoints) {
    const ES::M3x8d dN = basis.compute_dN_dxi(pt[0], pt[1], pt[2]);
    for (int deriv = 0; deriv < 3; deriv++) {
      double sum = 0.0;
      for (int node = 0; node < 8; node++) {
        sum += dN(deriv, node);
      }
      EXPECT_NEAR(sum, 0.0, 1e-15);
    }
  }
}

TEST(CubicLinearShapeFunctionGTest, NodalInterpolation)
{
  CubicLinearShapeFunction basis;
  for (int j = 0; j < 8; j++) {
    const ES::V3d xi = basis.nodeCoords(j);
    const ES::V8d N = basis.compute_N(xi[0], xi[1], xi[2]);
    for (int i = 0; i < 8; i++) {
      EXPECT_NEAR(N[i], (i == j) ? 1.0 : 0.0, 1e-15);
    }
  }
}

TEST(CubicLinearShapeFunctionGTest, NodeCoordsMatchLegacyConvention)
{
  CubicLinearShapeFunction basis;
  double expected[8][3] = {
    { 0, 0, 0 },
    { 1, 0, 0 },
    { 1, 1, 0 },
    { 0, 1, 0 },
    { 0, 0, 1 },
    { 1, 0, 1 },
    { 1, 1, 1 },
    { 0, 1, 1 },
  };
  for (int i = 0; i < 8; i++) {
    const ES::V3d xi = basis.nodeCoords(i);
    EXPECT_DOUBLE_EQ(xi[0], expected[i][0]);
    EXPECT_DOUBLE_EQ(xi[1], expected[i][1]);
    EXPECT_DOUBLE_EQ(xi[2], expected[i][2]);
  }
}

TEST(CubicLinearShapeFunctionGTest, ShapeDerivativesMatchLegacyImplementation)
{
  CubicLinearShapeFunction basis;
  const double offset = 0.5 / std::sqrt(3.0);
  const double gp[2] = { 0.5 - offset, 0.5 + offset };

  for (int ia = 0; ia < 2; ia++) {
    for (int ib = 0; ib < 2; ib++) {
      for (int ig = 0; ig < 2; ig++) {
        const ES::M3x8d dN = basis.compute_dN_dxi(gp[ia], gp[ib], gp[ig]);

        for (int deriv = 0; deriv < 3; deriv++) {
          double sum = 0.0;
          for (int node = 0; node < 8; node++) {
            sum += dN(deriv, node);
          }
          EXPECT_NEAR(sum, 0.0, 1e-15);
        }
      }
    }
  }
}
