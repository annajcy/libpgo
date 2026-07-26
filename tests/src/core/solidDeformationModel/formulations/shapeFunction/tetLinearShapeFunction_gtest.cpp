#include "gtest/gtest.h"
#include "formulations/shapeFunction/tetLinearShapeFunction.h"

using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

TEST(TetLinearShapeFunctionGTest, PartitionOfUnity)
{
  TetLinearShapeFunction basis;
  double testPoints[5][3] = {
    { 0.25, 0.25, 0.25 },
    { 0.0, 0.0, 0.0 },
    { 1.0, 0.0, 0.0 },
    { 0.0, 1.0, 0.0 },
    { 0.0, 0.0, 1.0 },
  };
  for (const auto &pt : testPoints) {
    const ES::V4d N = basis.compute_N(pt[0], pt[1], pt[2]);
    const double sum = N.sum();
    EXPECT_NEAR(sum, 1.0, 1e-15);
  }
}

TEST(TetLinearShapeFunctionGTest, DerivativeSumToZero)
{
  TetLinearShapeFunction basis;
  const ES::M3x4d dN = basis.compute_dN_dxi(0.25, 0.25, 0.25);

  for (int deriv = 0; deriv < 3; deriv++) {
    double sum = 0.0;
    for (int node = 0; node < 4; node++) {
      sum += dN(deriv, node);
    }
    EXPECT_NEAR(sum, 0.0, 1e-15);
  }
}

TEST(TetLinearShapeFunctionGTest, NodalInterpolation)
{
  TetLinearShapeFunction basis;
  for (int j = 0; j < 4; j++) {
    const ES::V3d xi = basis.nodeCoords(j);
    const ES::V4d N = basis.compute_N(xi[0], xi[1], xi[2]);
    for (int i = 0; i < 4; i++) {
      EXPECT_NEAR(N[i], (i == j) ? 1.0 : 0.0, 1e-15);
    }
  }
}

TEST(TetLinearShapeFunctionGTest, ShapeDerivativesAreConstant)
{
  TetLinearShapeFunction basis;
  const ES::M3x4d dN1 = basis.compute_dN_dxi(0.1, 0.2, 0.3);
  const ES::M3x4d dN2 = basis.compute_dN_dxi(0.4, 0.1, 0.1);
  EXPECT_TRUE(dN1.isApprox(dN2, 0.0));
}

TEST(TetLinearShapeFunctionGTest, NodeCoordsValid)
{
  TetLinearShapeFunction basis;
  ES::V3d xi = basis.nodeCoords(0);
  EXPECT_TRUE(xi.isApprox((ES::V3d() << 0.0, 0.0, 0.0).finished(), 0.0));

  xi = basis.nodeCoords(1);
  EXPECT_TRUE(xi.isApprox((ES::V3d() << 1.0, 0.0, 0.0).finished(), 0.0));

  xi = basis.nodeCoords(2);
  EXPECT_TRUE(xi.isApprox((ES::V3d() << 0.0, 1.0, 0.0).finished(), 0.0));

  xi = basis.nodeCoords(3);
  EXPECT_TRUE(xi.isApprox((ES::V3d() << 0.0, 0.0, 1.0).finished(), 0.0));
}
