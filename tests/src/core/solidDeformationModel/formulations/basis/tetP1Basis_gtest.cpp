#include "gtest/gtest.h"
#include "formulations/basis/tetP1Basis.h"

using namespace pgo::SolidDeformationModel;

TEST(TetP1BasisGTest, PartitionOfUnity)
{
  TetP1Basis basis;
  double testPoints[5][3] = {
    { 0.25, 0.25, 0.25 },
    { 0.0, 0.0, 0.0 },
    { 1.0, 0.0, 0.0 },
    { 0.0, 1.0, 0.0 },
    { 0.0, 0.0, 1.0 },
  };
  for (const auto &pt : testPoints) {
    double N[4];
    basis.N(pt[0], pt[1], pt[2], N);
    double sum = N[0] + N[1] + N[2] + N[3];
    EXPECT_NEAR(sum, 1.0, 1e-15);
  }
}

TEST(TetP1BasisGTest, DerivativeSumToZero)
{
  TetP1Basis basis;
  double dN[12];
  basis.dN_dxi(0.25, 0.25, 0.25, dN);

  for (int deriv = 0; deriv < 3; deriv++) {
    double sum = 0.0;
    for (int node = 0; node < 4; node++) {
      sum += dN[deriv + 3 * node];
    }
    EXPECT_NEAR(sum, 0.0, 1e-15);
  }
}

TEST(TetP1BasisGTest, NodalInterpolation)
{
  TetP1Basis basis;
  for (int j = 0; j < 4; j++) {
    double xi[3];
    basis.nodeCoords(j, xi);
    double N[4];
    basis.N(xi[0], xi[1], xi[2], N);
    for (int i = 0; i < 4; i++) {
      EXPECT_NEAR(N[i], (i == j) ? 1.0 : 0.0, 1e-15);
    }
  }
}

TEST(TetP1BasisGTest, ShapeDerivativesAreConstant)
{
  TetP1Basis basis;
  double dN1[12], dN2[12];
  basis.dN_dxi(0.1, 0.2, 0.3, dN1);
  basis.dN_dxi(0.4, 0.1, 0.1, dN2);
  for (int i = 0; i < 12; i++) {
    EXPECT_DOUBLE_EQ(dN1[i], dN2[i]);
  }
}

TEST(TetP1BasisGTest, NodeCoordsValid)
{
  TetP1Basis basis;
  double xi[3];
  basis.nodeCoords(0, xi);
  EXPECT_DOUBLE_EQ(xi[0], 0.0);
  EXPECT_DOUBLE_EQ(xi[1], 0.0);
  EXPECT_DOUBLE_EQ(xi[2], 0.0);

  basis.nodeCoords(1, xi);
  EXPECT_DOUBLE_EQ(xi[0], 1.0);
  EXPECT_DOUBLE_EQ(xi[1], 0.0);
  EXPECT_DOUBLE_EQ(xi[2], 0.0);

  basis.nodeCoords(2, xi);
  EXPECT_DOUBLE_EQ(xi[0], 0.0);
  EXPECT_DOUBLE_EQ(xi[1], 1.0);
  EXPECT_DOUBLE_EQ(xi[2], 0.0);

  basis.nodeCoords(3, xi);
  EXPECT_DOUBLE_EQ(xi[0], 0.0);
  EXPECT_DOUBLE_EQ(xi[1], 0.0);
  EXPECT_DOUBLE_EQ(xi[2], 1.0);
}
