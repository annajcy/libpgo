#include "gtest/gtest.h"
#include "formulations/basis/tetP1Basis.h"

using namespace pgo::SolidDeformationModel;

TEST(TetP1BasisGTest, PartitionOfUnity)
{
  // N_i(xi) should sum to 1 everywhere in the reference tet.
  double testPoints[5][3] = {
    { 0.25, 0.25, 0.25 },
    { 0.0, 0.0, 0.0 },
    { 1.0, 0.0, 0.0 },
    { 0.0, 1.0, 0.0 },
    { 0.0, 0.0, 1.0 },
  };
  for (const auto &pt : testPoints) {
    double N[4];
    TetP1Basis::N(pt[0], pt[1], pt[2], N);
    double sum = N[0] + N[1] + N[2] + N[3];
    EXPECT_NEAR(sum, 1.0, 1e-15);
  }
}

TEST(TetP1BasisGTest, DerivativeSumToZero)
{
  // sum_i dN_i/dxi == 0 for each derivative direction.
  double dN[12];
  TetP1Basis::dN_dxi(0.25, 0.25, 0.25, dN);

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
  // N_i(node j) = delta_ij
  for (int j = 0; j < 4; j++) {
    double xi[3];
    TetP1Basis::nodeCoords(j, xi);
    double N[4];
    TetP1Basis::N(xi[0], xi[1], xi[2], N);
    for (int i = 0; i < 4; i++) {
      EXPECT_NEAR(N[i], (i == j) ? 1.0 : 0.0, 1e-15);
    }
  }
}

TEST(TetP1BasisGTest, ShapeDerivativesAreConstant)
{
  // For linear tet, dN/dxi is independent of evaluation point.
  double dN1[12], dN2[12];
  TetP1Basis::dN_dxi(0.1, 0.2, 0.3, dN1);
  TetP1Basis::dN_dxi(0.4, 0.1, 0.1, dN2);
  for (int i = 0; i < 12; i++) {
    EXPECT_DOUBLE_EQ(dN1[i], dN2[i]);
  }
}

TEST(TetP1BasisGTest, NodeCoordsValid)
{
  // Node 0 is at origin, nodes 1-3 are on axes.
  double xi[3];
  TetP1Basis::nodeCoords(0, xi);
  EXPECT_DOUBLE_EQ(xi[0], 0.0);
  EXPECT_DOUBLE_EQ(xi[1], 0.0);
  EXPECT_DOUBLE_EQ(xi[2], 0.0);

  TetP1Basis::nodeCoords(1, xi);
  EXPECT_DOUBLE_EQ(xi[0], 1.0);
  EXPECT_DOUBLE_EQ(xi[1], 0.0);
  EXPECT_DOUBLE_EQ(xi[2], 0.0);

  TetP1Basis::nodeCoords(2, xi);
  EXPECT_DOUBLE_EQ(xi[0], 0.0);
  EXPECT_DOUBLE_EQ(xi[1], 1.0);
  EXPECT_DOUBLE_EQ(xi[2], 0.0);

  TetP1Basis::nodeCoords(3, xi);
  EXPECT_DOUBLE_EQ(xi[0], 0.0);
  EXPECT_DOUBLE_EQ(xi[1], 0.0);
  EXPECT_DOUBLE_EQ(xi[2], 1.0);
}
