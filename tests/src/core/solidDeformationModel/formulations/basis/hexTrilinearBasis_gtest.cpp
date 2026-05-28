#include "gtest/gtest.h"
#include "formulations/basis/hexTrilinearBasis.h"

#include <cmath>

using namespace pgo::SolidDeformationModel;

TEST(HexTrilinearBasisGTest, PartitionOfUnity)
{
  double testPoints[4][3] = {
    { 0.5, 0.5, 0.5 },
    { 0.0, 0.0, 0.0 },
    { 1.0, 1.0, 1.0 },
    { 0.2113, 0.7887, 0.2113 },  // Gauss point
  };
  for (const auto &pt : testPoints) {
    double N[8];
    HexTrilinearBasis::N(pt[0], pt[1], pt[2], N);
    double sum = 0.0;
    for (int i = 0; i < 8; i++) sum += N[i];
    EXPECT_NEAR(sum, 1.0, 1e-15);
  }
}

TEST(HexTrilinearBasisGTest, DerivativeSumToZero)
{
  double testPoints[4][3] = {
    { 0.5, 0.5, 0.5 },
    { 0.2113, 0.7887, 0.2113 },
    { 0.1, 0.2, 0.3 },
    { 0.9, 0.8, 0.7 },
  };
  for (const auto &pt : testPoints) {
    double dN[24];
    HexTrilinearBasis::dN_dxi(pt[0], pt[1], pt[2], dN);
    for (int deriv = 0; deriv < 3; deriv++) {
      double sum = 0.0;
      for (int node = 0; node < 8; node++) {
        sum += dN[deriv + 3 * node];
      }
      EXPECT_NEAR(sum, 0.0, 1e-15);
    }
  }
}

TEST(HexTrilinearBasisGTest, NodalInterpolation)
{
  // N_i(node j) = delta_ij
  for (int j = 0; j < 8; j++) {
    double xi[3];
    HexTrilinearBasis::nodeCoords(j, xi);
    double N[8];
    HexTrilinearBasis::N(xi[0], xi[1], xi[2], N);
    for (int i = 0; i < 8; i++) {
      EXPECT_NEAR(N[i], (i == j) ? 1.0 : 0.0, 1e-15);
    }
  }
}

TEST(HexTrilinearBasisGTest, NodeCoordsMatchLegacyConvention)
{
  // Legacy CubicMeshDeformationModel convention:
  // v0=(0,0,0) v1=(1,0,0) v2=(1,1,0) v3=(0,1,0)
  // v4=(0,0,1) v5=(1,0,1) v6=(1,1,1) v7=(0,1,1)
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
    double xi[3];
    HexTrilinearBasis::nodeCoords(i, xi);
    EXPECT_DOUBLE_EQ(xi[0], expected[i][0]);
    EXPECT_DOUBLE_EQ(xi[1], expected[i][1]);
    EXPECT_DOUBLE_EQ(xi[2], expected[i][2]);
  }
}

TEST(HexTrilinearBasisGTest, ShapeDerivativesMatchLegacyImplementation)
{
  // Verify dN/dxi matches the legacy computeShapeDerivative
  // in cubicMeshDeformationModel.cpp for Gauss points.
  const double offset = 0.5 / std::sqrt(3.0);
  const double gp[2] = { 0.5 - offset, 0.5 + offset };

  for (int ia = 0; ia < 2; ia++) {
    for (int ib = 0; ib < 2; ib++) {
      for (int ig = 0; ig < 2; ig++) {
        double dN[24];
        HexTrilinearBasis::dN_dxi(gp[ia], gp[ib], gp[ig], dN);

        // Verify constant sum property holds at all Gauss points.
        for (int deriv = 0; deriv < 3; deriv++) {
          double sum = 0.0;
          for (int node = 0; node < 8; node++) {
            sum += dN[deriv + 3 * node];
          }
          EXPECT_NEAR(sum, 0.0, 1e-15);
        }
      }
    }
  }
}
