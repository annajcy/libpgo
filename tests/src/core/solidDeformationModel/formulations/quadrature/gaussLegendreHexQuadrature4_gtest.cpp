#include "gtest/gtest.h"
#include "formulations/quadrature/gaussLegendreHexQuadrature.h"

#include <cmath>

using namespace pgo::SolidDeformationModel;

// 4-point-per-axis Gauss-Legendre on [0,1]^3 has 64 points and weights summing to the unit volume.
TEST(GaussLegendreHexQuadrature4GTest, WeightsSumToUnitVolume)
{
  GaussLegendreHexQuadrature4 q;
  EXPECT_EQ(q.numPoints(), 64);
  double sum = 0.0;
  for (int i = 0; i < q.numPoints(); i++)
    sum += q.weight(i);
  EXPECT_NEAR(sum, 1.0, 1e-14);
}

// 4-point Gauss is exact for polynomials up to degree 2*4-1 = 7 per axis. Verify against the
// analytic integral of x^a y^b z^c over the unit cube: 1/((a+1)(b+1)(c+1)).
TEST(GaussLegendreHexQuadrature4GTest, IntegratesPolynomialsExactlyToDegree7)
{
  GaussLegendreHexQuadrature4 q;
  const int degrees[] = { 0, 1, 2, 3, 5, 7 };
  for (int a : degrees) {
    for (int b : degrees) {
      for (int c : degrees) {
        double approx = 0.0;
        for (int i = 0; i < q.numPoints(); i++) {
          double xi[3];
          q.point(i, xi);
          approx += q.weight(i) * std::pow(xi[0], a) * std::pow(xi[1], b) * std::pow(xi[2], c);
        }
        double exact = 1.0 / ((a + 1) * (b + 1) * (c + 1));
        EXPECT_NEAR(approx, exact, 1e-13) << "monomial x^" << a << " y^" << b << " z^" << c;
      }
    }
  }
}

// All points lie strictly inside the unit cube.
TEST(GaussLegendreHexQuadrature4GTest, PointsInsideUnitCube)
{
  GaussLegendreHexQuadrature4 q;
  for (int i = 0; i < q.numPoints(); i++) {
    double xi[3];
    q.point(i, xi);
    for (int d = 0; d < 3; d++) {
      EXPECT_GT(xi[d], 0.0);
      EXPECT_LT(xi[d], 1.0);
    }
  }
}
