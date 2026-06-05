#include "gtest/gtest.h"
#include "formulations/basis/hexTricubicHermiteBasis.h"

#include <array>

using namespace pgo::SolidDeformationModel;

namespace
{
// Corner parametric bits — must match HexTrilinearBasis / kVertexAlpha/Beta/Gamma.
const int kCornerXi[8] = { 0, 1, 1, 0, 0, 1, 1, 0 };
const int kCornerEta[8] = { 0, 0, 1, 1, 0, 0, 1, 1 };
const int kCornerZeta[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };

// Mode derivative bits (which axes are differentiated), in the documented order.
const int kModeXi[8] = { 0, 1, 0, 0, 1, 1, 0, 1 };
const int kModeEta[8] = { 0, 0, 1, 0, 1, 0, 1, 1 };
const int kModeZeta[8] = { 0, 0, 0, 1, 0, 1, 1, 1 };

// A separable test field f(xi,eta,zeta) = p(xi) q(eta) r(zeta), each factor a cubic, so all
// mixed derivatives are products of the per-axis factor and its derivative.
struct Cubic
{
  double c0, c1, c2, c3;
  double value(double t) const { return c0 + c1 * t + c2 * t * t + c3 * t * t * t; }
  double deriv(double t) const { return c1 + 2 * c2 * t + 3 * c3 * t * t; }
  // D^d: d==0 -> value, d==1 -> derivative.
  double D(int d, double t) const { return d == 0 ? value(t) : deriv(t); }
};

const Cubic kP{ 1.0, 2.0, -1.5, 0.7 };
const Cubic kQ{ -0.5, 1.3, 0.9, -1.1 };
const Cubic kR{ 0.4, -0.8, 1.7, 0.6 };

double fieldValue(double xi, double eta, double zeta)
{
  return kP.value(xi) * kQ.value(eta) * kR.value(zeta);
}
double fieldDxi(double xi, double eta, double zeta)
{
  return kP.deriv(xi) * kQ.value(eta) * kR.value(zeta);
}
double fieldDeta(double xi, double eta, double zeta)
{
  return kP.value(xi) * kQ.deriv(eta) * kR.value(zeta);
}
double fieldDzeta(double xi, double eta, double zeta)
{
  return kP.value(xi) * kQ.value(eta) * kR.deriv(zeta);
}

// DOF for (corner, mode) consistent with the separable field's Hermite coefficients.
std::array<double, 64> separableFieldDofs()
{
  std::array<double, 64> dof{};
  for (int c = 0; c < 8; c++) {
    for (int m = 0; m < 8; m++) {
      double v = kP.D(kModeXi[m], kCornerXi[c]) * kQ.D(kModeEta[m], kCornerEta[c]) * kR.D(kModeZeta[m], kCornerZeta[c]);
      dof[c * 8 + m] = v;
    }
  }
  return dof;
}

const double kTestPoints[5][3] = {
  { 0.5, 0.5, 0.5 },
  { 0.2113, 0.7887, 0.3 },
  { 0.1, 0.85, 0.42 },
  { 0.9, 0.2, 0.66 },
  { 0.37, 0.37, 0.91 },
};
}  // namespace

// The basis must reproduce a separable cubic field exactly from its Hermite DOFs.
TEST(HexTricubicHermiteBasisGTest, ReproducesSeparableCubicValue)
{
  HexTricubicHermiteBasis basis;
  auto dof = separableFieldDofs();

  for (const auto &pt : kTestPoints) {
    double N[64];
    basis.N(pt[0], pt[1], pt[2], N);
    double interp = 0.0;
    for (int node = 0; node < 64; node++)
      interp += dof[node] * N[node];
    EXPECT_NEAR(interp, fieldValue(pt[0], pt[1], pt[2]), 1e-12)
      << "at (" << pt[0] << "," << pt[1] << "," << pt[2] << ")";
  }
}

// And it must reproduce that field's first derivatives exactly.
TEST(HexTricubicHermiteBasisGTest, ReproducesSeparableCubicGradient)
{
  HexTricubicHermiteBasis basis;
  auto dof = separableFieldDofs();

  for (const auto &pt : kTestPoints) {
    double dN[192];
    basis.dN_dxi(pt[0], pt[1], pt[2], dN);
    double gx = 0.0, gy = 0.0, gz = 0.0;
    for (int node = 0; node < 64; node++) {
      gx += dof[node] * dN[0 + 3 * node];
      gy += dof[node] * dN[1 + 3 * node];
      gz += dof[node] * dN[2 + 3 * node];
    }
    EXPECT_NEAR(gx, fieldDxi(pt[0], pt[1], pt[2]), 1e-12);
    EXPECT_NEAR(gy, fieldDeta(pt[0], pt[1], pt[2]), 1e-12);
    EXPECT_NEAR(gz, fieldDzeta(pt[0], pt[1], pt[2]), 1e-12);
  }
}

// dN_dxi must be consistent with N under central finite differences.
TEST(HexTricubicHermiteBasisGTest, DerivativeMatchesFiniteDifference)
{
  HexTricubicHermiteBasis basis;
  const double h = 1e-6;
  for (const auto &pt : kTestPoints) {
    double dN[192];
    basis.dN_dxi(pt[0], pt[1], pt[2], dN);
    for (int axis = 0; axis < 3; axis++) {
      double pp[3] = { pt[0], pt[1], pt[2] };
      double pm[3] = { pt[0], pt[1], pt[2] };
      pp[axis] += h;
      pm[axis] -= h;
      double Np[64], Nm[64];
      basis.N(pp[0], pp[1], pp[2], Np);
      basis.N(pm[0], pm[1], pm[2], Nm);
      for (int node = 0; node < 64; node++) {
        double fd = (Np[node] - Nm[node]) / (2 * h);
        EXPECT_NEAR(dN[axis + 3 * node], fd, 1e-5) << "node " << node << " axis " << axis;
      }
    }
  }
}

// Hermite nodal property: at corner c, the VALUE mode of c is 1 (others 0 in value), and every
// derivative mode is 0 in value.
TEST(HexTricubicHermiteBasisGTest, ValueModeNodalInterpolation)
{
  HexTricubicHermiteBasis basis;
  for (int c = 0; c < 8; c++) {
    double N[64];
    basis.N(kCornerXi[c], kCornerEta[c], kCornerZeta[c], N);
    for (int node = 0; node < 64; node++) {
      int nodeCorner = node / 8;
      int nodeMode = node % 8;
      double expected = (nodeMode == 0 && nodeCorner == c) ? 1.0 : 0.0;
      EXPECT_NEAR(N[node], expected, 1e-13) << "corner " << c << " node " << node;
    }
  }
}
