#include "cubicTricubicHermiteShapeFunction.h"

#include <stdexcept>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

namespace
{
// Corner parametric bits — must match CubicLinearShapeFunction / kVertexAlpha/Beta/Gamma so the 8 mesh
// hex vertices map 1:1 to the 8 Hermite corners.
const ES::V8i kCornerXi = (ES::V8i() << 0, 1, 1, 0, 0, 1, 1, 0).finished();
const ES::V8i kCornerEta = (ES::V8i() << 0, 0, 1, 1, 0, 0, 1, 1).finished();
const ES::V8i kCornerZeta = (ES::V8i() << 0, 0, 0, 0, 1, 1, 1, 1).finished();

// Mode derivative bits (which axes are differentiated), in the documented order:
// 0:VALUE 1:DXI 2:DETA 3:DZETA 4:DXI_DETA 5:DXI_DZETA 6:DETA_DZETA 7:DXI_DETA_DZETA.
const ES::V8i kModeXi = (ES::V8i() << 0, 1, 0, 0, 1, 1, 0, 1).finished();
const ES::V8i kModeEta = (ES::V8i() << 0, 0, 1, 0, 1, 0, 1, 1).finished();
const ES::V8i kModeZeta = (ES::V8i() << 0, 0, 0, 1, 0, 1, 1, 1).finished();

// 1D cubic Hermite factor on [0,1]. endpoint e in {0,1}, deriv d in {0=value, 1=slope}.
inline double herm(int e, int d, double t)
{
  const double t2 = t * t;
  const double t3 = t2 * t;
  if (e == 0) {
    if (d == 0)
      return 1.0 - 3.0 * t2 + 2.0 * t3;  // h00
    return t - 2.0 * t2 + t3;            // h10
  }
  else {
    if (d == 0)
      return 3.0 * t2 - 2.0 * t3;        // h01
    return -t2 + t3;                     // h11
  }
}

// Derivative of herm w.r.t. t.
inline double hermD(int e, int d, double t)
{
  const double t2 = t * t;
  if (e == 0) {
    if (d == 0)
      return -6.0 * t + 6.0 * t2;        // h00'
    return 1.0 - 4.0 * t + 3.0 * t2;     // h10'
  }
  else {
    if (d == 0)
      return 6.0 * t - 6.0 * t2;         // h01'
    return -2.0 * t + 3.0 * t2;          // h11'
  }
}
}  // namespace

ES::V64d CubicTricubicHermiteShapeFunction::compute_N(double xi, double eta, double zeta) const
{
  ES::V64d N;
  for (int c = 0; c < 8; c++) {
    for (int m = 0; m < 8; m++) {
      const int node = c * 8 + m;
      N[node] = herm(kCornerXi[c], kModeXi[m], xi) * herm(kCornerEta[c], kModeEta[m], eta) * herm(kCornerZeta[c], kModeZeta[m], zeta);
    }
  }
  return N;
}

ES::M3x64d CubicTricubicHermiteShapeFunction::compute_dN_dxi(double xi, double eta, double zeta) const
{
  ES::M3x64d dN;
  // Column-major 3 x 64: dN(deriv, node).
  for (int c = 0; c < 8; c++) {
    for (int m = 0; m < 8; m++) {
      const int node = c * 8 + m;
      const double fx = herm(kCornerXi[c], kModeXi[m], xi);
      const double fy = herm(kCornerEta[c], kModeEta[m], eta);
      const double fz = herm(kCornerZeta[c], kModeZeta[m], zeta);
      const double dfx = hermD(kCornerXi[c], kModeXi[m], xi);
      const double dfy = hermD(kCornerEta[c], kModeEta[m], eta);
      const double dfz = hermD(kCornerZeta[c], kModeZeta[m], zeta);

      dN(0, node) = dfx * fy * fz;
      dN(1, node) = fx * dfy * fz;
      dN(2, node) = fx * fy * dfz;
    }
  }
  return dN;
}

void CubicTricubicHermiteShapeFunction::compute_N(double xi, double eta, double zeta,
  ES::RefVecXd N_out) const
{
  if (N_out.size() != kNumNodes)
    throw std::invalid_argument("CubicTricubicHermiteShapeFunction::compute_N output has the wrong size.");
  N_out = compute_N(xi, eta, zeta);
}

void CubicTricubicHermiteShapeFunction::compute_dN_dxi(double xi, double eta, double zeta,
  ES::RefMatXd dN_out) const
{
  if (dN_out.rows() != 3 || dN_out.cols() != kNumNodes)
    throw std::invalid_argument("CubicTricubicHermiteShapeFunction::compute_dN_dxi output has the wrong shape.");
  dN_out = compute_dN_dxi(xi, eta, zeta);
}

ES::V3d CubicTricubicHermiteShapeFunction::nodeCoords(int node) const
{
  // All 8 modes of a corner share that corner's parametric coordinates.
  const int c = (node >= 0 && node < 64) ? node / 8 : 0;
  ES::V3d xi;
  xi << static_cast<double>(kCornerXi[c]),
    static_cast<double>(kCornerEta[c]),
    static_cast<double>(kCornerZeta[c]);
  return xi;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
