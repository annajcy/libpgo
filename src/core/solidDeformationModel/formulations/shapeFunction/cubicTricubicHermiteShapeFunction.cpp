#include "cubicTricubicHermiteShapeFunction.h"

namespace pgo
{
namespace SolidDeformationModel
{

namespace
{
// Corner parametric bits — must match CubicLinearShapeFunction / kVertexAlpha/Beta/Gamma so the 8 mesh
// hex vertices map 1:1 to the 8 Hermite corners.
const int kCornerXi[8] = { 0, 1, 1, 0, 0, 1, 1, 0 };
const int kCornerEta[8] = { 0, 0, 1, 1, 0, 0, 1, 1 };
const int kCornerZeta[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };

// Mode derivative bits (which axes are differentiated), in the documented order:
// 0:VALUE 1:DXI 2:DETA 3:DZETA 4:DXI_DETA 5:DXI_DZETA 6:DETA_DZETA 7:DXI_DETA_DZETA.
const int kModeXi[8] = { 0, 1, 0, 0, 1, 1, 0, 1 };
const int kModeEta[8] = { 0, 0, 1, 0, 1, 0, 1, 1 };
const int kModeZeta[8] = { 0, 0, 0, 1, 0, 1, 1, 1 };

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

void CubicTricubicHermiteShapeFunction::N(double xi, double eta, double zeta, double N_out[]) const
{
  for (int c = 0; c < 8; c++) {
    for (int m = 0; m < 8; m++) {
      const int node = c * 8 + m;
      N_out[node] = herm(kCornerXi[c], kModeXi[m], xi) * herm(kCornerEta[c], kModeEta[m], eta) * herm(kCornerZeta[c], kModeZeta[m], zeta);
    }
  }
}

void CubicTricubicHermiteShapeFunction::dN_dxi(double xi, double eta, double zeta, double dN_out[]) const
{
  // Column-major 3 x 64: dN_out[deriv + 3 * node].
  for (int c = 0; c < 8; c++) {
    for (int m = 0; m < 8; m++) {
      const int node = c * 8 + m;
      const double fx = herm(kCornerXi[c], kModeXi[m], xi);
      const double fy = herm(kCornerEta[c], kModeEta[m], eta);
      const double fz = herm(kCornerZeta[c], kModeZeta[m], zeta);
      const double dfx = hermD(kCornerXi[c], kModeXi[m], xi);
      const double dfy = hermD(kCornerEta[c], kModeEta[m], eta);
      const double dfz = hermD(kCornerZeta[c], kModeZeta[m], zeta);

      dN_out[0 + 3 * node] = dfx * fy * fz;
      dN_out[1 + 3 * node] = fx * dfy * fz;
      dN_out[2 + 3 * node] = fx * fy * dfz;
    }
  }
}

void CubicTricubicHermiteShapeFunction::nodeCoords(int node, double xi[3]) const
{
  // All 8 modes of a corner share that corner's parametric coordinates.
  const int c = (node >= 0 && node < 64) ? node / 8 : 0;
  xi[0] = static_cast<double>(kCornerXi[c]);
  xi[1] = static_cast<double>(kCornerEta[c]);
  xi[2] = static_cast<double>(kCornerZeta[c]);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
