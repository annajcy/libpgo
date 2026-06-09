#include "tetLinearShapeFunction.h"

#include "EigenSupport.h"

#include <cstring>

namespace pgo
{
namespace SolidDeformationModel
{

namespace ES = pgo::EigenSupport;

void TetLinearShapeFunction::N(double xi, double eta, double zeta, double N_out[]) const
{
  N_out[0] = 1.0 - xi - eta - zeta;
  N_out[1] = xi;
  N_out[2] = eta;
  N_out[3] = zeta;
}

void TetLinearShapeFunction::dN_dxi(double xi, double eta, double zeta, double dN_dxi[]) const
{
  (void)xi;
  (void)eta;
  (void)zeta;
  // Column-major 3x4: dN_dxi(deriv, node) = dN_dxi[deriv + 3 * node]
  //
  // Node 0: dN0/dxi = (-1, -1, -1)
  dN_dxi[0] = -1.0;   // d/dxi
  dN_dxi[1] = -1.0;   // d/deta
  dN_dxi[2] = -1.0;   // d/dzeta
  // Node 1: dN1/dxi = (1, 0, 0)
  dN_dxi[3] = 1.0;
  dN_dxi[4] = 0.0;
  dN_dxi[5] = 0.0;
  // Node 2: dN2/dxi = (0, 1, 0)
  dN_dxi[6] = 0.0;
  dN_dxi[7] = 1.0;
  dN_dxi[8] = 0.0;
  // Node 3: dN3/dxi = (0, 0, 1)
  dN_dxi[9] = 0.0;
  dN_dxi[10] = 0.0;
  dN_dxi[11] = 1.0;
}

void TetLinearShapeFunction::nodeCoords(int node, double xi[3]) const
{
  switch (node) {
  case 0: xi[0] = 0.0; xi[1] = 0.0; xi[2] = 0.0; break;
  case 1: xi[0] = 1.0; xi[1] = 0.0; xi[2] = 0.0; break;
  case 2: xi[0] = 0.0; xi[1] = 1.0; xi[2] = 0.0; break;
  case 3: xi[0] = 0.0; xi[1] = 0.0; xi[2] = 1.0; break;
  default: xi[0] = xi[1] = xi[2] = 0.0; break;
  }
}

void tetLinearComputeDs(const double x[12], double Ds[9])
{
  ES::M3d D;
  D.col(0) = ES::V3d(x[3], x[4], x[5]) - ES::V3d(x[0], x[1], x[2]);
  D.col(1) = ES::V3d(x[6], x[7], x[8]) - ES::V3d(x[0], x[1], x[2]);
  D.col(2) = ES::V3d(x[9], x[10], x[11]) - ES::V3d(x[0], x[1], x[2]);
  (Eigen::Map<ES::M3d>(Ds)) = D;
}

void tetLinearComputeDFDx(const double DmInv[9], double dFdx[9 * 12])
{
  ES::M9x12d dF = ES::M9x12d::Zero();
  ES::M3d D = Eigen::Map<const ES::M3d>(DmInv);

  double v0 = -(D(0, 0) + D(1, 0) + D(2, 0));
  double v1 = -(D(0, 1) + D(1, 1) + D(2, 1));
  double v2 = -(D(0, 2) + D(1, 2) + D(2, 2));

  dF(0, 0) = v0;  dF(3, 0) = v1;  dF(6, 0) = v2;
  dF(1, 1) = v0;  dF(4, 1) = v1;  dF(7, 1) = v2;
  dF(2, 2) = v0;  dF(5, 2) = v1;  dF(8, 2) = v2;

  dF(0, 3) = D(0, 0);  dF(3, 3) = D(0, 1);  dF(6, 3) = D(0, 2);
  dF(1, 4) = D(0, 0);  dF(4, 4) = D(0, 1);  dF(7, 4) = D(0, 2);
  dF(2, 5) = D(0, 0);  dF(5, 5) = D(0, 1);  dF(8, 5) = D(0, 2);

  dF(0, 6) = D(1, 0);  dF(3, 6) = D(1, 1);  dF(6, 6) = D(1, 2);
  dF(1, 7) = D(1, 0);  dF(4, 7) = D(1, 1);  dF(7, 7) = D(1, 2);
  dF(2, 8) = D(1, 0);  dF(5, 8) = D(1, 1);  dF(8, 8) = D(1, 2);

  dF(0, 9) = D(2, 0);  dF(3, 9) = D(2, 1);  dF(6, 9) = D(2, 2);
  dF(1, 10) = D(2, 0); dF(4, 10) = D(2, 1); dF(7, 10) = D(2, 2);
  dF(2, 11) = D(2, 0); dF(5, 11) = D(2, 1); dF(8, 11) = D(2, 2);

  std::memcpy(dFdx, dF.data(), sizeof(double) * 9 * 12);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
