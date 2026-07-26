#include "tetLinearShapeFunction.h"

#include "EigenSupport.h"

#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

namespace ES = pgo::EigenSupport;

ES::V4d TetLinearShapeFunction::compute_N(double xi, double eta, double zeta) const
{
  ES::V4d N;
  N << 1.0 - xi - eta - zeta, xi, eta, zeta;
  return N;
}

ES::M3x4d TetLinearShapeFunction::compute_dN_dxi(double xi, double eta, double zeta) const
{
  (void)xi;
  (void)eta;
  (void)zeta;
  ES::M3x4d dN_dxi;
  dN_dxi << -1.0, 1.0, 0.0, 0.0,
    -1.0, 0.0, 1.0, 0.0,
    -1.0, 0.0, 0.0, 1.0;
  return dN_dxi;
}

void TetLinearShapeFunction::compute_N(double xi, double eta, double zeta,
  ES::RefVecXd N_out) const
{
  if (N_out.size() != kNumNodes)
    throw std::invalid_argument("TetLinearShapeFunction::compute_N output has the wrong size.");
  N_out = compute_N(xi, eta, zeta);
}

void TetLinearShapeFunction::compute_dN_dxi(double xi, double eta, double zeta,
  ES::RefMatXd dN_dxi) const
{
  (void)xi;
  (void)eta;
  (void)zeta;
  if (dN_dxi.rows() != 3 || dN_dxi.cols() != kNumNodes)
    throw std::invalid_argument("TetLinearShapeFunction::compute_dN_dxi output has the wrong shape.");
  dN_dxi = compute_dN_dxi(xi, eta, zeta);
}

ES::V3d TetLinearShapeFunction::nodeCoords(int node) const
{
  ES::V3d xi;
  switch (node) {
  case 0: xi << 0.0, 0.0, 0.0; break;
  case 1: xi << 1.0, 0.0, 0.0; break;
  case 2: xi << 0.0, 1.0, 0.0; break;
  case 3: xi << 0.0, 0.0, 1.0; break;
  default: xi.setZero(); break;
  }
  return xi;
}

ES::M3d tetLinearComputeDs(const ES::V12d &x)
{
  ES::M3d D;
  D.col(0) = ES::V3d(x[3], x[4], x[5]) - ES::V3d(x[0], x[1], x[2]);
  D.col(1) = ES::V3d(x[6], x[7], x[8]) - ES::V3d(x[0], x[1], x[2]);
  D.col(2) = ES::V3d(x[9], x[10], x[11]) - ES::V3d(x[0], x[1], x[2]);
  return D;
}

ES::M9x12d tetLinearComputeDFDx(const ES::M3d &D)
{
  ES::M9x12d dF = ES::M9x12d::Zero();

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

  return dF;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
