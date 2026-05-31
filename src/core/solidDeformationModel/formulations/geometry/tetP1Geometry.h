#pragma once

#include "EigenSupport.h"

namespace pgo
{
namespace SolidDeformationModel
{
namespace ES = pgo::EigenSupport;

// Standalone tet P1 geometry helpers for callers that don't have a
// VolumetricKernel available (constraint functions, tests).
// Rest-geometry Dm/DmInv/dFdx belong in the kernel; only keep helpers
// that operate on current (deformed) positions or externally-supplied DmInv.

// Deformed Ds matrix from current positions x[12].
inline void tetP1ComputeDs(const double x[12], double Ds[9])
{
  ES::M3d D;
  D.col(0) = ES::V3d(x[3], x[4], x[5]) - ES::V3d(x[0], x[1], x[2]);
  D.col(1) = ES::V3d(x[6], x[7], x[8]) - ES::V3d(x[0], x[1], x[2]);
  D.col(2) = ES::V3d(x[9], x[10], x[11]) - ES::V3d(x[0], x[1], x[2]);
  (Eigen::Map<ES::M3d>(Ds)) = D;
}

// dF/dx from an externally-supplied DmInv[9] (e.g. setDmInv path).
inline void tetP1ComputeDFDx(const double DmInv[9], double dFdx[9 * 12])
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

  memcpy(dFdx, dF.data(), sizeof(double) * 9 * 12);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
