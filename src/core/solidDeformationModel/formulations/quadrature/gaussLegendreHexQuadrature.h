#pragma once

#include <cmath>

namespace pgo
{
namespace SolidDeformationModel
{

// 2x2x2 Gauss-Legendre quadrature for the unit hex [0,1]^3.
// Gauss points at 0.5 +/- 0.5/sqrt(3) in each direction.
// Reference weights = 1/8 per point.
// Used with HexTrilinearBasis for hex trilinear deformation gradient formulation.
class GaussLegendreHexQuadrature2
{
public:
  static constexpr int numPoints = 8;

  // Quadrature point ordering: innermost loop over gamma, then beta, then alpha
  // (matching the legacy CubicMeshDeformationModel convention).
  static void point(int i, double xi[3])
  {
    constexpr double offset = 0.5 / 1.7320508075688772;  // 0.5 / sqrt(3)
    constexpr double gp[2] = { 0.5 - offset, 0.5 + offset };

    // ia is outer loop, ig is inner loop (matching legacy convention)
    const int ia = i / 4;
    const int ib = (i / 2) % 2;
    const int ig = i % 2;

    xi[0] = gp[ia];
    xi[1] = gp[ib];
    xi[2] = gp[ig];
  }

  static double weight(int) { return 0.125; }  // 1/8
};

}  // namespace SolidDeformationModel
}  // namespace pgo
