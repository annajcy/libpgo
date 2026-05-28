#pragma once

#include <cmath>

namespace pgo
{
namespace SolidDeformationModel
{

// Single-point quadrature for the constant-strain tetrahedron.
// Point at reference centroid (1/4, 1/4, 1/4), reference weight = 1/6.
// Used with TetP1Basis for tet P1 deformation gradient formulation.
// For tet P1，point is conventional, only total reference weight matters.
class TetP1DefaultQuadrature
{
public:
  static constexpr int numPoints = 1;

  static void point(int, double xi[3])
  {
    xi[0] = 0.25;
    xi[1] = 0.25;
    xi[2] = 0.25;
  }

  static double weight(int) { return 1.0 / 6.0; }
};

}  // namespace SolidDeformationModel
}  // namespace pgo
