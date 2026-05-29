#pragma once

#include "quadrature.h"

namespace pgo
{
namespace SolidDeformationModel
{

// Single-point quadrature for the constant-strain tetrahedron.
// Point at reference centroid (1/4, 1/4, 1/4), reference weight = 1/6.
// Used with TetP1Basis for tet P1 deformation gradient formulation.
class TetP1DefaultQuadrature : public Quadrature
{
public:
  static constexpr int kNumPoints = 1;

  int numPoints() const override { return kNumPoints; }
  void point(int i, double xi[3]) const override;
  double weight(int i) const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
