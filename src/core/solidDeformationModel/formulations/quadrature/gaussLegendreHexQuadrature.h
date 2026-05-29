#pragma once

#include "quadrature.h"

namespace pgo
{
namespace SolidDeformationModel
{

// 2x2x2 Gauss-Legendre quadrature for the unit hex [0,1]^3.
// Gauss points at 0.5 +/- 0.5/sqrt(3) in each direction.
// Reference weights = 1/8 per point.
// Used with HexTrilinearBasis for hex trilinear deformation gradient formulation.
class GaussLegendreHexQuadrature2 : public Quadrature
{
public:
  static constexpr int kNumPoints = 8;

  int numPoints() const override { return kNumPoints; }
  void point(int i, double xi[3]) const override;
  double weight(int i) const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
