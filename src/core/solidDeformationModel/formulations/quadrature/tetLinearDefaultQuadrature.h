#pragma once

#include "quadrature.h"

namespace pgo
{
namespace SolidDeformationModel
{

// Single-point quadrature for the constant-strain tetrahedron.
// Point at reference centroid (1/4, 1/4, 1/4), reference weight = 1/6.
// Used with TetLinearShapeFunction for tet P1 deformation gradient formulation.
class TetLinearDefaultQuadrature : public Quadrature
{
public:
  static constexpr int kNumPoints = 1;

  int numPoints() const override { return kNumPoints; }
  std::unique_ptr<Quadrature> clone() const override { return std::make_unique<TetLinearDefaultQuadrature>(*this); }
  EigenSupport::V3d point(int i) const override;
  double weight(int i) const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
