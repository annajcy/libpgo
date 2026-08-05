#pragma once

#include "quadrature.h"

namespace pgo
{
namespace SolidDeformationModel
{

// 4-point degree-2 tetrahedron rule (reference volume 1/6, weight 1/24 each).
// Exact for the quadratic N^T N mass integrand of the linear tet; used as
// TetLinearFormulation::massQuadrature() because the elastic 1-point rule
// under-integrates it.
class TetDegree2Quadrature : public Quadrature
{
public:
  static constexpr int kNumPoints = 4;

  int numPoints() const override { return kNumPoints; }
  EigenSupport::V3d point(int i) const override;
  double weight(int i) const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
