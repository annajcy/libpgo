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
  std::unique_ptr<Quadrature> clone() const override { return std::make_unique<TetDegree2Quadrature>(*this); }
  void point(int i, double xi[3]) const override;
  double weight(int i) const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
