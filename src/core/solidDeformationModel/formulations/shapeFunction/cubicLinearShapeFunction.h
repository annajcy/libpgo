#pragma once

#include "shapeFunction.h"

namespace pgo
{
namespace SolidDeformationModel
{

// Reference-element interpolation for the trilinear hexahedron.
// Reference domain: alpha, beta, gamma in [0, 1]^3.
//
// Node ordering (matching CubicMeshDeformationModel convention):
//   v0=(0,0,0)  v1=(1,0,0)  v2=(1,1,0)  v3=(0,1,0)
//   v4=(0,0,1)  v5=(1,0,1)  v6=(1,1,1)  v7=(0,1,1)
//
// Shape functions:
//   N_i(alpha,beta,gamma) = L(alpha, i_alpha) * L(beta, i_beta) * L(gamma, i_gamma)
//   where L(x, vertexBit) = vertexBit ? x : (1 - x)
//
// dN_dxi is returned as a 3 x 8 column-major matrix:
//   row 0 = d/dalpha, row 1 = d/dbeta, row 2 = d/dgamma.
class CubicLinearShapeFunction : public ShapeFunction
{
public:
  static constexpr int kNumNodes = 8;
  static constexpr int kLocalDofs = 24;

  int numNodes() const override { return kNumNodes; }
  int localDofs() const override { return kLocalDofs; }

  EigenSupport::V8d compute_N(double alpha, double beta, double gamma) const;
  EigenSupport::M3x8d compute_dN_dxi(double alpha, double beta, double gamma) const;
  void compute_N(double alpha, double beta, double gamma,
    EigenSupport::RefVecXd N_out) const override;
  void compute_dN_dxi(double alpha, double beta, double gamma,
    EigenSupport::RefMatXd dN_dxi) const override;
  EigenSupport::V3d nodeCoords(int node) const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
