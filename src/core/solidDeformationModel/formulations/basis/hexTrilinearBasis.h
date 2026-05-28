#pragma once

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
class HexTrilinearBasis
{
public:
  static constexpr int numNodes = 8;
  static constexpr int localDofs = 24;

  // Shape function values at reference coordinate (alpha, beta, gamma).
  // N is length-8 array.
  static void N(double alpha, double beta, double gamma, double N[8]);

  // Shape function derivatives w.r.t. reference coordinates.
  // dN_dxi is 3x8 column-major: dN_dxi(deriv, node).
  static void dN_dxi(double alpha, double beta, double gamma, double dN_dxi[24]);

  // Node reference coordinates in (alpha, beta, gamma) parameter space.
  static void nodeCoords(int node, double xi[3]);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
