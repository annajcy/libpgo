#pragma once

namespace pgo
{
namespace SolidDeformationModel
{

// Reference-element interpolation for the linear tetrahedron (P1).
// Reference domain: xi, eta, zeta >= 0, xi + eta + zeta <= 1.
//
// Shape functions:
//   N0 = 1 - xi - eta - zeta    dN0/dxi = (-1, -1, -1)
//   N1 = xi                     dN1/dxi = ( 1,  0,  0)
//   N2 = eta                    dN2/dxi = ( 0,  1,  0)
//   N3 = zeta                   dN3/dxi = ( 0,  0,  1)
//
// dN_dxi is returned as a 3 x 4 column-major matrix:
//   row 0 = d/dxi, row 1 = d/deta, row 2 = d/dzeta.
class TetP1Basis
{
public:
  static constexpr int numNodes = 4;
  static constexpr int localDofs = 12;

  // Shape function values at reference coordinate (xi, eta, zeta).
  // N is length-4 array, node ordering: 0..3.
  static void N(double xi, double eta, double zeta, double N[4]);

  // Shape function derivatives w.r.t. reference coordinates.
  // dN_dxi is 3x4 column-major: dN_dxi(deriv, node).
  // For linear tet, derivatives are constant — parameters are
  // accepted for API uniformity but unused.
  static void dN_dxi(double xi, double eta, double zeta, double dN_dxi[12]);

  // Node reference coordinates in (xi, eta, zeta) parameter space.
  static void nodeCoords(int node, double xi[3]);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
