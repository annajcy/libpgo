#pragma once

#include "shapeFunction.h"

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
class TetLinearShapeFunction : public ShapeFunction
{
public:
  static constexpr int kNumNodes = 4;
  static constexpr int kLocalDofs = 12;

  int numNodes() const override { return kNumNodes; }
  int localDofs() const override { return kLocalDofs; }
  std::unique_ptr<ShapeFunction> clone() const override { return std::make_unique<TetLinearShapeFunction>(*this); }

  void N(double xi, double eta, double zeta, double N_out[]) const override;
  void dN_dxi(double xi, double eta, double zeta, double dN_dxi[]) const override;
  void nodeCoords(int node, double xi[3]) const override;
};

// Tet-linear deformation geometry helpers for callers that do not have a
// VolumetricElementMapping instance available.
void tetLinearComputeDs(const double x[12], double Ds[9]);
void tetLinearComputeDFDx(const double DmInv[9], double dFdx[9 * 12]);

}  // namespace SolidDeformationModel
}  // namespace pgo
