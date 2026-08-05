#pragma once

#include "shapeFunction.h"

namespace pgo
{
namespace SolidDeformationModel
{

// Tricubic Hermite basis on the unit hex [0,1]^3.
//
// 8 corners x 8 Hermite modes = 64 scalar basis functions; with 3 spatial coords that is
// 192 local DOFs per element. Each corner carries the modes (value + the 7 derivative modes):
//
//   mode 0: VALUE            (dxi,deta,dzeta) = (0,0,0)
//   mode 1: D_XI             (1,0,0)
//   mode 2: D_ETA            (0,1,0)
//   mode 3: D_ZETA           (0,0,1)
//   mode 4: D_XI_ETA         (1,1,0)
//   mode 5: D_XI_ZETA        (1,0,1)
//   mode 6: D_ETA_ZETA       (0,1,1)
//   mode 7: D_XI_ETA_ZETA    (1,1,1)
//
// Corner ordering matches CubicLinearShapeFunction (so the 8 mesh hex vertices map 1:1):
//   corner c parametric coords come from kVertexAlpha/Beta/Gamma.
//
// Scalar basis index (the "node" the mapping sees): node = corner * 8 + mode.
// Local DOF index: node * 3 + coord. dN_dxi is column-major 3 x 64: dN[deriv + 3*node].
//
// Defining property (tensor-product cubic Hermite): given a separable cubic field
// f(xi,eta,zeta) = p(xi) q(eta) r(zeta) with DOFs
//   DOF(corner c, mode m) = D^{m_xi}p(c_xi) * D^{m_eta}q(c_eta) * D^{m_zeta}r(c_zeta)
// (D^0 g = g, D^1 g = g'), the basis reproduces f and its first derivatives exactly.
class CubicTricubicHermiteShapeFunction : public ShapeFunction
{
public:
  static constexpr int kNumNodes = 64;
  static constexpr int kLocalDofs = 192;

  int numNodes() const override { return kNumNodes; }
  int localDofs() const override { return kLocalDofs; }

  EigenSupport::V64d compute_N(double xi, double eta, double zeta) const;
  EigenSupport::M3x64d compute_dN_dxi(double xi, double eta, double zeta) const;
  void compute_N(double xi, double eta, double zeta,
    EigenSupport::RefVecXd N_out) const override;
  void compute_dN_dxi(double xi, double eta, double zeta,
    EigenSupport::RefMatXd dN_out) const override;
  EigenSupport::V3d nodeCoords(int node) const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
