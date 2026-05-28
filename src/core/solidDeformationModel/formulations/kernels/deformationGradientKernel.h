#pragma once

#include "EigenSupport.h"

#include <array>
#include <cmath>

namespace ES = pgo::EigenSupport;

namespace pgo
{
namespace SolidDeformationModel
{

// DeformationGradientKernel<Basis, Quadrature>
//
// Rest-geometry precomputation and deformation-gradient kinematics
// shared by tet P1 and hex trilinear formulations.
//
// For each quadrature point, precomputes:
//   dN_dxi[q]     - shape derivatives in reference coords (3 x numNodes)
//   restDmInv[q]  - inverse of rest Jacobian (3x3)
//   dN_dX[q]      - shape derivatives in physical coords (3 x numNodes)
//   rest_dFdx[q]  - dF/dx at rest configuration (9 x localDofs)
//   weightDetJ[q] - |det(Dm)| * quadrature weight
//   restBm[q]     - weightDetJ * dN_dX (3 x numNodes)
//
// Runtime API:
//   computeFref(xLocal, q, F)   -> F = x * dN_dxi^T * DmInv
//   F at quad point q from local positions

template<class Basis, class Quadrature>
class DeformationGradientKernel
{
public:
  static constexpr int numNodes = Basis::numNodes;
  static constexpr int localDofs = Basis::localDofs;
  static constexpr int numQuadPts = Quadrature::numPoints;

  using M3xN = Eigen::Matrix<double, 3, numNodes>;
  using M9xNDOF = Eigen::Matrix<double, 9, localDofs>;

  explicit DeformationGradientKernel(const double restPositions[localDofs]);

  int numQuadraturePoints() const { return numQuadPts; }
  double weightDetJ(int q) const { return weightDetJ_[q]; }

  void computeFref(const double *xLocal, int q, double F[9]) const;
  void computedFrefdx(int q, double *dFdx) const;

  // Internal accessors for ElementModel
  const ES::M3d &restDmInv(int q) const { return restDmInv_[q]; }
  const M3xN &restBm(int q) const { return restBm_[q]; }
  const M9xNDOF &rest_dFdx(int q) const { return rest_dFdx_[q]; }

private:
  std::array<M3xN, numQuadPts> dN_dxi_;
  std::array<ES::M3d, numQuadPts> restDmInv_;
  std::array<M3xN, numQuadPts> dN_dX_;
  std::array<M9xNDOF, numQuadPts> rest_dFdx_;
  std::array<double, numQuadPts> weightDetJ_;
  std::array<M3xN, numQuadPts> restBm_;
};

// ============================================================
// Implementation
// ============================================================

template<class Basis, class Quadrature>
DeformationGradientKernel<Basis, Quadrature>::DeformationGradientKernel(
  const double restPositions[localDofs])
{
  // Build the 3 x numNodes rest position matrix.
  M3xN X;
  for (int vi = 0; vi < numNodes; vi++) {
    X.col(vi) = ES::V3d(restPositions[vi * 3 + 0],
                          restPositions[vi * 3 + 1],
                          restPositions[vi * 3 + 2]);
  }

  for (int q = 0; q < numQuadPts; q++) {
    double xi[3];
    Quadrature::point(q, xi);

    // Evaluate basis derivatives at the quadrature point.
    double dN_flat[3 * numNodes];
    Basis::dN_dxi(xi[0], xi[1], xi[2], dN_flat);

    // dN_dxi as 3 x numNodes matrix.
    // Flattened column-major: dN_dxi(deriv, node) = dN_flat[deriv + 3 * node]
    dN_dxi_[q] = Eigen::Map<const Eigen::Matrix<double, 3, numNodes, Eigen::ColMajor>>(dN_flat);

    // Rest Jacobian: Dm = X * dN_dxi^T  (3x3)
    ES::M3d Dm = X * dN_dxi_[q].transpose();
    double detDm = Dm.determinant();

    restDmInv_[q] = Dm.fullPivLu().inverse();

    // Shape derivatives in physical coords.
    dN_dX_[q] = restDmInv_[q].transpose() * dN_dxi_[q];

    weightDetJ_[q] = std::abs(detDm) * Quadrature::weight(q);

    // B-matrix: weight * det(J) * dN_dX
    restBm_[q] = weightDetJ_[q] * dN_dX_[q];

    // Precompute dF/dx.
    rest_dFdx_[q].setZero();
    for (int vi = 0; vi < numNodes; vi++) {
      for (int coord = 0; coord < 3; coord++) {
        const int dof = vi * 3 + coord;
        for (int deriv = 0; deriv < 3; deriv++) {
          rest_dFdx_[q](deriv * 3 + coord, dof) = dN_dX_[q](deriv, vi);
        }
      }
    }
  }
}

template<class Basis, class Quadrature>
void DeformationGradientKernel<Basis, Quadrature>::computeFref(
  const double *xLocal, int q, double F[9]) const
{
  M3xN xMat;
  for (int vi = 0; vi < numNodes; vi++) {
    xMat.col(vi) = ES::V3d(xLocal[vi * 3 + 0],
                             xLocal[vi * 3 + 1],
                             xLocal[vi * 3 + 2]);
  }

  // Fref = x * dN_dxi^T * DmInv
  Eigen::Map<ES::M3d> FMap(F);
  FMap = xMat * dN_dxi_[q].transpose() * restDmInv_[q];
}

template<class Basis, class Quadrature>
void DeformationGradientKernel<Basis, Quadrature>::computedFrefdx(
  int q, double *dFdx) const
{
  Eigen::Map<M9xNDOF> dFdxMap(dFdx);
  dFdxMap = rest_dFdx_[q];
}

}  // namespace SolidDeformationModel
}  // namespace pgo
