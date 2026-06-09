#pragma once

#include "EigenSupport.h"

#include "../basis/basis.h"
#include "../quadrature/quadrature.h"

#include <memory>
#include <vector>

namespace ES = pgo::EigenSupport;

namespace pgo
{
namespace SolidDeformationModel
{

// VolumetricKinematics
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

class VolumetricKinematics
{
public:
  using M3xN = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  using M9xNDOF = Eigen::Matrix<double, 9, Eigen::Dynamic>;

  VolumetricKinematics(const double *restPositions,
    const Basis &basis, const Quadrature &quadrature);
  VolumetricKinematics(const double *restPositions,
    std::unique_ptr<Basis> basis, std::unique_ptr<Quadrature> quadrature);

  int numQuadraturePoints() const { return numQuadPts_; }
  int numNodes() const { return numNodes_; }
  int localDofs() const { return localDofs_; }

  double weightDetJ(int q) const { return weightDetJ_[q]; }

  void computeFref(const double *xLocal, int q, double F[9]) const;
  void computedFrefdx(int q, double *dFdx) const;

  const ES::M3d &restDmInv(int q) const { return restDmInv_[q]; }
  const M3xN &restBm(int q) const { return restBm_[q]; }
  const M9xNDOF &rest_dFdx(int q) const { return rest_dFdx_[q]; }

private:
  int numNodes_;
  int numQuadPts_;
  int localDofs_;

  std::unique_ptr<Basis> basis_;
  std::unique_ptr<Quadrature> quadrature_;
  std::vector<M3xN> dN_dxi_;
  std::vector<ES::M3d> restDmInv_;
  std::vector<M3xN> dN_dX_;
  std::vector<M9xNDOF> rest_dFdx_;
  std::vector<double> weightDetJ_;
  std::vector<M3xN> restBm_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
