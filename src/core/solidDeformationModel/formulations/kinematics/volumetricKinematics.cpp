#include "volumetricKinematics.h"

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

VolumetricKinematics::VolumetricKinematics(
  const double *restPositions,
  const Basis &basis, const Quadrature &quadrature):
  VolumetricKinematics(
    restPositions, basis.clone(), quadrature.clone())
{
}

VolumetricKinematics::VolumetricKinematics(
  const double *restPositions,
  std::unique_ptr<Basis> basis, std::unique_ptr<Quadrature> quadrature):
  basis_(std::move(basis)),
  quadrature_(std::move(quadrature))
{
  if (!basis_) {
    throw std::invalid_argument("VolumetricKinematics requires a non-null basis.");
  }
  if (!quadrature_) {
    throw std::invalid_argument("VolumetricKinematics requires a non-null quadrature.");
  }

  numNodes_ = basis_->numNodes();
  localDofs_ = basis_->localDofs();
  numQuadPts_ = quadrature_->numPoints();

  M3xN X(3, numNodes_);
  for (int vi = 0; vi < numNodes_; vi++) {
    X.col(vi) = ES::V3d(restPositions[vi * 3 + 0],
                          restPositions[vi * 3 + 1],
                          restPositions[vi * 3 + 2]);
  }

  dN_dxi_.resize(numQuadPts_, M3xN(3, numNodes_));
  restDmInv_.resize(numQuadPts_);
  dN_dX_.resize(numQuadPts_, M3xN(3, numNodes_));
  rest_dFdx_.resize(numQuadPts_, M9xNDOF(9, localDofs_));
  weightDetJ_.resize(numQuadPts_);
  restBm_.resize(numQuadPts_, M3xN(3, numNodes_));

  std::vector<double> dN_flat(3 * numNodes_);
  for (int q = 0; q < numQuadPts_; q++) {
    double xi[3];
    quadrature_->point(q, xi);
    basis_->dN_dxi(xi[0], xi[1], xi[2], dN_flat.data());
    dN_dxi_[q] = Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::ColMajor>>(
      dN_flat.data(), 3, numNodes_);
    const M3xN &dN_dxi = dN_dxi_[q];

    ES::M3d Dm = X * dN_dxi.transpose();
    double detDm = Dm.determinant();

    restDmInv_[q] = Dm.fullPivLu().inverse();

    dN_dX_[q] = restDmInv_[q].transpose() * dN_dxi;

    weightDetJ_[q] = std::abs(detDm) * quadrature_->weight(q);

    restBm_[q] = weightDetJ_[q] * dN_dX_[q];

    rest_dFdx_[q].setZero(9, localDofs_);
    for (int vi = 0; vi < numNodes_; vi++) {
      for (int coord = 0; coord < 3; coord++) {
        const int dof = vi * 3 + coord;
        for (int deriv = 0; deriv < 3; deriv++) {
          rest_dFdx_[q](deriv * 3 + coord, dof) = dN_dX_[q](deriv, vi);
        }
      }
    }
  }
}

void VolumetricKinematics::computeFref(
  const double *xLocal, int q, double F[9]) const
{
  M3xN xMat(3, numNodes_);
  for (int vi = 0; vi < numNodes_; vi++) {
    xMat.col(vi) = ES::V3d(xLocal[vi * 3 + 0],
                             xLocal[vi * 3 + 1],
                             xLocal[vi * 3 + 2]);
  }

  Eigen::Map<ES::M3d> FMap(F);
  FMap = xMat * dN_dxi_[q].transpose() * restDmInv_[q];
}

void VolumetricKinematics::computedFrefdx(
  int q, double *dFdx) const
{
  Eigen::Map<M9xNDOF> dFdxMap(dFdx, 9, localDofs_);
  dFdxMap = rest_dFdx_[q];
}

}  // namespace SolidDeformationModel
}  // namespace pgo
