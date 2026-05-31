#include "volumetricKernel.h"

#include <cmath>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

VolumetricKernel::VolumetricKernel(
  const double *restPositions,
  const Basis &basis, const Quadrature &quadrature)
{
  numNodes_ = basis.numNodes();
  localDofs_ = basis.localDofs();
  numQuadPts_ = quadrature.numPoints();

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
    quadrature.point(q, xi);

    basis.dN_dxi(xi[0], xi[1], xi[2], dN_flat.data());

    dN_dxi_[q] = Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::ColMajor>>(
      dN_flat.data(), 3, numNodes_);

    ES::M3d Dm = X * dN_dxi_[q].transpose();
    double detDm = Dm.determinant();

    restDmInv_[q] = Dm.fullPivLu().inverse();

    dN_dX_[q] = restDmInv_[q].transpose() * dN_dxi_[q];

    weightDetJ_[q] = std::abs(detDm) * quadrature.weight(q);

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

void VolumetricKernel::computeFref(
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

void VolumetricKernel::computedFrefdx(
  int q, double *dFdx) const
{
  Eigen::Map<M9xNDOF> dFdxMap(dFdx, 9, localDofs_);
  dFdxMap = rest_dFdx_[q];
}

}  // namespace SolidDeformationModel
}  // namespace pgo
