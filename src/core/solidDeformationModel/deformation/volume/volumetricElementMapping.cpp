#include "volumetricElementMapping.h"

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

VolumetricElementMapping::VolumetricElementMapping(
  std::span<const double> restPositions,
  const ShapeFunction &basis, const Quadrature &quadrature):
  VolumetricElementMapping(
    restPositions, basis.clone(), quadrature.clone())
{
}

VolumetricElementMapping::VolumetricElementMapping(
  std::span<const double> restPositions,
  std::unique_ptr<ShapeFunction> basis, std::unique_ptr<Quadrature> quadrature):
  basis_(std::move(basis)),
  quadrature_(std::move(quadrature))
{
  if (!basis_) {
    throw std::invalid_argument("VolumetricElementMapping requires a non-null basis.");
  }
  if (!quadrature_) {
    throw std::invalid_argument("VolumetricElementMapping requires a non-null quadrature.");
  }

  numNodes_ = basis_->numNodes();
  localDofs_ = basis_->localDofs();
  numQuadPts_ = quadrature_->numPoints();

  if (restPositions.size() != static_cast<std::size_t>(3 * numNodes_)) {
    throw std::invalid_argument("VolumetricElementMapping rest-position buffer has the wrong size.");
  }

  M3xN restCoefficients(3, numNodes_);
  for (int node = 0; node < numNodes_; node++) {
    restCoefficients.col(node) = ES::V3d(restPositions[node * 3 + 0],
      restPositions[node * 3 + 1],
      restPositions[node * 3 + 2]);
  }

  dN_dxi_.resize(numQuadPts_, M3xN(3, numNodes_));
  restDmInv_.resize(numQuadPts_);
  dN_dX_.resize(numQuadPts_, M3xN(3, numNodes_));
  rest_dFdx_.resize(numQuadPts_, M9xNDOF(9, localDofs_));
  weightDetJ_.resize(numQuadPts_);
  restBm_.resize(numQuadPts_, M3xN(3, numNodes_));

  M3xN dN_dxiBuffer(3, numNodes_);
  for (int q = 0; q < numQuadPts_; q++) {
    const ES::V3d xi = quadrature_->point(q);
    basis_->compute_dN_dxi(xi[0], xi[1], xi[2], dN_dxiBuffer);
    dN_dxi_[q] = dN_dxiBuffer;
    const M3xN &dN_dxi = dN_dxi_[q];

    ES::M3d Dm = restCoefficients * dN_dxi.transpose();
    double detDm = Dm.determinant();

    restDmInv_[q] = Dm.fullPivLu().inverse();

    dN_dX_[q] = restDmInv_[q].transpose() * dN_dxi;

    weightDetJ_[q] = std::abs(detDm) * quadrature_->weight(q);

    restBm_[q] = weightDetJ_[q] * dN_dX_[q];

    rest_dFdx_[q].setZero(9, localDofs_);
    for (int node = 0; node < numNodes_; node++) {
      for (int coord = 0; coord < 3; coord++) {
        const int dof = node * 3 + coord;
        for (int deriv = 0; deriv < 3; deriv++) {
          rest_dFdx_[q](deriv * 3 + coord, dof) = dN_dX_[q](deriv, node);
        }
      }
    }
  }
}

ES::M3d VolumetricElementMapping::computeFref(
  std::span<const double> xLocal, int q) const
{
  if (q < 0 || q >= numQuadPts_)
    throw std::out_of_range("VolumetricElementMapping quadrature index is out of range.");
  if (xLocal.size() != static_cast<std::size_t>(localDofs_)) {
    throw std::invalid_argument("VolumetricElementMapping local-position buffer has the wrong size.");
  }

  M3xN coefficients(3, numNodes_);
  for (int node = 0; node < numNodes_; node++) {
    coefficients.col(node) = ES::V3d(xLocal[node * 3 + 0],
      xLocal[node * 3 + 1],
      xLocal[node * 3 + 2]);
  }

  return coefficients * dN_dxi_[q].transpose() * restDmInv_[q];
}

void VolumetricElementMapping::computedFrefdx(
  int q, M9xNDOF &dFdx) const
{
  if (q < 0 || q >= numQuadPts_)
    throw std::out_of_range("VolumetricElementMapping quadrature index is out of range.");
  if (dFdx.rows() != 9 || dFdx.cols() != localDofs_)
    throw std::invalid_argument("VolumetricElementMapping dFrefdx output has the wrong shape.");
  dFdx = rest_dFdx_[q];
}

}  // namespace SolidDeformationModel
}  // namespace pgo
