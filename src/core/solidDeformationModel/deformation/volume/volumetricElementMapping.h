#pragma once

#include "EigenSupport.h"

#include "formulations/shapeFunction/shapeFunction.h"
#include "formulations/quadrature/quadrature.h"

#include <memory>
#include <span>
#include <vector>

namespace ES = pgo::EigenSupport;

namespace pgo
{
namespace SolidDeformationModel
{

// VolumetricElementMapping
//
// Rest-geometry precomputation and deformation-gradient mapping
// shared by volumetric formulations. The "nodes" here are scalar shape-function
// coefficients, not necessarily mesh vertices: tricubic Hermite has 64 such
// coefficients, each carrying a 3D vector, for 192 local DOFs.
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
//   computeFref(xLocal, q)      -> F = x * dN_dxi^T * DmInv
//   F at quad point q from local positions

class VolumetricElementMapping
{
public:
  using M3xN = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  using M9xNDOF = Eigen::Matrix<double, 9, Eigen::Dynamic>;

  VolumetricElementMapping(std::span<const double> restPositions,
    const ShapeFunction &basis, const Quadrature &quadrature);
  VolumetricElementMapping(std::span<const double> restPositions,
    std::unique_ptr<ShapeFunction> basis, std::unique_ptr<Quadrature> quadrature);

  int numQuadraturePoints() const { return numQuadPts_; }
  int numNodes() const { return numNodes_; }
  int localDofs() const { return localDofs_; }

  double weightDetJ(int q) const { return weightDetJ_[q]; }

  ES::M3d computeFref(std::span<const double> xLocal, int q) const;
  void computedFrefdx(int q, M9xNDOF &dFdx) const;

  const ES::M3d &restDmInv(int q) const { return restDmInv_[q]; }
  const M3xN &restBm(int q) const { return restBm_[q]; }
  const M9xNDOF &rest_dFdx(int q) const { return rest_dFdx_[q]; }

private:
  int numNodes_;
  int numQuadPts_;
  int localDofs_;

  std::unique_ptr<ShapeFunction> basis_;
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
