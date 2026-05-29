#pragma once

#include "EigenSupport.h"

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

// FundamentalFormsKernel
//
// Per-element rest-geometry precomputation plus first/second fundamental form
// kinematics for the Koiter thin-shell model.  Not templated — the 6-node /
// 18-dof topology and oppVtx mapping are fixed for the Koiter shell.
//
// Owns: rest positions, hasVtx mask, cached restI/restII/restArea.
// Does NOT depend on ElasticModel, PlasticModel, or DeformationModelCacheData.
//
// Missing-neighbor contract:
//   hasVtx_[i] == false for i in [3,5] causes the kernel to skip the
//   opposite-normal contribution for that edge, exactly as the legacy code does.
//   Missing-neighbor restX_ entries are not read.

class FundamentalFormsKernel
{
public:
  static constexpr int numNodes = 6;
  static constexpr int localDofs = 18;
  static constexpr int numTriangleNodes = 3;

  static constexpr int oppVtx[3] = { 4, 5, 3 };

  FundamentalFormsKernel(const double restX[18], const bool hasVtx[6]);

  ES::M2d compute_a_and_derivatives(
    const ES::V3d x[3],
    Eigen::Matrix<double, 4, 9> *da_dx,
    ES::M9d ahess[4]) const;

  ES::M2d compute_b_and_derivatives(
    const ES::V3d x[6],
    Eigen::Matrix<double, 4, 18> *db_dx,
    ES::M18d bhess[4]) const;

  const bool *hasVtx() const { return hasVtx_; }
  const ES::M2d &restI() const { return restI_; }
  const ES::M2d &restII() const { return restII_; }
  double restArea() const { return restArea_; }

private:
  ES::V3d secondFundamentalFormEntries(
    const ES::V3d x[6],
    Eigen::Matrix<double, 3, 18> *derivative,
    ES::M18d hessian[3]) const;

  static ES::M3d crossMatrix(const Eigen::Vector3d &v);

  ES::V3d faceNormal(
    const ES::V3d x0, const ES::V3d x1, const ES::V3d x2,
    Eigen::Matrix<double, 3, 9> *derivative,
    ES::M9d hessian[3]) const;

  ES::V3d restX_[6];
  bool hasVtx_[6];
  ES::M2d restI_, restII_;
  double restArea_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
