#pragma once

#include "shellKernel.h"

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

// KoiterShellKernel
//
// Per-element rest-geometry precomputation plus first/second fundamental form
// kinematics for the Koiter thin-shell model.  The 6-node / 18-dof topology
// and oppVtx mapping are fixed for the Koiter triangle shell.
//
// Owns: rest positions, hasVtx mask, cached restI/restII/restArea.
//
// Missing-neighbor contract:
//   hasVtx_[i] == false for i in [3,5] causes the kernel to skip the
//   opposite-normal contribution for that edge.
//   Missing-neighbor restX_ entries are not read.

class KoiterShellKernel : public ShellKernel
{
public:
  static constexpr int numNodes = 6;
  static constexpr int localDofs = 18;
  static constexpr int numTriangleNodes = 3;

  static constexpr int oppVtx[3] = { 4, 5, 3 };

  KoiterShellKernel(const double restX[18], const bool hasVtx[6]);

  int getNumNodes() const override { return numNodes; }
  int getLocalDofs() const override { return localDofs; }

  ES::M2d compute_a_and_derivatives(
    const ES::V3d x[3],
    Eigen::Matrix<double, 4, 9> *da_dx,
    ES::M9d ahess[4]) const override;

  ES::M2d compute_b_and_derivatives(
    const ES::V3d x[6],
    Eigen::Matrix<double, 4, 18> *db_dx,
    ES::M18d bhess[4]) const override;

  const bool *hasVtx() const override { return hasVtx_; }
  const ES::M2d &restI() const override { return restI_; }
  const ES::M2d &restII() const override { return restII_; }
  double restArea() const override { return restArea_; }

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
