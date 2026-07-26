#pragma once

#include "shellElementMapping.h"

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

// KoiterShellElementMapping
//
// Per-element rest-geometry precomputation plus first/second fundamental form
// mapping for the Koiter thin-shell model.  The 6-node / 18-dof topology
// and oppVtx mapping are fixed for the Koiter triangle shell.
//
// Owns: rest positions, hasVtx mask, cached restI/restII/restArea.
//
// Missing-neighbor contract:
//   hasVtx_[i] == false for i in [3,5] causes the mapping to skip the
//   opposite-normal contribution for that edge.
//   Missing-neighbor restX_ entries are not read.

class KoiterShellElementMapping : public ShellElementMapping
{
public:
  static constexpr int numNodes = 6;
  static constexpr int localDofs = 18;
  static constexpr int numTriangleNodes = 3;

  static constexpr std::array<int, 3> oppVtx = { 4, 5, 3 };

  KoiterShellElementMapping(const ES::V18d &restX, const std::array<bool, 6> &hasVtx);

  int getNumNodes() const override { return numNodes; }
  int getLocalDofs() const override { return localDofs; }

  ES::M2d compute_a(const APositions &x) const override;
  ES::M4x9d compute_da_dx(const APositions &x) const override;
  ES::M9x36d compute_d2a_dx2(const APositions &x) const override;

  ES::M2d compute_b(const BPositions &x) const override;
  ES::M4x18d compute_db_dx(const BPositions &x) const override;
  ES::M18x72d compute_d2b_dx2(const BPositions &x) const override;

  const std::array<bool, 6> &hasVtx() const override { return hasVtx_; }
  const ES::M2d &restI() const override { return restI_; }
  const ES::M2d &restII() const override { return restII_; }
  double restArea() const override { return restArea_; }

private:
  struct AResult
  {
    ES::M2d value = ES::M2d::Zero();
    ES::M4x9d derivative = ES::M4x9d::Zero();
    std::array<ES::M9d, 4> hessian{};
  };

  struct BResult
  {
    ES::M2d value = ES::M2d::Zero();
    ES::M4x18d derivative = ES::M4x18d::Zero();
    std::array<ES::M18d, 4> hessian{};
  };

  struct SecondFundamentalFormResult
  {
    ES::V3d value = ES::V3d::Zero();
    ES::M3x18d derivative = ES::M3x18d::Zero();
    std::array<ES::M18d, 3> hessian{};
  };

  struct FaceNormalResult
  {
    ES::V3d value = ES::V3d::Zero();
    ES::M3x9d derivative = ES::M3x9d::Zero();
    std::array<ES::M9d, 3> hessian{};
  };

  AResult compute_a_impl(const APositions &x, bool computeDerivative, bool computeHessian) const;
  BResult compute_b_impl(const BPositions &x, bool computeDerivative, bool computeHessian) const;
  SecondFundamentalFormResult secondFundamentalFormEntries(
    const BPositions &x, bool computeDerivative, bool computeHessian) const;

  static ES::M3d crossMatrix(const Eigen::Vector3d &v);

  FaceNormalResult faceNormal(
    const ES::V3d &x0, const ES::V3d &x1, const ES::V3d &x2,
    bool computeDerivative, bool computeHessian) const;

  std::array<ES::V3d, 6> restX_;
  std::array<bool, 6> hasVtx_;
  ES::M2d restI_, restII_;
  double restArea_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
