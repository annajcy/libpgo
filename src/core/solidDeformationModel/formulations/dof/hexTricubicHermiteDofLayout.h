#pragma once

#include "dofLayout.h"

namespace pgo
{
namespace SolidDeformationModel
{

class SimulationMesh;

// DOF layout for the regular-grid tricubic Hermite hex formulation.
//
// Each mesh vertex carries 24 DOFs = 8 Hermite modes x 3 coords. Global DOF index:
//   global = vertexIndex * 24 + mode * 3 + coord
// Local element DOFs = 8 corners x 8 modes x 3 coords = 192. Local index matches the basis
// "node = corner*8 + mode" ordering the kinematics consumes:
//   local = (corner * 8 + mode) * 3 + coord
//
// This is a pure index permutation (transform T = identity): it assumes the element-local
// xi/eta/zeta axes are aligned with the global axes so derivative modes are shared directly
// across elements. That holds for a regular axis-aligned (or affine-parallelepiped) grid -- the
// MVP scope. A general cubic mesh with flipped/rotated local axes would need gather = T*global /
// scatter = T^T*local, which this copy-by-index layout cannot express (see dofLayout.h).
class HexTricubicHermiteDofLayout : public DofLayout
{
public:
  static constexpr int kModesPerVertex = 8;
  static constexpr int kDofsPerVertex = 24;   // 8 modes * 3 coords
  static constexpr int kLocalDofs = 192;       // 8 corners * 24

  explicit HexTricubicHermiteDofLayout(const SimulationMesh &mesh);

  int numGlobalDofs() const override;
  int numLocalDofs(int ele) const override;

  void getGlobalDofIndices(int ele, std::vector<int> &indices) const override;

  void gather(int ele, const double *global, double *local) const override;
  void scatterAddGradient(int ele, const double *local, double *global) const override;

  void addHessianSparsity(int ele,
    std::vector<EigenSupport::TripletD> &entries) const override;

  void buildLocalToGlobalMatrixIndices(int ele,
    const EigenSupport::SpMatD &KTemplate,
    DynamicIndexMatrix &indices) const override;

private:
  const SimulationMesh &mesh_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
