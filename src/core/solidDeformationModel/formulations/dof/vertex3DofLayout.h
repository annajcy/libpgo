#pragma once

#include "dofLayout.h"

namespace pgo
{
namespace SolidDeformationModel
{

class SimulationMesh;

// DOF layout where each vertex carries 3 displacement DOFs (x, y, z).
// Global DOF index = vertexIndex * 3 + coordinate (0=x, 1=y, 2=z).
// Local DOFs = numElementVertices * 3.
//
// Negative vertex indices (used by shell elements as sentinels) produce
// zero local contribution — gather/scatter skip them.
//
// Full implementation (including the .cpp with actual gather/scatter/sparsity)
// arrives in Task 6.
class Vertex3DofLayout : public DofLayout
{
public:
  explicit Vertex3DofLayout(const SimulationMesh *mesh);

  int numGlobalDofs() const override;
  int numLocalDofs(int ele) const override;

  void gather(int ele, const double *global, double *local) const override;
  void scatterAddGradient(int ele, const double *local, double *global) const override;

  void addHessianSparsity(int ele,
    std::vector<EigenSupport::TripletD> &entries) const override;

  void buildLocalToGlobalMatrixIndices(int ele,
    const EigenSupport::SpMatD &KTemplate,
    DynamicIndexMatrix &indices) const override;

private:
  const SimulationMesh *mesh_ = nullptr;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
