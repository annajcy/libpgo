#pragma once

#include "dofLayout.h"
#include "simulation/simulationMesh.h"

namespace pgo
{
namespace SolidDeformationModel
{

template<int DofsPerVertex>
class PerVertexDofLayout : public DofLayout
{
public:
  static constexpr int kDofsPerVertex = DofsPerVertex;

  explicit PerVertexDofLayout(const SimulationMesh &mesh):
    mesh_(mesh)
  {
  }

  int numGlobalDofs() const override
  {
    return mesh_.getNumVertices() * DofsPerVertex;
  }

  int numLocalDofs(int) const override
  {
    return mesh_.getNumElementVertices() * DofsPerVertex;
  }

  void getDofGroups(int ele, std::vector<DofGroup> &groups) const override
  {
    const int neleVtx = mesh_.getNumElementVertices();
    groups.clear();
    groups.reserve(neleVtx);
    for (int v = 0; v < neleVtx; v++) {
      const int vid = mesh_.getVertexIndex(ele, v);
      if (vid < 0)
        continue;
      groups.push_back({ v * DofsPerVertex, vid * DofsPerVertex, DofsPerVertex });
    }
  }

private:
  const SimulationMesh &mesh_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
