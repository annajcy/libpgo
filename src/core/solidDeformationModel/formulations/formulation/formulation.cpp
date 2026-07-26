#include "formulation.h"

#include "formulations/dof/vertex3DofLayout.h"
#include "simulation/simulationMesh.h"

namespace pgo
{
namespace SolidDeformationModel
{

std::unique_ptr<DofLayout> Formulation::createDofLayout(const SimulationMesh &mesh) const
{
  return std::make_unique<Vertex3DofLayout>(mesh);
}

EigenSupport::VXd Formulation::buildGlobalRestDofs(const SimulationMesh &mesh) const
{
  const int nvtx = mesh.getNumVertices();
  EigenSupport::VXd rest(nvtx * 3);
  for (int vi = 0; vi < nvtx; vi++) {
    rest.segment<3>(vi * 3) = mesh.getVertex(vi);
  }
  return rest;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
