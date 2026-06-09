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
    double p[3];
    mesh.getVertex(vi, p);
    rest[vi * 3 + 0] = p[0];
    rest[vi * 3 + 1] = p[1];
    rest[vi * 3 + 2] = p[2];
  }
  return rest;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
