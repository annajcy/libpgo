#pragma once

#include "simulation/simulationMesh.h"

#include <memory>

namespace pgo
{
namespace Mesh
{
class TriMeshGeo;
}
namespace SolidDeformationModel
{

std::shared_ptr<SimulationMesh> loadTriangleMesh(
  const Mesh::TriMeshGeo &triMeshGeo);

void computeTriangleUV(SimulationMesh &mesh, double scaleFactor);

}  // namespace SolidDeformationModel
}  // namespace pgo
