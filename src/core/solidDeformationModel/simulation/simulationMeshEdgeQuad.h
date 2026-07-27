#pragma once

#include "simulation/simulationMeshBase.h"

#include <memory>

namespace pgo
{
namespace Mesh
{
class TriMeshGeo;
}
namespace SolidDeformationModel
{

std::shared_ptr<SimulationMesh> loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo);

}  // namespace SolidDeformationModel
}  // namespace pgo
