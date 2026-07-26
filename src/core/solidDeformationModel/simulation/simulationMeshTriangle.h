#pragma once

#include "simulation/simulationMeshBase.h"
#include "simulation/simulationMeshMaterial.h"

namespace pgo
{
namespace Mesh
{
class TriMeshGeo;
}
namespace SolidDeformationModel
{

std::unique_ptr<SimulationMesh> loadTriangleMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  const SimulationMeshENuMaterial &mat);
std::unique_ptr<SimulationMesh> loadTriangleMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  ElementField<SimulationMeshENuMaterial> materials);

void computeTriangleUV(SimulationMesh &mesh, double scaleFactor);

}  // namespace SolidDeformationModel
}  // namespace pgo
