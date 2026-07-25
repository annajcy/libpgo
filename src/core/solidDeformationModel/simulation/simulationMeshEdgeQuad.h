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

// Material fields are indexed by the source triangles.  The two materials on
// either side of each interior edge are averaged into the edge-quad material.
std::unique_ptr<SimulationMesh> loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  const SimulationMeshENuMaterial &mat);
std::unique_ptr<SimulationMesh> loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  ElementField<SimulationMeshENuMaterial> triangleMaterials);
std::unique_ptr<SimulationMesh> loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  const SimulationMeshENuhMaterial &mat);
std::unique_ptr<SimulationMesh> loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  ElementField<SimulationMeshENuhMaterial> triangleMaterials);

}  // namespace SolidDeformationModel
}  // namespace pgo
