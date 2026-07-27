#pragma once

#include "simulation/simulationMeshBase.h"
#include "simulation/simulationAsset.h"
#include "simulation/elementField.h"
#include "simulation/importedMaterial.h"

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
std::unique_ptr<SimulationAsset> loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  const ImportedENuMaterial &mat);
std::unique_ptr<SimulationAsset> loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  ElementField<ImportedENuMaterial> triangleMaterials);
std::unique_ptr<SimulationAsset> loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  const ImportedENuhMaterial &mat);
std::unique_ptr<SimulationAsset> loadEdgeQuadMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  ElementField<ImportedENuhMaterial> triangleMaterials);

}  // namespace SolidDeformationModel
}  // namespace pgo
