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

std::unique_ptr<SimulationAsset> loadTriangleMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  const ImportedENuMaterial &mat);
std::unique_ptr<SimulationAsset> loadTriangleMesh(
  const Mesh::TriMeshGeo &triMeshGeo,
  ElementField<ImportedENuMaterial> materials);

void computeTriangleUV(SimulationMesh &mesh, double scaleFactor);

}  // namespace SolidDeformationModel
}  // namespace pgo
