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

std::unique_ptr<SimulationAsset> loadShellMesh(
  // Copies one immutable material value to every triangle.
  const Mesh::TriMeshGeo &triMeshGeo, const ImportedENuhMaterial &mat);
std::unique_ptr<SimulationAsset> loadShellMesh(
  // The field must contain exactly one value per triangle.  Palette indices
  // are validated when the ElementField is constructed.
  const Mesh::TriMeshGeo &triMeshGeo,
  ElementField<ImportedENuhMaterial> materials);

}  // namespace SolidDeformationModel
}  // namespace pgo
