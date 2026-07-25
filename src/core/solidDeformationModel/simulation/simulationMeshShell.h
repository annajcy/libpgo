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

std::unique_ptr<SimulationMesh> loadShellMesh(
  // Copies one immutable material value to every triangle.
  const Mesh::TriMeshGeo &triMeshGeo, const SimulationMeshENuhMaterial &mat);
std::unique_ptr<SimulationMesh> loadShellMesh(
  // The field must contain exactly one value per triangle.  Palette indices
  // are validated when the ElementField is constructed.
  const Mesh::TriMeshGeo &triMeshGeo,
  ElementField<SimulationMeshENuhMaterial> materials);

}  // namespace SolidDeformationModel
}  // namespace pgo
