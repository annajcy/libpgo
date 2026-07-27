#pragma once

#include "simulation/simulationMeshBase.h"
#include "simulation/import/simulationImportResult.h"

namespace pgo
{
namespace VolumetricMeshes
{
class CubicMesh;
class TetMesh;
}
namespace SolidDeformationModel
{

std::unique_ptr<SimulationImportResult> loadTetMesh(const VolumetricMeshes::TetMesh &tetmesh);
std::unique_ptr<SimulationImportResult> loadCubicMesh(const VolumetricMeshes::CubicMesh &cubicMesh);

}  // namespace SolidDeformationModel
}  // namespace pgo
