#pragma once

#include "simulation/simulationMeshBase.h"
#include "simulation/simulationAsset.h"

namespace pgo
{
namespace VolumetricMeshes
{
class CubicMesh;
class TetMesh;
}
namespace SolidDeformationModel
{

std::unique_ptr<SimulationAsset> loadTetMesh(const VolumetricMeshes::TetMesh &tetmesh);
std::unique_ptr<SimulationAsset> loadCubicMesh(const VolumetricMeshes::CubicMesh &cubicMesh);

}  // namespace SolidDeformationModel
}  // namespace pgo
