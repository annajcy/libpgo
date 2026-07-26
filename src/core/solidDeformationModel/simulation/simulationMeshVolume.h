#pragma once

#include "simulation/simulationMeshBase.h"

namespace pgo
{
namespace VolumetricMeshes
{
class CubicMesh;
class TetMesh;
}
namespace SolidDeformationModel
{

std::unique_ptr<SimulationMesh> loadTetMesh(const VolumetricMeshes::TetMesh &tetmesh);
std::unique_ptr<SimulationMesh> loadCubicMesh(const VolumetricMeshes::CubicMesh &cubicMesh);

}  // namespace SolidDeformationModel
}  // namespace pgo
