#pragma once

#include "simulation/simulationMesh.h"

#include <memory>

namespace pgo
{
namespace VolumetricMeshes
{
class CubicMesh;
class TetMesh;
}
namespace SolidDeformationModel
{

std::shared_ptr<SimulationMesh> loadTetMesh(const VolumetricMeshes::TetMesh &tetmesh);
std::shared_ptr<SimulationMesh> loadCubicMesh(const VolumetricMeshes::CubicMesh &cubicMesh);

}  // namespace SolidDeformationModel
}  // namespace pgo
