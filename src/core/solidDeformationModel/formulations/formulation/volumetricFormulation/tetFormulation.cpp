#include "tetFormulation.h"

#include "simulation/simulationMesh.h"

namespace pgo
{
namespace SolidDeformationModel
{

SimulationMeshType TetFormulation::compatibleMeshType() const
{
  return SimulationMeshType::TET;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
