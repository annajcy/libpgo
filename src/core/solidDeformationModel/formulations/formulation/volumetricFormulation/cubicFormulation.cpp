#include "cubicFormulation.h"

#include "simulation/simulationMesh.h"

namespace pgo
{
namespace SolidDeformationModel
{

SimulationMeshType CubicFormulation::compatibleMeshType() const
{
  return SimulationMeshType::CUBIC;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
