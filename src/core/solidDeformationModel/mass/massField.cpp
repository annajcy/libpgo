#include "massField.h"

#include "simulation/simulationMesh.h"

#include <stdexcept>
#include <string>

namespace pgo
{
namespace SolidDeformationModel
{

void MassField::validate(const SimulationMesh &mesh) const
{
  if (!compatibleWith(mesh.getElementType())) {
    throw std::invalid_argument(
      std::string("mass field is incompatible with mesh type ") +
      meshTypeName(mesh.getElementType()));
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo
