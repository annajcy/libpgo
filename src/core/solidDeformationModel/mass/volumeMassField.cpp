#include "volumeMassField.h"

#include "simulation/simulationMesh.h"

#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

bool VolumeMassField::compatibleWith(SimulationMeshType meshType) const
{
  return meshType == SimulationMeshType::TET || meshType == SimulationMeshType::CUBIC;
}

ConstantVolumeDensity::ConstantVolumeDensity(double density)
  : density_(density)
{
  if (!(density > 0.0)) {
    throw std::invalid_argument("ConstantVolumeDensity requires density > 0");
  }
}

ElementwiseVolumeDensity::ElementwiseVolumeDensity(EigenSupport::VXd densities)
  : densities_(std::move(densities))
{
  if (densities_.size() == 0 || (densities_.array() <= 0.0).any()) {
    throw std::invalid_argument("ElementwiseVolumeDensity requires positive per-element densities");
  }
}

void ElementwiseVolumeDensity::validate(const SimulationMesh &mesh) const
{
  VolumeMassField::validate(mesh);
  if (static_cast<int>(densities_.size()) != mesh.getNumElements()) {
    throw std::invalid_argument("ElementwiseVolumeDensity size does not match mesh element count");
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo
