#include "material/runtime/materialAssignment.h"

#include "simulation/simulationMesh.h"

#include <stdexcept>

namespace pgo::SolidDeformationModel
{
MaterialAssignment::MaterialAssignment(
  std::shared_ptr<const SimulationMesh> mesh,
  std::shared_ptr<const MaterialParameterization> parameterization,
  std::shared_ptr<const MaterialParameterData> parameterData,
  std::shared_ptr<const MaterialFrameField> materialFrames):
  mesh_(std::move(mesh)),
  parameterization_(std::move(parameterization)),
  parameterData_(std::move(parameterData)),
  materialFrames_(std::move(materialFrames))
{
  if (!mesh_ || !parameterization_ || !parameterData_ || !materialFrames_)
    throw std::invalid_argument(
      "MaterialAssignment requires mesh, parameterization, parameter data and material frames.");
  if (parameterization_->numElements() != mesh_->getNumElements())
    throw std::invalid_argument(
      "MaterialAssignment parameterization element count does not match mesh.");
  parameterization_->validate(*parameterData_);
  if (materialFrames_->numElements() != mesh_->getNumElements())
    throw std::invalid_argument(
      "MaterialAssignment material frame element count does not match mesh.");

  initialMaterialState_ = MaterialState(
    parameterData_->elastic.initialOptimizableValues,
    parameterData_->plastic.initialOptimizableValues);
}

}  // namespace pgo::SolidDeformationModel
