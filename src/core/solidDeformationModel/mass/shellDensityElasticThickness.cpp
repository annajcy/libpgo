#include "shellDensityElasticThickness.h"

#include "material/fields/materialParameters.h"
#include "simulation/simulationMesh.h"

#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

ShellDensityElasticThickness::ShellDensityElasticThickness(double density,
  std::shared_ptr<const MaterialParameterSpace> space,
  MaterialParameterRef thickness):
  density_(density),
  space_(std::move(space)),
  thickness_(thickness)
{
  if (!(density > 0.0)) {
    throw std::invalid_argument("ShellDensityElasticThickness requires density > 0");
  }
  if (!space_) {
    throw std::invalid_argument("ShellDensityElasticThickness requires a parameter space");
  }
  if (&thickness_.block() != &space_->elastic()) {
    throw std::invalid_argument(
      "ShellDensityElasticThickness parameter must belong to the elastic block of its space");
  }
  if (thickness_.name() != "thickness") {
    throw std::invalid_argument(
      "ShellDensityElasticThickness parameter must be named 'thickness'");
  }
}

void ShellDensityElasticThickness::validate(const SimulationMesh &mesh) const
{
  ShellMassField::validate(mesh);
  if (thickness_.block().dofLayout().numElements() != mesh.getNumElements()) {
    throw std::invalid_argument(
      "ShellDensityElasticThickness parameter field shape does not match mesh element count");
  }
}

double ShellDensityElasticThickness::arealDensity(
  int ele, MaterialStateView state) const
{
  return density_ * thickness_.value(ele, 0, state);
}

void ShellDensityElasticThickness::arealDensityParameterDerivative(
  int ele, MaterialStateView state, double *out) const
{
  thickness_.localDerivative(ele, 0, state, out);
  const int numLocal = thickness_.block().dofLayout().numLocalDofs();
  for (int k = 0; k < numLocal; k++)
    out[k] *= density_;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
