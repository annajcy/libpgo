#include "shellDensityElasticThickness.h"

#include "material/fields/parameterField.h"
#include "simulation/simulationMesh.h"

#include <stdexcept>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

ShellDensityElasticThickness::ShellDensityElasticThickness(double density,
  std::shared_ptr<const OptimizableField> field, int thicknessChannel)
  : density_(density), field_(std::move(field)), channel_(thicknessChannel)
{
  if (!(density > 0.0)) {
    throw std::invalid_argument("ShellDensityElasticThickness requires density > 0");
  }
  if (!field_) {
    throw std::invalid_argument("ShellDensityElasticThickness requires a parameter field");
  }
  if (channel_ < 0 || channel_ >= field_->numChannels()) {
    throw std::invalid_argument("ShellDensityElasticThickness thickness channel out of range");
  }
}

void ShellDensityElasticThickness::validate(const SimulationMesh &mesh) const
{
  ShellMassField::validate(mesh);
  if (!field_->dofLayout()->matchesParameterShape(field_->numChannels(), mesh.getNumElements())) {
    throw std::invalid_argument(
      "ShellDensityElasticThickness parameter field shape does not match mesh element count");
  }
}

double ShellDensityElasticThickness::arealDensity(int ele) const
{
  double value[64];  // far exceeds any current or foreseeable channel count
  assert(field_->numChannels() <= 64);
  field_->computeValue(ele, 0, value);
  return density_ * value[channel_];
}

void ShellDensityElasticThickness::arealDensityParameterDerivative(int ele, double *out) const
{
  const int numChannels = field_->numChannels();
  const int numLocal = field_->dofLayout()->numLocalDofs();
  std::vector<double> deriv(static_cast<size_t>(numChannels) * numLocal);
  field_->computeDerivative(ele, 0, deriv.data());
  // deriv is column-major numChannels x numLocal; take the thickness row.
  for (int k = 0; k < numLocal; k++)
    out[k] = density_ * deriv[static_cast<size_t>(k) * numChannels + channel_];
}

}  // namespace SolidDeformationModel
}  // namespace pgo
