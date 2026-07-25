#include "volumeDensityField.h"

#include <cmath>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{

VolumeDensityField::VolumeDensityField(
  std::shared_ptr<const ElementScalarFieldSource> source):
  source_(std::move(source))
{
  if (!source_)
    throw std::invalid_argument("VolumeDensityField requires a source");
  if (source_->parameterDependency())
    throw std::invalid_argument(
      "parameter-dependent volume density is not supported");
}

VolumeDensityField VolumeDensityField::constant(double density)
{
  return VolumeDensityField(
    std::make_shared<ConstantScalarFieldSource>(density));
}

VolumeDensityField VolumeDensityField::elementwise(EigenSupport::VXd densities)
{
  return VolumeDensityField(
    std::make_shared<ElementwiseScalarFieldSource>(std::move(densities)));
}

void VolumeDensityField::validate(int numElements) const
{
  source_->validate(numElements);
}

double VolumeDensityField::value(
  int element, int quadrature, MaterialParameterEvaluationView state) const
{
  const double density = source_->value(element, quadrature, state);
  if (!std::isfinite(density) || !(density > 0.0))
    throw std::invalid_argument(
      "VolumeDensityField source returned a non-finite or non-positive density");
  return density;
}

}  // namespace pgo::SolidDeformationModel
