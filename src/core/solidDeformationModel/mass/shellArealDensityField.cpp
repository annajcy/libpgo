#include "shellArealDensityField.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace pgo::SolidDeformationModel
{

ShellArealDensityField::ShellArealDensityField(
  std::shared_ptr<const ElementScalarFieldSource> source):
  source_(std::move(source))
{
  if (!source_)
    throw std::invalid_argument("ShellArealDensityField requires a source");
}

ShellArealDensityField ShellArealDensityField::constant(double arealDensity)
{
  return ShellArealDensityField(
    std::make_shared<ConstantScalarFieldSource>(arealDensity));
}

ShellArealDensityField ShellArealDensityField::elementwise(
  EigenSupport::VXd arealDensities)
{
  return ShellArealDensityField(
    std::make_shared<ElementwiseScalarFieldSource>(std::move(arealDensities)));
}

ShellArealDensityField ShellArealDensityField::fromDensityThickness(
  double density, double thickness)
{
  if (!std::isfinite(density) || !std::isfinite(thickness) ||
      !(density > 0.0) || !(thickness > 0.0))
    throw std::invalid_argument(
      "ShellArealDensityField requires finite positive density and thickness");
  return constant(density * thickness);
}

ShellArealDensityField ShellArealDensityField::fromDensityThickness(
  double density, EigenSupport::VXd thickness)
{
  if (!std::isfinite(density) || !(density > 0.0) ||
      thickness.size() == 0 || !thickness.allFinite() ||
      (thickness.array() <= 0.0).any())
    throw std::invalid_argument(
      "ShellArealDensityField requires finite positive density and thicknesses");
  return elementwise(density * thickness.array());
}

void ShellArealDensityField::validate(int numElements) const
{
  source_->validate(numElements);
}

double ShellArealDensityField::value(int element, int quadrature) const
{
  const double arealDensity = source_->value(element, quadrature);
  if (!std::isfinite(arealDensity) || !(arealDensity > 0.0))
    throw std::invalid_argument(
      "ShellArealDensityField source returned a non-finite or non-positive density");
  return arealDensity;
}

}  // namespace pgo::SolidDeformationModel
