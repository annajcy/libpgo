#include "shellArealDensityField.h"

#include <cmath>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{

namespace
{

void validateEvaluationState(
  const std::optional<MaterialParameterRef> &dependency,
  MaterialParameterEvaluationView state)
{
  if (!dependency)
    return;
  if (state.empty())
    throw std::invalid_argument(
      "parameter-dependent areal density requires material parameter state");
  if (&dependency->field() != &state.space().elastic())
    throw std::invalid_argument(
      "parameter-dependent areal density must depend on the evaluation space elastic field");
}

}  // namespace

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

ShellArealDensityField ShellArealDensityField::fromElasticParameter(
  double scale, MaterialParameterRef parameter)
{
  return ShellArealDensityField(
    std::make_shared<ScaledElasticParameterFieldSource>(
      scale, std::move(parameter)));
}

void ShellArealDensityField::validate(int numElements) const
{
  source_->validate(numElements);
}

double ShellArealDensityField::value(
  int element, int quadrature, MaterialParameterEvaluationView state) const
{
  const auto dependency = source_->parameterDependency();
  validateEvaluationState(dependency, state);
  const double arealDensity = source_->value(element, quadrature, state);
  if (!std::isfinite(arealDensity) || !(arealDensity > 0.0))
    throw std::invalid_argument(
      "ShellArealDensityField source returned a non-finite or non-positive density");
  return arealDensity;
}

std::optional<MaterialParameterRef>
ShellArealDensityField::parameterDependency() const
{
  return source_->parameterDependency();
}

void ShellArealDensityField::localParameterDerivative(
  int element,
  int quadrature,
  MaterialParameterEvaluationView state,
  std::span<double> output) const
{
  const auto dependency = source_->parameterDependency();
  validateEvaluationState(dependency, state);
  source_->localParameterDerivative(element, quadrature, state, output);
}

}  // namespace pgo::SolidDeformationModel
