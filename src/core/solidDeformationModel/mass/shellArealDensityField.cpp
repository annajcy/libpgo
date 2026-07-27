#include "shellArealDensityField.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace pgo::SolidDeformationModel
{

namespace
{

void validateEvaluationState(
  const OptimizableParameterRef *dependency,
  const OptimizableParameterEvaluationView &state)
{
  if (dependency == nullptr)
    return;
  if (state.empty())
    throw std::invalid_argument(
      "parameter-dependent areal density requires optimizable parameter state");
  if (!dependency->field().sharesStateWith(state.elasticField()))
    throw std::invalid_argument(
      "parameter-dependent areal density must depend on the evaluation elastic field");
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
  double scale, OptimizableParameterRef parameter)
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
  int element,
  int quadrature,
  const OptimizableParameterEvaluationView &state) const
{
  auto evaluation = evaluator(state);
  return evaluation.value(element, quadrature);
}

ShellArealDensityField::Evaluator
ShellArealDensityField::evaluator(OptimizableParameterEvaluationView state) const
{
  return Evaluator(source_, std::move(state));
}

const OptimizableParameterRef *ShellArealDensityField::parameterDependency() const
{
  return source_->parameterDependency();
}

void ShellArealDensityField::localParameterDerivative(
  int element,
  int quadrature,
  const OptimizableParameterEvaluationView &state,
  EigenSupport::RefVecXd output) const
{
  auto evaluation = evaluator(state);
  evaluation.localParameterDerivative(element, quadrature, output);
}

ShellArealDensityField::Evaluator::Evaluator(
  std::shared_ptr<const ElementScalarFieldSource> source,
  OptimizableParameterEvaluationView state):
  source_(std::move(source)),
  state_(std::move(state))
{
  dependency_ = source_->parameterDependency();
  validateEvaluationState(dependency_, state_);
  if (dependency_ != nullptr)
    scratch_.prepare(dependency_->field());
}

double ShellArealDensityField::Evaluator::value(
  int element, int quadrature) const
{
  const double arealDensity = source_->valueWithScratch(
    element, quadrature, state_, scratch_);
  if (!std::isfinite(arealDensity) || !(arealDensity > 0.0))
    throw std::invalid_argument(
      "ShellArealDensityField source returned a non-finite or non-positive density");
  return arealDensity;
}

void ShellArealDensityField::Evaluator::localParameterDerivative(
  int element,
  int quadrature,
  EigenSupport::RefVecXd output) const
{
  const auto expected = dependency_
    ? dependency_->field().layout().numLocalParameters()
    : 0;
  if (output.size() != expected)
    throw std::invalid_argument(
      "areal density derivative buffer has size " +
      std::to_string(output.size()) + ", expected " +
      std::to_string(expected));
  source_->localParameterDerivativeWithScratch(
    element, quadrature, state_, scratch_, output);
}

}  // namespace pgo::SolidDeformationModel
