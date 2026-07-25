#include "elementScalarField.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <string>

namespace pgo::SolidDeformationModel
{

ConstantScalarFieldSource::ConstantScalarFieldSource(double value): value_(value)
{
  if (!std::isfinite(value) || !(value > 0.0))
    throw std::invalid_argument(
      "constant scalar field requires a finite value > 0");
}

void ConstantScalarFieldSource::validate(int) const
{
}

double ConstantScalarFieldSource::value(int, int, MaterialParameterEvaluationView) const
{
  return value_;
}

std::optional<MaterialParameterRef> ConstantScalarFieldSource::parameterDependency() const
{
  return std::nullopt;
}

void ConstantScalarFieldSource::localParameterDerivative(
  int, int, MaterialParameterEvaluationView, std::span<double> output) const
{
  std::fill(output.begin(), output.end(), 0.0);
}

ElementwiseScalarFieldSource::ElementwiseScalarFieldSource(EigenSupport::VXd values):
  values_(std::move(values))
{
  if (values_.size() == 0 || !values_.allFinite() ||
      (values_.array() <= 0.0).any())
    throw std::invalid_argument(
      "elementwise scalar field requires finite positive values");
}

void ElementwiseScalarFieldSource::validate(int numElements) const
{
  if (values_.size() != numElements)
    throw std::invalid_argument("elementwise scalar field size does not match element count");
}

double ElementwiseScalarFieldSource::value(
  int element, int, MaterialParameterEvaluationView) const
{
  assert(element >= 0 && element < values_.size());
  return values_[element];
}

std::optional<MaterialParameterRef> ElementwiseScalarFieldSource::parameterDependency() const
{
  return std::nullopt;
}

void ElementwiseScalarFieldSource::localParameterDerivative(
  int, int, MaterialParameterEvaluationView, std::span<double> output) const
{
  std::fill(output.begin(), output.end(), 0.0);
}

ScaledElasticParameterFieldSource::ScaledElasticParameterFieldSource(
  double scale, MaterialParameterRef parameter):
  scale_(scale),
  parameter_(std::move(parameter))
{
  if (!std::isfinite(scale) || !(scale > 0.0))
    throw std::invalid_argument(
      "scaled elastic parameter field requires a finite scale > 0");
}

void ScaledElasticParameterFieldSource::validate(int numElements) const
{
  if (parameter_.field().dofLayout().numElements() != numElements)
    throw std::invalid_argument(
      "scaled elastic parameter field shape does not match element count");
}

double ScaledElasticParameterFieldSource::value(
  int element, int quadrature, MaterialParameterEvaluationView state) const
{
  return scale_ * parameter_.value(element, quadrature, state);
}

std::optional<MaterialParameterRef>
ScaledElasticParameterFieldSource::parameterDependency() const
{
  return parameter_;
}

void ScaledElasticParameterFieldSource::localParameterDerivative(
  int element,
  int quadrature,
  MaterialParameterEvaluationView state,
  std::span<double> output) const
{
  const auto expected = static_cast<std::size_t>(
    parameter_.field().dofLayout().numLocalDofs());
  if (output.size() != expected)
    throw std::invalid_argument(
      "scaled elastic parameter field derivative buffer has size " +
      std::to_string(output.size()) + ", expected " +
      std::to_string(expected));
  parameter_.localDerivative(element, quadrature, state, output.data());
  for (double &value : output)
    value *= scale_;
}

}  // namespace pgo::SolidDeformationModel
