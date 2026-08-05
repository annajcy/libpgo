#include "elementScalarField.h"

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <string>

namespace pgo::SolidDeformationModel
{

double ElementScalarFieldSource::valueWithScratch(
  int element,
  int quadrature,
  const MaterialStateView &state,
  MaterialStateEvaluationScratch &) const
{
  return value(element, quadrature, state);
}

void ElementScalarFieldSource::localParameterDerivativeWithScratch(
  int element,
  int quadrature,
  const MaterialStateView &state,
  MaterialStateEvaluationScratch &,
  EigenSupport::RefVecXd output) const
{
  localParameterDerivative(element, quadrature, state, output);
}

ConstantScalarFieldSource::ConstantScalarFieldSource(double value): value_(value)
{
  if (!std::isfinite(value) || !(value > 0.0))
    throw std::invalid_argument(
      "constant scalar field requires a finite value > 0");
}

void ConstantScalarFieldSource::validate(int) const
{
}

double ConstantScalarFieldSource::value(
  int, int, const MaterialStateView &) const
{
  return value_;
}

const OptimizableParameterRef *ConstantScalarFieldSource::parameterDependency() const
{
  return nullptr;
}

void ConstantScalarFieldSource::localParameterDerivative(
  int, int, const MaterialStateView &, EigenSupport::RefVecXd output) const
{
  output.setZero();
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
  int element, int, const MaterialStateView &) const
{
  assert(element >= 0 && element < values_.size());
  return values_[element];
}

const OptimizableParameterRef *ElementwiseScalarFieldSource::parameterDependency() const
{
  return nullptr;
}

void ElementwiseScalarFieldSource::localParameterDerivative(
  int, int, const MaterialStateView &, EigenSupport::RefVecXd output) const
{
  output.setZero();
}

ScaledElasticParameterFieldSource::ScaledElasticParameterFieldSource(
  double scale, OptimizableParameterRef parameter):
  scale_(scale),
  parameter_(std::move(parameter))
{
  if (!std::isfinite(scale) || !(scale > 0.0))
    throw std::invalid_argument(
      "scaled elastic parameter field requires a finite scale > 0");
}

void ScaledElasticParameterFieldSource::validate(int numElements) const
{
  if (parameter_.field().layout().numElements() != numElements)
    throw std::invalid_argument(
      "scaled elastic parameter field shape does not match element count");
}

double ScaledElasticParameterFieldSource::value(
  int element,
  int quadrature,
  const MaterialStateView &state) const
{
  return scale_ * parameter_.value(
    element, quadrature, state.elasticValues());
}

double ScaledElasticParameterFieldSource::valueWithScratch(
  int element,
  int quadrature,
  const MaterialStateView &state,
  MaterialStateEvaluationScratch &scratch) const
{
  return scale_ * parameter_.value(
    element, quadrature, state.elasticValues(), scratch);
}

const OptimizableParameterRef *
ScaledElasticParameterFieldSource::parameterDependency() const
{
  return &parameter_;
}

void ScaledElasticParameterFieldSource::localParameterDerivative(
  int element,
  int quadrature,
  const MaterialStateView &state,
  EigenSupport::RefVecXd output) const
{
  const auto expected = parameter_.field().layout().numLocalParameters();
  if (output.size() != expected)
    throw std::invalid_argument(
      "scaled elastic parameter field derivative buffer has size " +
      std::to_string(output.size()) + ", expected " +
      std::to_string(expected));
  parameter_.localDerivative(
    element, quadrature, state.elasticValues(), output);
  output *= scale_;
}

void ScaledElasticParameterFieldSource::localParameterDerivativeWithScratch(
  int element,
  int quadrature,
  const MaterialStateView &state,
  MaterialStateEvaluationScratch &scratch,
  EigenSupport::RefVecXd output) const
{
  const auto expected = parameter_.field().layout().numLocalParameters();
  if (output.size() != expected)
    throw std::invalid_argument(
      "scaled elastic parameter field derivative buffer has size " +
      std::to_string(output.size()) + ", expected " +
      std::to_string(expected));
  parameter_.localDerivative(
    element, quadrature, state.elasticValues(), scratch, output);
  output *= scale_;
}

}  // namespace pgo::SolidDeformationModel
