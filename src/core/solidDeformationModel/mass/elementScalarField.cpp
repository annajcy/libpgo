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
  const MaterialParameterEvaluationView &state,
  MaterialParameterEvaluationScratch &) const
{
  return value(element, quadrature, state);
}

void ElementScalarFieldSource::localParameterDerivativeWithScratch(
  int element,
  int quadrature,
  const MaterialParameterEvaluationView &state,
  MaterialParameterEvaluationScratch &,
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
  int, int, const MaterialParameterEvaluationView &) const
{
  return value_;
}

const MaterialParameterRef *ConstantScalarFieldSource::parameterDependency() const
{
  return nullptr;
}

void ConstantScalarFieldSource::localParameterDerivative(
  int, int, const MaterialParameterEvaluationView &, EigenSupport::RefVecXd output) const
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
  int element, int, const MaterialParameterEvaluationView &) const
{
  assert(element >= 0 && element < values_.size());
  return values_[element];
}

const MaterialParameterRef *ElementwiseScalarFieldSource::parameterDependency() const
{
  return nullptr;
}

void ElementwiseScalarFieldSource::localParameterDerivative(
  int, int, const MaterialParameterEvaluationView &, EigenSupport::RefVecXd output) const
{
  output.setZero();
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
  int element,
  int quadrature,
  const MaterialParameterEvaluationView &state) const
{
  return scale_ * parameter_.value(element, quadrature, state);
}

double ScaledElasticParameterFieldSource::valueWithScratch(
  int element,
  int quadrature,
  const MaterialParameterEvaluationView &state,
  MaterialParameterEvaluationScratch &scratch) const
{
  return scale_ * parameter_.value(element, quadrature, state, scratch);
}

const MaterialParameterRef *
ScaledElasticParameterFieldSource::parameterDependency() const
{
  return &parameter_;
}

void ScaledElasticParameterFieldSource::localParameterDerivative(
  int element,
  int quadrature,
  const MaterialParameterEvaluationView &state,
  EigenSupport::RefVecXd output) const
{
  const auto expected = parameter_.field().dofLayout().numLocalDofs();
  if (output.size() != expected)
    throw std::invalid_argument(
      "scaled elastic parameter field derivative buffer has size " +
      std::to_string(output.size()) + ", expected " +
      std::to_string(expected));
  parameter_.localDerivative(element, quadrature, state, output);
  output *= scale_;
}

void ScaledElasticParameterFieldSource::localParameterDerivativeWithScratch(
  int element,
  int quadrature,
  const MaterialParameterEvaluationView &state,
  MaterialParameterEvaluationScratch &scratch,
  EigenSupport::RefVecXd output) const
{
  const auto expected = parameter_.field().dofLayout().numLocalDofs();
  if (output.size() != expected)
    throw std::invalid_argument(
      "scaled elastic parameter field derivative buffer has size " +
      std::to_string(output.size()) + ", expected " +
      std::to_string(expected));
  parameter_.localDerivative(element, quadrature, state, scratch, output);
  output *= scale_;
}

}  // namespace pgo::SolidDeformationModel
