#include "elementScalarField.h"

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <utility>

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

double ConstantScalarFieldSource::value(int, int) const
{
  return value_;
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
    throw std::invalid_argument(
      "elementwise scalar field size does not match element count");
}

double ElementwiseScalarFieldSource::value(int element, int) const
{
  assert(element >= 0 && element < values_.size());
  return values_[element];
}

}  // namespace pgo::SolidDeformationModel
