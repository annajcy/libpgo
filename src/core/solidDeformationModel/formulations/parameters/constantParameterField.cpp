#include "constantParameterField.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace pgo
{
namespace SolidDeformationModel
{

ConstantParameterField::ConstantParameterField(
  int numChannels, int numElements, const double *globalParams):
  numElements_(numElements),
  values_(numChannels > 0 && globalParams
            ? Eigen::Map<const ES::VXd>(globalParams, numChannels)
            : ES::VXd(std::max(numChannels, 0))),
  dofLayout_(numChannels)
{
  spec_.numChannels = numChannels;
  if (numChannels > 0 && !globalParams)
    values_.setZero();
}

ConstantParameterField::ConstantParameterField(
  ParameterFieldSpec spec, int numElements, ES::VXd values):
  spec_(std::move(spec)),
  numElements_(numElements),
  values_(std::move(values)),
  dofLayout_(spec_.numChannels)
{
  if (values_.size() != spec_.numChannels)
    throw std::invalid_argument("ConstantParameterField: values size does not match numChannels.");
}

void ConstantParameterField::setValues(ES::VXd values)
{
  if (values.size() != values_.size())
    throw std::invalid_argument("ConstantParameterField::setValues: values size mismatch.");
  values_ = std::move(values);
}

void ConstantParameterField::setGlobalData(const double *data)
{
  if (values_.size() == 0)
    return;
  if (!data)
    throw std::invalid_argument("ConstantParameterField::setGlobalData: data must be non-null for non-empty field.");
  values_ = Eigen::Map<const ES::VXd>(data, values_.size());
}

void ConstantParameterField::computeValue(
  int /*ele*/, int /*quadratureId*/, double *out) const
{
  if (spec_.numChannels == 0)
    return;
  std::memcpy(out, values_.data(), static_cast<std::size_t>(spec_.numChannels) * sizeof(double));
}

void ConstantParameterField::computeDerivative(
  int /*ele*/, int /*quadratureId*/, double *derivOut) const
{
  if (spec_.numChannels == 0)
    return;
  ES::Mp<ES::MXd>(derivOut, spec_.numChannels, spec_.numChannels).setIdentity();
}

ConstantParameterField::ConstantDofLayout::ConstantDofLayout(int numChannels):
  numChannels_(numChannels)
{
}

bool ConstantParameterField::ConstantDofLayout::matchesParameterShape(
  int numChannels, int /*numElements*/) const
{
  return numChannels_ == numChannels;
}

int ConstantParameterField::ConstantDofLayout::globalDof(
  int /*ele*/, int localDof) const
{
  if (localDof < 0 || localDof >= numChannels_)
    throw std::out_of_range("ConstantParameterField::globalDof: local dof index out of range.");
  return localDof;
}

void ConstantParameterField::ConstantDofLayout::gather(
  int /*ele*/, const double *global, double *local) const
{
  if (numChannels_ == 0)
    return;
  std::memcpy(local, global, static_cast<std::size_t>(numChannels_) * sizeof(double));
}

}  // namespace SolidDeformationModel
}  // namespace pgo
