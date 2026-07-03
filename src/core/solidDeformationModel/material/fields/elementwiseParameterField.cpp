#include "elementwiseParameterField.h"

#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace pgo
{
namespace SolidDeformationModel
{

ElementwiseParameterField::ElementwiseParameterField(
  ParameterFieldSpec spec, int numElements, ES::VXd values):
  spec_(std::move(spec)),
  numElements_(numElements),
  values_(std::move(values)),
  dofLayout_(spec_.numChannels, numElements)
{
  if (numElements_ < 0)
    throw std::invalid_argument("ElementwiseParameterField: numElements must be non-negative.");
  const auto expected = static_cast<Eigen::Index>(numElements_) * spec_.numChannels;
  if (values_.size() != expected)
    throw std::invalid_argument("ElementwiseParameterField: values size does not match numElements * numChannels.");
}

void ElementwiseParameterField::setValues(ES::VXd values)
{
  if (values.size() != values_.size())
    throw std::invalid_argument("ElementwiseParameterField::setValues: values size mismatch.");
  values_ = std::move(values);
}

void ElementwiseParameterField::setGlobalData(const double *data)
{
  if (values_.size() == 0)
    return;
  if (!data)
    throw std::invalid_argument("ElementwiseParameterField::setGlobalData: data must be non-null for non-empty field.");
  values_ = Eigen::Map<const ES::VXd>(data, values_.size());
}

void ElementwiseParameterField::computeValue(
  int ele, int /*quadratureId*/, double *out) const
{
  if (spec_.numChannels == 0)
    return;
  dofLayout_.gather(ele, values_.data(), out);
}

void ElementwiseParameterField::computeDerivative(
  int /*ele*/, int /*quadratureId*/, double *derivOut) const
{
  if (spec_.numChannels == 0)
    return;
  ES::Mp<ES::MXd>(derivOut, spec_.numChannels, spec_.numChannels).setIdentity();
}

ElementwiseParameterField::ElementParameterDofLayout::ElementParameterDofLayout(
  int numChannels, int numElements):
  numChannels_(numChannels), numElements_(numElements)
{
}

bool ElementwiseParameterField::ElementParameterDofLayout::matchesParameterShape(
  int numChannels, int numElements) const
{
  return numChannels_ == numChannels && numElements_ == numElements;
}

int ElementwiseParameterField::ElementParameterDofLayout::globalDof(
  int ele, int localDof) const
{
  if (ele < 0 || ele >= numElements_)
    throw std::out_of_range("ElementwiseParameterField::globalDof: element index out of range.");
  if (localDof < 0 || localDof >= numChannels_)
    throw std::out_of_range("ElementwiseParameterField::globalDof: local dof index out of range.");
  return ele * numChannels_ + localDof;
}

void ElementwiseParameterField::ElementParameterDofLayout::gather(
  int ele, const double *global, double *local) const
{
  if (numChannels_ == 0)
    return;
  if (ele < 0 || ele >= numElements_)
    throw std::out_of_range("ElementwiseParameterField::gather: element index out of range.");
  const double *src = global + static_cast<std::ptrdiff_t>(ele) * numChannels_;
  std::memcpy(local, src, static_cast<std::size_t>(numChannels_) * sizeof(double));
}

}  // namespace SolidDeformationModel
}  // namespace pgo
