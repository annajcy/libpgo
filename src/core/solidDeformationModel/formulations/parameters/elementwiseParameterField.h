#pragma once

#include "parameterField.h"

#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

class ElementwiseParameterField : public OptimizableField
{
public:
  ElementwiseParameterField(ParameterFieldSpec spec, int numElements, ES::VXd values);

  const ParameterFieldSpec &spec() const override { return spec_; }
  ParameterFieldKind kind() const override { return ParameterFieldKind::ELEMENTWISE; }
  int numChannels() const override { return spec_.numChannels; }
  int numLocalDofs() const override { return spec_.numChannels; }
  int numElements() const { return numElements_; }

  const ES::VXd &values() const { return values_; }
  void setValues(ES::VXd values);

  void computeValue(int ele, int quadratureId, double *out) const override;
  void setGlobalData(const double *data) override;

  const ParameterDofLayout *dofLayout() const override { return &dofLayout_; }
  const double *globalData() const override { return values_.data(); }
  void computeDerivative(int ele, int quadratureId, double *derivOut) const override;

private:
  class ElementParameterDofLayout : public OptimizableField::ParameterDofLayout
  {
  public:
    ElementParameterDofLayout(int numChannels, int numElements);
    int numLocalDofs() const override { return numChannels_; }
    int numGlobalDofs() const override { return numChannels_ * numElements_; }
    void gather(int ele, const double *global, double *local) const override;

  private:
    int numChannels_ = 0;
    int numElements_ = 0;
  };

  ParameterFieldSpec spec_;
  int numElements_ = 0;
  ES::VXd values_;
  ElementParameterDofLayout dofLayout_;
};

// ---- Implementation ----

inline ElementwiseParameterField::ElementwiseParameterField(
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

inline void ElementwiseParameterField::setValues(ES::VXd values)
{
  if (values.size() != values_.size())
    throw std::invalid_argument("ElementwiseParameterField::setValues: values size mismatch.");
  values_ = std::move(values);
}

inline void ElementwiseParameterField::setGlobalData(const double *data)
{
  if (values_.size() == 0)
    return;
  if (!data)
    throw std::invalid_argument("ElementwiseParameterField::setGlobalData: data must be non-null for non-empty field.");
  values_ = Eigen::Map<const ES::VXd>(data, values_.size());
}

inline void ElementwiseParameterField::computeValue(
  int ele, int /*quadratureId*/, double *out) const
{
  if (spec_.numChannels == 0)
    return;
  dofLayout_.gather(ele, values_.data(), out);
}

inline void ElementwiseParameterField::computeDerivative(
  int /*ele*/, int /*quadratureId*/, double *derivOut) const
{
  if (spec_.numChannels == 0)
    return;
  ES::Mp<ES::MXd>(derivOut, spec_.numChannels, spec_.numChannels).setIdentity();
}

inline ElementwiseParameterField::ElementParameterDofLayout::ElementParameterDofLayout(
  int numChannels, int numElements):
  numChannels_(numChannels), numElements_(numElements)
{
}

inline void ElementwiseParameterField::ElementParameterDofLayout::gather(
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
