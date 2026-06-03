#pragma once

#include "parameterField.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

// A parameter field whose single set of `numChannels` parameters is shared by
// every element of the mesh. Its global dof count is `numChannels` (not
// numChannels * numElements): all elements gather the same values and, during
// optimization, map to the same global columns so their derivatives accumulate.
class ConstantParameterField : public OptimizableField
{
public:
  ConstantParameterField(int numChannels, int numElements, const double *globalParams);
  ConstantParameterField(ParameterFieldSpec spec, int numElements, ES::VXd values);

  const ParameterFieldSpec &spec() const override { return spec_; }
  ParameterFieldKind kind() const override { return ParameterFieldKind::CONSTANT; }
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
  class ConstantDofLayout : public OptimizableField::ParameterDofLayout
  {
  public:
    explicit ConstantDofLayout(int numChannels);
    int numLocalDofs() const override { return numChannels_; }
    int numGlobalDofs() const override { return numChannels_; }
    void gather(int ele, const double *global, double *local) const override;

  private:
    int numChannels_ = 0;
  };

  ParameterFieldSpec spec_;
  int numElements_ = 0;
  ES::VXd values_;
  ConstantDofLayout dofLayout_;
};

// ---- Implementation ----

inline ConstantParameterField::ConstantParameterField(
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

inline ConstantParameterField::ConstantParameterField(
  ParameterFieldSpec spec, int numElements, ES::VXd values):
  spec_(std::move(spec)),
  numElements_(numElements),
  values_(std::move(values)),
  dofLayout_(spec_.numChannels)
{
  if (values_.size() != spec_.numChannels)
    throw std::invalid_argument("ConstantParameterField: values size does not match numChannels.");
}

inline void ConstantParameterField::setValues(ES::VXd values)
{
  if (values.size() != values_.size())
    throw std::invalid_argument("ConstantParameterField::setValues: values size mismatch.");
  values_ = std::move(values);
}

inline void ConstantParameterField::setGlobalData(const double *data)
{
  if (values_.size() == 0)
    return;
  if (!data)
    throw std::invalid_argument("ConstantParameterField::setGlobalData: data must be non-null for non-empty field.");
  values_ = Eigen::Map<const ES::VXd>(data, values_.size());
}

inline void ConstantParameterField::computeValue(
  int /*ele*/, int /*quadratureId*/, double *out) const
{
  if (spec_.numChannels == 0)
    return;
  std::memcpy(out, values_.data(), static_cast<std::size_t>(spec_.numChannels) * sizeof(double));
}

inline void ConstantParameterField::computeDerivative(
  int /*ele*/, int /*quadratureId*/, double *derivOut) const
{
  if (spec_.numChannels == 0)
    return;
  ES::Mp<ES::MXd>(derivOut, spec_.numChannels, spec_.numChannels).setIdentity();
}

inline ConstantParameterField::ConstantDofLayout::ConstantDofLayout(int numChannels):
  numChannels_(numChannels)
{
}

inline void ConstantParameterField::ConstantDofLayout::gather(
  int /*ele*/, const double *global, double *local) const
{
  if (numChannels_ == 0)
    return;
  std::memcpy(local, global, static_cast<std::size_t>(numChannels_) * sizeof(double));
}

}  // namespace SolidDeformationModel
}  // namespace pgo
