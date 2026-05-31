#pragma once

#include "parameterField.h"

#include <cstring>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

class ConstantParameterField : public OptimizableField
{
public:
  ConstantParameterField(int numChannels, int numElements, const double *globalParams);

  ParameterFieldKind kind() const override { return ParameterFieldKind::CONSTANT; }
  int numChannels() const override { return numChannels_; }
  int numLocalDofs() const override { return numChannels_; }

  void computeValue(int ele, int quadratureId, double *out) const override;
  void setGlobalData(const double *data) override { globalParams_ = data; }

  const ParameterDofLayout *dofLayout() const override { return &dofLayout_; }
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

  int numChannels_ = 0;
  const double *globalParams_ = nullptr;
  ElementParameterDofLayout dofLayout_;
};

// ---- Implementation ----

inline ConstantParameterField::ConstantParameterField(
  int numChannels, int numElements, const double *globalParams):
  numChannels_(numChannels),
  globalParams_(globalParams),
  dofLayout_(numChannels, numElements)
{
}

inline void ConstantParameterField::computeValue(
  int ele, int /*quadratureId*/, double *out) const
{
  if (numChannels_ == 0) return;
  dofLayout_.gather(ele, globalParams_, out);
}

inline void ConstantParameterField::computeDerivative(
  int /*ele*/, int /*quadratureId*/, double *derivOut) const
{
  if (numChannels_ == 0) return;
  ES::Mp<ES::MXd>(derivOut, numChannels_, numChannels_).setIdentity();
}

inline ConstantParameterField::ElementParameterDofLayout::ElementParameterDofLayout(
  int numChannels, int numElements):
  numChannels_(numChannels), numElements_(numElements)
{
}

inline void ConstantParameterField::ElementParameterDofLayout::gather(
  int ele, const double *global, double *local) const
{
  if (numChannels_ == 0) return;
  const double *src = global + static_cast<std::ptrdiff_t>(ele) * numChannels_;
  std::memcpy(local, src, static_cast<std::size_t>(numChannels_) * sizeof(double));
}

}  // namespace SolidDeformationModel
}  // namespace pgo
