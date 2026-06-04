#pragma once

#include "parameterField.h"

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
  int numValueRows() const override { return spec_.numChannels == 0 ? 0 : numElements_; }
  void computeDerivative(int ele, int quadratureId, double *derivOut) const override;

private:
  class ElementParameterDofLayout : public OptimizableField::ParameterDofLayout
  {
  public:
    ElementParameterDofLayout(int numChannels, int numElements);
    int numLocalDofs() const override { return numChannels_; }
    int numGlobalDofs() const override { return numChannels_ * numElements_; }
    bool matchesParameterShape(int numChannels, int numElements) const override;
    int globalDof(int ele, int localDof) const override;
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

}  // namespace SolidDeformationModel
}  // namespace pgo
