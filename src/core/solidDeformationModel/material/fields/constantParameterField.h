#pragma once

#include "parameterField.h"

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
  int numValueRows() const override { return spec_.numChannels == 0 ? 0 : 1; }
  void computeDerivative(int ele, int quadratureId, double *derivOut) const override;

private:
  class ConstantDofLayout : public OptimizableField::ParameterDofLayout
  {
  public:
    explicit ConstantDofLayout(int numChannels);
    int numLocalDofs() const override { return numChannels_; }
    int numGlobalDofs() const override { return numChannels_; }
    bool matchesParameterShape(int numChannels, int numElements) const override;
    int globalDof(int ele, int localDof) const override;
    void gather(int ele, const double *global, double *local) const override;

  private:
    int numChannels_ = 0;
  };

  ParameterFieldSpec spec_;
  int numElements_ = 0;
  ES::VXd values_;
  ConstantDofLayout dofLayout_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
