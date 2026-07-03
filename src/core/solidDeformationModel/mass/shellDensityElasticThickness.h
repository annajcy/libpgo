#pragma once

#include "shellMassField.h"
#include "elasticParameterDependentMassField.h"

#include <memory>

namespace pgo
{
namespace SolidDeformationModel
{

class OptimizableField;

// rho * h with h read live from a channel of an elastic OptimizableField
// (e.g. the Koiter elastic field's thickness channel 4). No duplicated
// thickness storage: after the optimizer calls setGlobalData on the field,
// this mass field sees the new values.
class ShellDensityElasticThickness :
  public ShellMassField,
  public ElasticParameterDependentMassField
{
public:
  ShellDensityElasticThickness(double density,
    std::shared_ptr<const OptimizableField> field, int thicknessChannel);

  void validate(const SimulationMesh &mesh) const override;
  double arealDensity(int ele) const override;

  const OptimizableField &parameterField() const override { return *field_; }
  void arealDensityParameterDerivative(int ele, double *out) const override;

private:
  double density_ = 0.0;
  std::shared_ptr<const OptimizableField> field_;
  int channel_ = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
