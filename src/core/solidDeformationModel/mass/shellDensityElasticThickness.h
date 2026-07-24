#pragma once

#include "shellMassField.h"
#include "elasticParameterDependentMassField.h"
#include "material/fields/materialParameters.h"

#include <memory>

namespace pgo
{
namespace SolidDeformationModel
{

// rho * h with h read from a semantic elastic material parameter reference.
class ShellDensityElasticThickness :
  public ShellMassField,
  public ElasticParameterDependentMassField
{
public:
  ShellDensityElasticThickness(double density,
    std::shared_ptr<const MaterialParameterSpace> space,
    MaterialParameterRef thickness);

  void validate(const SimulationMesh &mesh) const override;
  double arealDensity(int ele, MaterialStateView state) const override;

  const MaterialParameterRef &parameter() const override { return thickness_; }
  void arealDensityParameterDerivative(
    int ele, MaterialStateView state, double *out) const override;

private:
  double density_ = 0.0;
  std::shared_ptr<const MaterialParameterSpace> space_;
  MaterialParameterRef thickness_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
