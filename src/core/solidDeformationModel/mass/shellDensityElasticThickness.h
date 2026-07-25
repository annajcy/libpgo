#pragma once

#include "shellMassField.h"
#include "elasticParameterDependentMassField.h"
#include "material/core/materialParameters.h"

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
  ShellDensityElasticThickness(double density, MaterialParameterRef thickness);

  void validate(const SimulationMesh &mesh) const override;
  double arealDensity(int ele, MaterialParameterEvaluationView state) const override;

  const MaterialParameterRef &parameter() const override { return thickness_; }
  void arealDensityParameterDerivative(
    int ele, MaterialParameterEvaluationView state, double *out) const override;

private:
  double density_ = 0.0;
  MaterialParameterRef thickness_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
