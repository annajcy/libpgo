#pragma once

namespace pgo
{
namespace SolidDeformationModel
{

class OptimizableField;

// Capability interface for mass fields whose density depends on optimizable
// elastic parameters. Implementations multiply-inherit their domain MassField
// and this interface; callers probe with dynamic_cast.
class ElasticParameterDependentMassField
{
public:
  virtual ~ElasticParameterDependentMassField() = default;

  virtual const OptimizableField &parameterField() const = 0;

  // d(arealDensity(ele)) / d(local parameter dofs); out has length
  // parameterField().dofLayout()->numLocalDofs().
  virtual void arealDensityParameterDerivative(int ele, double *out) const = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
