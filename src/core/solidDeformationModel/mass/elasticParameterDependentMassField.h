#pragma once

namespace pgo
{
namespace SolidDeformationModel
{

class MaterialParameterRef;
class MaterialParameterEvaluationView;

// Capability interface for mass fields whose density depends on optimizable
// elastic parameters. Implementations multiply-inherit their domain MassField
// and this interface; callers probe with dynamic_cast.
class ElasticParameterDependentMassField
{
public:
  virtual ~ElasticParameterDependentMassField() = default;

  virtual const MaterialParameterRef &parameter() const = 0;

  // d(arealDensity(ele)) / d(local parameter dofs); out has length
  // parameter().field().dofLayout().numLocalDofs().
  virtual void arealDensityParameterDerivative(
    int ele, MaterialParameterEvaluationView state, double *out) const = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
