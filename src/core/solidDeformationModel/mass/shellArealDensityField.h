#pragma once

#include "elementScalarField.h"

#include <memory>

namespace pgo::SolidDeformationModel
{

class ShellArealDensityField final
{
public:
  explicit ShellArealDensityField(
    std::shared_ptr<const ElementScalarFieldSource> source);

  static ShellArealDensityField constant(double arealDensity);
  static ShellArealDensityField elementwise(EigenSupport::VXd arealDensities);
  static ShellArealDensityField fromDensityThickness(
    double density, double thickness);
  static ShellArealDensityField fromDensityThickness(
    double density, EigenSupport::VXd thickness);
  static ShellArealDensityField fromElasticParameter(
    double scale, MaterialParameterRef parameter);

  void validate(int numElements) const;
  double value(
    int element,
    int quadrature = 0,
    MaterialParameterEvaluationView state = {}) const;

  std::optional<MaterialParameterRef> parameterDependency() const;
  void localParameterDerivative(
    int element,
    int quadrature,
    MaterialParameterEvaluationView state,
    std::span<double> output) const;

private:
  std::shared_ptr<const ElementScalarFieldSource> source_;
};

}  // namespace pgo::SolidDeformationModel
