#pragma once

#include "elementScalarField.h"

#include <memory>
#include <span>

namespace pgo::SolidDeformationModel
{

class ShellArealDensityField final
{
public:
  class Evaluator
  {
  public:
    double value(int element, int quadrature = 0) const;
    void localParameterDerivative(
      int element, int quadrature, std::span<double> output) const;

  private:
    friend class ShellArealDensityField;

    Evaluator(
      std::shared_ptr<const ElementScalarFieldSource> source,
      MaterialParameterEvaluationView state);

    std::shared_ptr<const ElementScalarFieldSource> source_;
    const MaterialParameterRef *dependency_ = nullptr;
    MaterialParameterEvaluationView state_;
    mutable MaterialParameterEvaluationScratch scratch_;
  };

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
    const MaterialParameterEvaluationView &state = {}) const;

  Evaluator evaluator(MaterialParameterEvaluationView state) const;

  const MaterialParameterRef *parameterDependency() const;
  void localParameterDerivative(
    int element,
    int quadrature,
    const MaterialParameterEvaluationView &state,
    std::span<double> output) const;

private:
  std::shared_ptr<const ElementScalarFieldSource> source_;
};

}  // namespace pgo::SolidDeformationModel
