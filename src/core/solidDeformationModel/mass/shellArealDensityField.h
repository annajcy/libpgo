#pragma once

#include "elementScalarField.h"

#include <memory>

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
      int element, int quadrature, EigenSupport::RefVecXd output) const;

  private:
    friend class ShellArealDensityField;

    Evaluator(
      std::shared_ptr<const ElementScalarFieldSource> source,
      OptimizableParameterEvaluationView state);

    std::shared_ptr<const ElementScalarFieldSource> source_;
    const OptimizableParameterRef *dependency_ = nullptr;
    OptimizableParameterEvaluationView state_;
    mutable OptimizableParameterEvaluationScratch scratch_;
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
    double scale, OptimizableParameterRef parameter);

  void validate(int numElements) const;
  double value(
    int element,
    int quadrature = 0,
    const OptimizableParameterEvaluationView &state = {}) const;

  Evaluator evaluator(OptimizableParameterEvaluationView state) const;

  const OptimizableParameterRef *parameterDependency() const;
  void localParameterDerivative(
    int element,
    int quadrature,
    const OptimizableParameterEvaluationView &state,
    EigenSupport::RefVecXd output) const;

private:
  std::shared_ptr<const ElementScalarFieldSource> source_;
};

}  // namespace pgo::SolidDeformationModel
