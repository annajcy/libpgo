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
      MaterialStateView state);

    std::shared_ptr<const ElementScalarFieldSource> source_;
    const OptimizableParameterRef *dependency_ = nullptr;
    MaterialStateView state_;
    mutable MaterialStateEvaluationScratch scratch_;
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
    const MaterialStateView &state = {}) const;

  Evaluator evaluator(MaterialStateView state) const;

  const OptimizableParameterRef *parameterDependency() const;
  void localParameterDerivative(
    int element,
    int quadrature,
    const MaterialStateView &state,
    EigenSupport::RefVecXd output) const;

private:
  std::shared_ptr<const ElementScalarFieldSource> source_;
};

}  // namespace pgo::SolidDeformationModel
