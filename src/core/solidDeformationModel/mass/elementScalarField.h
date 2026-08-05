#pragma once

#include "material/runtime/optimizableParameterRef.h"

#include <memory>

namespace pgo::SolidDeformationModel
{

// A scalar field sampled on element/quadrature locations.  The interface is
// deliberately independent of volume/shell units; those semantics are added
// by VolumeDensityField and ShellArealDensityField.
class ElementScalarFieldSource
{
public:
  virtual ~ElementScalarFieldSource() = default;

  virtual void validate(int numElements) const = 0;

  virtual double value(
    int element,
    int quadrature,
    const MaterialStateView &state) const = 0;

  // Batch callers can provide reusable storage. The default implementation
  // preserves the behavior of custom sources that do not need it.
  virtual double valueWithScratch(
    int element,
    int quadrature,
    const MaterialStateView &state,
    MaterialStateEvaluationScratch &scratch) const;

  // At most one parameter field is supported by this refactor. A null pointer
  // denotes a source independent of material parameters. The returned pointer
  // remains valid for the lifetime of this source.
  virtual const OptimizableParameterRef *parameterDependency() const = 0;

  // The output vector has exactly
  // parameterDependency()->field().layout().numLocalParameters() entries when
  // the source is parameter-dependent, and is empty otherwise. The semantic
  // wrappers validate this contract before dispatching to the source.
  virtual void localParameterDerivative(
    int element,
    int quadrature,
    const MaterialStateView &state,
    EigenSupport::RefVecXd output) const = 0;

  virtual void localParameterDerivativeWithScratch(
    int element,
    int quadrature,
    const MaterialStateView &state,
    MaterialStateEvaluationScratch &scratch,
    EigenSupport::RefVecXd output) const;
};

class ConstantScalarFieldSource final : public ElementScalarFieldSource
{
public:
  explicit ConstantScalarFieldSource(double value);

  void validate(int numElements) const override;
  double value(
    int element,
    int quadrature,
    const MaterialStateView &state) const override;
  const OptimizableParameterRef *parameterDependency() const override;
  void localParameterDerivative(
    int element,
    int quadrature,
    const MaterialStateView &state,
    EigenSupport::RefVecXd output) const override;

private:
  double value_ = 0.0;
};

class ElementwiseScalarFieldSource final : public ElementScalarFieldSource
{
public:
  explicit ElementwiseScalarFieldSource(EigenSupport::VXd values);

  void validate(int numElements) const override;
  double value(
    int element,
    int quadrature,
    const MaterialStateView &state) const override;
  const OptimizableParameterRef *parameterDependency() const override;
  void localParameterDerivative(
    int element,
    int quadrature,
    const MaterialStateView &state,
    EigenSupport::RefVecXd output) const override;

private:
  EigenSupport::VXd values_;
};

// scale * parameter(element, quadrature). The referenced parameter is
// required to belong to the elastic field by the shell formulation.
class ScaledElasticParameterFieldSource final : public ElementScalarFieldSource
{
public:
  ScaledElasticParameterFieldSource(double scale, OptimizableParameterRef parameter);

  void validate(int numElements) const override;
  double value(
    int element,
    int quadrature,
    const MaterialStateView &state) const override;
  double valueWithScratch(
    int element,
    int quadrature,
    const MaterialStateView &state,
    MaterialStateEvaluationScratch &scratch) const override;
  const OptimizableParameterRef *parameterDependency() const override;
  void localParameterDerivative(
    int element,
    int quadrature,
    const MaterialStateView &state,
    EigenSupport::RefVecXd output) const override;
  void localParameterDerivativeWithScratch(
    int element,
    int quadrature,
    const MaterialStateView &state,
    MaterialStateEvaluationScratch &scratch,
    EigenSupport::RefVecXd output) const override;

private:
  double scale_ = 0.0;
  OptimizableParameterRef parameter_;
};

}  // namespace pgo::SolidDeformationModel
