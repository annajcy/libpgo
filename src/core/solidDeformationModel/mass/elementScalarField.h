#pragma once

#include "material/core/materialParameters.h"

#include <memory>
#include <optional>
#include <span>

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
    MaterialParameterEvaluationView state) const = 0;

  // At most one parameter field is supported by this refactor. An empty
  // result denotes a source independent of material parameters.
  virtual std::optional<MaterialParameterRef> parameterDependency() const = 0;

  // The output span has parameterDependency()->field().dofLayout().numLocalDofs()
  // entries when the source is parameter-dependent. Independent sources fill
  // it with zeroes.
  virtual void localParameterDerivative(
    int element,
    int quadrature,
    MaterialParameterEvaluationView state,
    std::span<double> output) const = 0;
};

class ConstantScalarFieldSource final : public ElementScalarFieldSource
{
public:
  explicit ConstantScalarFieldSource(double value);

  void validate(int numElements) const override;
  double value(
    int element,
    int quadrature,
    MaterialParameterEvaluationView state) const override;
  std::optional<MaterialParameterRef> parameterDependency() const override;
  void localParameterDerivative(
    int element,
    int quadrature,
    MaterialParameterEvaluationView state,
    std::span<double> output) const override;

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
    MaterialParameterEvaluationView state) const override;
  std::optional<MaterialParameterRef> parameterDependency() const override;
  void localParameterDerivative(
    int element,
    int quadrature,
    MaterialParameterEvaluationView state,
    std::span<double> output) const override;

private:
  EigenSupport::VXd values_;
};

// scale * parameter(element, quadrature). The referenced parameter is
// required to belong to the elastic field by the shell formulation.
class ScaledElasticParameterFieldSource final : public ElementScalarFieldSource
{
public:
  ScaledElasticParameterFieldSource(double scale, MaterialParameterRef parameter);

  void validate(int numElements) const override;
  double value(
    int element,
    int quadrature,
    MaterialParameterEvaluationView state) const override;
  std::optional<MaterialParameterRef> parameterDependency() const override;
  void localParameterDerivative(
    int element,
    int quadrature,
    MaterialParameterEvaluationView state,
    std::span<double> output) const override;

private:
  double scale_ = 0.0;
  MaterialParameterRef parameter_;
};

}  // namespace pgo::SolidDeformationModel
