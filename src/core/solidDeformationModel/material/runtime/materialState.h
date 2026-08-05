#pragma once

#include "material/parameterization/materialParameterField.h"
#include "EigenSupport.h"

#include <memory>
#include <span>
#include <vector>

namespace pgo::SolidDeformationModel
{

/// Reusable storage for sampling one material parameter field.
struct MaterialStateEvaluationScratch
{
  std::vector<double> local;
  std::vector<double> material;
  EigenSupport::MXd jacobian;

  void prepare(
    const OptimizableParameterField &field,
    int numMaterialLocations = 1);
};

class MaterialStateView;

/// Immutable material values associated with one pair of parameter fields.
class MaterialState
{
public:
  MaterialState() = default;
  MaterialState(
    std::shared_ptr<const OptimizableParameterField> elasticField,
    std::shared_ptr<const OptimizableParameterField> plasticField,
    EigenSupport::VXd elasticValues,
    EigenSupport::VXd plasticValues);

  bool empty() const { return !elasticField_ || !plasticField_; }
  const OptimizableParameterField &elasticField() const
  {
    return *elasticField_;
  }
  const OptimizableParameterField &plasticField() const
  {
    return *plasticField_;
  }
  const std::shared_ptr<const OptimizableParameterField> &elasticFieldHandle() const
  {
    return elasticField_;
  }
  const std::shared_ptr<const OptimizableParameterField> &plasticFieldHandle() const
  {
    return plasticField_;
  }
  const EigenSupport::VXd &elasticValues() const { return *elasticValues_; }
  const EigenSupport::VXd &plasticValues() const { return *plasticValues_; }

  MaterialStateView view() const;
  operator MaterialStateView() const;
  MaterialState withElasticValues(
    std::span<const double> elasticValues) const;
  MaterialState withPlasticValues(
    std::span<const double> plasticValues) const;
  MaterialState withValues(
    std::span<const double> elasticValues,
    std::span<const double> plasticValues) const;

private:
  MaterialState(
    std::shared_ptr<const OptimizableParameterField> elasticField,
    std::shared_ptr<const OptimizableParameterField> plasticField,
    std::shared_ptr<const EigenSupport::VXd> elasticValues,
    std::shared_ptr<const EigenSupport::VXd> plasticValues);

  std::shared_ptr<const OptimizableParameterField> elasticField_;
  std::shared_ptr<const OptimizableParameterField> plasticField_;
  std::shared_ptr<const EigenSupport::VXd> elasticValues_;
  std::shared_ptr<const EigenSupport::VXd> plasticValues_;
};

/// Non-owning view used during one material evaluation.
class MaterialStateView
{
public:
  MaterialStateView() = default;

  bool empty() const { return !elasticField_ || !plasticField_; }
  const OptimizableParameterField &elasticField() const;
  const OptimizableParameterField &plasticField() const;

  std::span<const double> elasticValues() const { return elasticValues_; }
  std::span<const double> plasticValues() const { return plasticValues_; }
  std::span<const double> values(
    const OptimizableParameterField &field) const;

  void evaluateElement(
    const OptimizableParameterField &field,
    int element,
    int numMaterialLocations,
    std::span<double> localParameterScratch,
    std::span<double> materialValues) const;
  std::span<const double> evaluateElement(
    const OptimizableParameterField &field,
    int element,
    int numMaterialLocations,
    MaterialStateEvaluationScratch &scratch) const;

private:
  friend class MaterialState;

  MaterialStateView(
    std::shared_ptr<const OptimizableParameterField> elasticField,
    std::shared_ptr<const OptimizableParameterField> plasticField,
    std::span<const double> elasticValues,
    std::span<const double> plasticValues,
    std::shared_ptr<const EigenSupport::VXd> elasticOwner = {},
    std::shared_ptr<const EigenSupport::VXd> plasticOwner = {}):
    elasticField_(std::move(elasticField)),
    plasticField_(std::move(plasticField)),
    elasticValues_(elasticValues),
    plasticValues_(plasticValues),
    elasticOwner_(std::move(elasticOwner)),
    plasticOwner_(std::move(plasticOwner))
  {
  }

  std::shared_ptr<const OptimizableParameterField> elasticField_;
  std::shared_ptr<const OptimizableParameterField> plasticField_;
  std::span<const double> elasticValues_;
  std::span<const double> plasticValues_;
  std::shared_ptr<const EigenSupport::VXd> elasticOwner_;
  std::shared_ptr<const EigenSupport::VXd> plasticOwner_;
};

}  // namespace pgo::SolidDeformationModel
