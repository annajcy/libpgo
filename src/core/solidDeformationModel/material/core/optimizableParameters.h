#pragma once

#include "materialParameterField.h"
#include "EigenSupport.h"

#include <memory>
#include <mutex>
#include <span>
#include <vector>

namespace pgo::SolidDeformationModel
{

/// Reusable storage for sampling one optimizable parameter field.
struct OptimizableParameterEvaluationScratch
{
  std::vector<double> local;
  std::vector<double> material;
  EigenSupport::MXd jacobian;

  void prepare(
    const OptimizableParameterField &field,
    int numMaterialLocations = 1);
};

class OptimizableParameterEvaluationView;

/// Immutable committed value snapshot. Field handles are retained privately
/// to preserve field identity during evaluation.
class OptimizableParameterSnapshot
{
public:
  OptimizableParameterSnapshot() = default;

  bool empty() const { return !elasticField_ || !plasticField_; }
  const OptimizableParameterField &elasticField() const
  {
    return *elasticField_;
  }
  const OptimizableParameterField &plasticField() const
  {
    return *plasticField_;
  }
  const EigenSupport::VXd &elasticValues() const { return *elasticValues_; }
  const EigenSupport::VXd &plasticValues() const { return *plasticValues_; }

  OptimizableParameterEvaluationView view() const;
  OptimizableParameterEvaluationView withElasticValues(
    std::span<const double> elasticValues) const;
  OptimizableParameterEvaluationView withPlasticValues(
    std::span<const double> plasticValues) const;
  OptimizableParameterEvaluationView withValues(
    std::span<const double> elasticValues,
    std::span<const double> plasticValues) const;

private:
  friend class OptimizableParameters;

  OptimizableParameterSnapshot(
    std::shared_ptr<const OptimizableParameterField> elasticField,
    std::shared_ptr<const OptimizableParameterField> plasticField,
    std::shared_ptr<const EigenSupport::VXd> elasticValues,
    std::shared_ptr<const EigenSupport::VXd> plasticValues);

  std::shared_ptr<const OptimizableParameterField> elasticField_;
  std::shared_ptr<const OptimizableParameterField> plasticField_;
  std::shared_ptr<const EigenSupport::VXd> elasticValues_;
  std::shared_ptr<const EigenSupport::VXd> plasticValues_;
};

/// Non-owning trial value view. Committed owners are retained when needed;
/// caller-provided trial spans are borrowed for one evaluation only.
class OptimizableParameterEvaluationView
{
public:
  OptimizableParameterEvaluationView() = default;

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
    OptimizableParameterEvaluationScratch &scratch) const;

private:
  friend class OptimizableParameterSnapshot;

  OptimizableParameterEvaluationView(
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

/// Thread-safe handle to the currently committed optimizable values.
class OptimizableParameters final
{
public:
  OptimizableParameters(
    std::shared_ptr<const OptimizableParameterField> elasticField,
    std::shared_ptr<const OptimizableParameterField> plasticField,
    EigenSupport::VXd elasticValues,
    EigenSupport::VXd plasticValues);

  const OptimizableParameterField &elasticField() const
  {
    return *elasticField_;
  }
  const OptimizableParameterField &plasticField() const
  {
    return *plasticField_;
  }
  const std::shared_ptr<const OptimizableParameterField> &
  elasticFieldHandle() const
  {
    return elasticField_;
  }
  const std::shared_ptr<const OptimizableParameterField> &
  plasticFieldHandle() const
  {
    return plasticField_;
  }

  OptimizableParameterSnapshot snapshot() const;
  EigenSupport::VXd elasticSnapshot() const;
  EigenSupport::VXd plasticSnapshot() const;

  void setElasticValues(EigenSupport::ConstRefVecXd values);
  void setPlasticValues(EigenSupport::ConstRefVecXd values);
  void setValues(
    EigenSupport::ConstRefVecXd elasticValues,
    EigenSupport::ConstRefVecXd plasticValues);

private:
  struct CommittedValues
  {
    std::shared_ptr<const EigenSupport::VXd> elastic;
    std::shared_ptr<const EigenSupport::VXd> plastic;
  };

  std::shared_ptr<const OptimizableParameterField> elasticField_;
  std::shared_ptr<const OptimizableParameterField> plasticField_;
  mutable std::mutex mutex_;
  std::shared_ptr<const CommittedValues> committed_;
};

}  // namespace pgo::SolidDeformationModel
