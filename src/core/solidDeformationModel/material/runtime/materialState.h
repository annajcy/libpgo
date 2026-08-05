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

/// Immutable elastic and plastic global optimizable parameter values.
class MaterialState
{
public:
  MaterialState();
  MaterialState(
    EigenSupport::VXd elasticValues,
    EigenSupport::VXd plasticValues);

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
    std::shared_ptr<const EigenSupport::VXd> elasticValues,
    std::shared_ptr<const EigenSupport::VXd> plasticValues);

  std::shared_ptr<const EigenSupport::VXd> elasticValues_;
  std::shared_ptr<const EigenSupport::VXd> plasticValues_;
};

/// Non-owning values used during one material evaluation.
class MaterialStateView
{
public:
  MaterialStateView() = default;
  MaterialStateView(
    std::span<const double> elasticValues,
    std::span<const double> plasticValues):
    elasticValues_(elasticValues), plasticValues_(plasticValues)
  {
  }

  std::span<const double> elasticValues() const { return elasticValues_; }
  std::span<const double> plasticValues() const { return plasticValues_; }

  void evaluateElement(
    const OptimizableParameterField &field,
    std::span<const double> globalValues,
    int element,
    int numMaterialLocations,
    std::span<double> localParameterScratch,
    std::span<double> materialValues) const;
  std::span<const double> evaluateElement(
    const OptimizableParameterField &field,
    std::span<const double> globalValues,
    int element,
    int numMaterialLocations,
    MaterialStateEvaluationScratch &scratch) const;

private:
  std::span<const double> elasticValues_;
  std::span<const double> plasticValues_;
};

}  // namespace pgo::SolidDeformationModel
