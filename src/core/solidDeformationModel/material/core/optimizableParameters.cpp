#include "optimizableParameters.h"

#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace
{
void validateStateSize(const char *name, std::size_t actual, int expected)
{
  if (actual != static_cast<std::size_t>(expected))
    throw std::invalid_argument(
      std::string(name) + " value count does not match its parameter field.");
}

std::shared_ptr<const EigenSupport::VXd> makeValueOwner(
  EigenSupport::VXd values)
{
  return std::make_shared<const EigenSupport::VXd>(std::move(values));
}
}  // namespace

void OptimizableParameterEvaluationScratch::prepare(
  const OptimizableParameterField &field,
  int numMaterialLocations)
{
  if (numMaterialLocations < 0)
    throw std::invalid_argument(
      "Optimizable parameter location count must be non-negative.");
  const std::size_t localParameters = static_cast<std::size_t>(
    field.layout().numLocalParameters());
  const std::size_t channels = static_cast<std::size_t>(
    field.evaluator().numChannels());
  local.resize(localParameters);
  material.resize(
    static_cast<std::size_t>(numMaterialLocations) * channels);
  jacobian.resize(static_cast<Eigen::Index>(channels),
    static_cast<Eigen::Index>(localParameters));
}

OptimizableParameterSnapshot::OptimizableParameterSnapshot(
  std::shared_ptr<const OptimizableParameterField> elasticField,
  std::shared_ptr<const OptimizableParameterField> plasticField,
  std::shared_ptr<const EigenSupport::VXd> elasticValues,
  std::shared_ptr<const EigenSupport::VXd> plasticValues):
  elasticField_(std::move(elasticField)),
  plasticField_(std::move(plasticField)),
  elasticValues_(std::move(elasticValues)),
  plasticValues_(std::move(plasticValues))
{
  if (!elasticField_ || !plasticField_ || !elasticValues_ || !plasticValues_)
    throw std::invalid_argument(
      "OptimizableParameterSnapshot requires complete state handles.");
  validateStateSize(
    "elastic", elasticValues_->size(),
    elasticField_->layout().numGlobalParameters());
  validateStateSize(
    "plastic", plasticValues_->size(),
    plasticField_->layout().numGlobalParameters());
}

OptimizableParameterEvaluationView OptimizableParameterSnapshot::view() const
{
  if (empty())
    return {};
  return OptimizableParameterEvaluationView(
    elasticField_, plasticField_,
    std::span<const double>(elasticValues_->data(), elasticValues_->size()),
    std::span<const double>(plasticValues_->data(), plasticValues_->size()),
    elasticValues_, plasticValues_);
}

OptimizableParameterEvaluationView
OptimizableParameterSnapshot::withElasticValues(
  std::span<const double> elasticValues) const
{
  if (empty())
    throw std::invalid_argument("OptimizableParameterSnapshot is empty.");
  validateStateSize(
    "elastic", elasticValues.size(),
    elasticField_->layout().numGlobalParameters());
  return OptimizableParameterEvaluationView(
    elasticField_, plasticField_, elasticValues,
    std::span<const double>(plasticValues_->data(), plasticValues_->size()),
    {}, plasticValues_);
}

OptimizableParameterEvaluationView
OptimizableParameterSnapshot::withPlasticValues(
  std::span<const double> plasticValues) const
{
  if (empty())
    throw std::invalid_argument("OptimizableParameterSnapshot is empty.");
  validateStateSize(
    "plastic", plasticValues.size(),
    plasticField_->layout().numGlobalParameters());
  return OptimizableParameterEvaluationView(
    elasticField_, plasticField_,
    std::span<const double>(elasticValues_->data(), elasticValues_->size()),
    plasticValues, elasticValues_, {});
}

OptimizableParameterEvaluationView OptimizableParameterSnapshot::withValues(
  std::span<const double> elasticValues,
  std::span<const double> plasticValues) const
{
  if (empty())
    throw std::invalid_argument("OptimizableParameterSnapshot is empty.");
  validateStateSize(
    "elastic", elasticValues.size(),
    elasticField_->layout().numGlobalParameters());
  validateStateSize(
    "plastic", plasticValues.size(),
    plasticField_->layout().numGlobalParameters());
  return OptimizableParameterEvaluationView(
    elasticField_, plasticField_, elasticValues, plasticValues);
}

const OptimizableParameterField &
OptimizableParameterEvaluationView::elasticField() const
{
  if (empty())
    throw std::logic_error(
      "An empty optimizable parameter evaluation view has no elastic field.");
  return *elasticField_;
}

const OptimizableParameterField &
OptimizableParameterEvaluationView::plasticField() const
{
  if (empty())
    throw std::logic_error(
      "An empty optimizable parameter evaluation view has no plastic field.");
  return *plasticField_;
}

std::span<const double> OptimizableParameterEvaluationView::values(
  const OptimizableParameterField &field) const
{
  if (empty())
    throw std::invalid_argument(
      "Optimizable parameter evaluation view is empty.");
  if (field.sharesStateWith(*elasticField_))
    return elasticValues_;
  if (field.sharesStateWith(*plasticField_))
    return plasticValues_;
  throw std::invalid_argument(
    "Optimizable parameter field does not belong to the evaluation view.");
}

void OptimizableParameterEvaluationView::evaluateElement(
  const OptimizableParameterField &field,
  int element,
  int numMaterialLocations,
  std::span<double> localParameterScratch,
  std::span<double> materialValues) const
{
  const ParameterLayout &layout = field.layout();
  const DifferentiableMaterialEvaluator &evaluator = field.evaluator();
  if (element < 0 || element >= layout.numElements())
    throw std::out_of_range(
      "Optimizable parameter element is out of range.");
  if (numMaterialLocations < 0)
    throw std::invalid_argument(
      "Optimizable parameter location count must be non-negative.");

  const std::span<const double> globalValues = values(field);
  const std::size_t numLocalParameters =
    static_cast<std::size_t>(layout.numLocalParameters());
  const std::size_t numChannels =
    static_cast<std::size_t>(evaluator.numChannels());
  const std::size_t requiredMaterialValues =
    static_cast<std::size_t>(numMaterialLocations) * numChannels;
  if (localParameterScratch.size() < numLocalParameters)
    throw std::invalid_argument(
      "Optimizable parameter local scratch is too small.");
  if (materialValues.size() < requiredMaterialValues)
    throw std::invalid_argument(
      "Optimizable parameter output is too small.");
  if (numMaterialLocations == 0 || numChannels == 0)
    return;

  const std::span<double> localParameters =
    localParameterScratch.first(numLocalParameters);
  layout.gather(element, globalValues, localParameters);
  for (int q = 0; q < numMaterialLocations; q++) {
    evaluator.evaluate(
      element, q, localParameters,
      materialValues.subspan(
        static_cast<std::size_t>(q) * numChannels, numChannels));
  }
}

std::span<const double> OptimizableParameterEvaluationView::evaluateElement(
  const OptimizableParameterField &field,
  int element,
  int numMaterialLocations,
  OptimizableParameterEvaluationScratch &scratch) const
{
  scratch.prepare(field, numMaterialLocations);
  evaluateElement(
    field, element, numMaterialLocations,
    scratch.local, scratch.material);
  return scratch.material;
}

OptimizableParameters::OptimizableParameters(
  std::shared_ptr<const OptimizableParameterField> elasticField,
  std::shared_ptr<const OptimizableParameterField> plasticField,
  EigenSupport::VXd elasticValues,
  EigenSupport::VXd plasticValues):
  elasticField_(std::move(elasticField)),
  plasticField_(std::move(plasticField))
{
  if (!elasticField_ || !plasticField_)
    throw std::invalid_argument(
      "OptimizableParameters requires elastic and plastic fields.");
  if (elasticField_->numElements() != plasticField_->numElements())
    throw std::invalid_argument(
      "OptimizableParameters fields must share an element count.");
  validateStateSize(
    "elastic", elasticValues.size(),
    elasticField_->layout().numGlobalParameters());
  validateStateSize(
    "plastic", plasticValues.size(),
    plasticField_->layout().numGlobalParameters());
  auto committed = std::make_shared<CommittedValues>();
  committed->elastic = makeValueOwner(std::move(elasticValues));
  committed->plastic = makeValueOwner(std::move(plasticValues));
  committed_ = std::move(committed);
}

OptimizableParameterSnapshot OptimizableParameters::snapshot() const
{
  std::lock_guard lock(mutex_);
  return OptimizableParameterSnapshot(
    elasticField_, plasticField_, committed_->elastic, committed_->plastic);
}

EigenSupport::VXd OptimizableParameters::elasticSnapshot() const
{
  return snapshot().elasticValues();
}

EigenSupport::VXd OptimizableParameters::plasticSnapshot() const
{
  return snapshot().plasticValues();
}

void OptimizableParameters::setElasticValues(
  EigenSupport::ConstRefVecXd values)
{
  validateStateSize(
    "elastic", static_cast<std::size_t>(values.size()),
    elasticField_->layout().numGlobalParameters());
  auto elasticOwner = makeValueOwner(EigenSupport::VXd(values));
  std::lock_guard lock(mutex_);
  auto next = std::make_shared<CommittedValues>();
  next->elastic = std::move(elasticOwner);
  next->plastic = committed_->plastic;
  committed_ = std::move(next);
}

void OptimizableParameters::setPlasticValues(
  EigenSupport::ConstRefVecXd values)
{
  validateStateSize(
    "plastic", static_cast<std::size_t>(values.size()),
    plasticField_->layout().numGlobalParameters());
  auto plasticOwner = makeValueOwner(EigenSupport::VXd(values));
  std::lock_guard lock(mutex_);
  auto next = std::make_shared<CommittedValues>();
  next->elastic = committed_->elastic;
  next->plastic = std::move(plasticOwner);
  committed_ = std::move(next);
}

void OptimizableParameters::setValues(
  EigenSupport::ConstRefVecXd elasticValues,
  EigenSupport::ConstRefVecXd plasticValues)
{
  validateStateSize(
    "elastic", static_cast<std::size_t>(elasticValues.size()),
    elasticField_->layout().numGlobalParameters());
  validateStateSize(
    "plastic", static_cast<std::size_t>(plasticValues.size()),
    plasticField_->layout().numGlobalParameters());
  auto elasticOwner = makeValueOwner(EigenSupport::VXd(elasticValues));
  auto plasticOwner = makeValueOwner(EigenSupport::VXd(plasticValues));
  std::lock_guard lock(mutex_);
  auto next = std::make_shared<CommittedValues>();
  next->elastic = std::move(elasticOwner);
  next->plastic = std::move(plasticOwner);
  committed_ = std::move(next);
}

}  // namespace pgo::SolidDeformationModel
