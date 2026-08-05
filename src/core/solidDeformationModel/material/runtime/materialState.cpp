#include "material/runtime/materialState.h"

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

void MaterialStateEvaluationScratch::prepare(
  const OptimizableParameterField &field,
  int numMaterialLocations)
{
  if (numMaterialLocations < 0)
    throw std::invalid_argument(
      "Optimizable parameter location count must be non-negative.");
  const std::size_t localParameters = static_cast<std::size_t>(
    field.layout().numLocalParameters());
  const std::size_t channels = static_cast<std::size_t>(
    field.mapping().numChannels());
  local.resize(localParameters);
  material.resize(
    static_cast<std::size_t>(numMaterialLocations) * channels);
  jacobian.resize(static_cast<Eigen::Index>(channels),
    static_cast<Eigen::Index>(localParameters));
}

MaterialState::MaterialState(
  std::shared_ptr<const OptimizableParameterField> elasticField,
  std::shared_ptr<const OptimizableParameterField> plasticField,
  EigenSupport::VXd elasticValues,
  EigenSupport::VXd plasticValues):
  MaterialState(
    std::move(elasticField), std::move(plasticField),
    makeValueOwner(std::move(elasticValues)),
    makeValueOwner(std::move(plasticValues)))
{
}

MaterialState::MaterialState(
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
      "MaterialState requires complete state handles.");
  if (elasticField_->numElements() != plasticField_->numElements())
    throw std::invalid_argument(
      "MaterialState fields must share an element count.");
  validateStateSize(
    "elastic", elasticValues_->size(),
    elasticField_->layout().numGlobalParameters());
  validateStateSize(
    "plastic", plasticValues_->size(),
    plasticField_->layout().numGlobalParameters());
}

MaterialStateView MaterialState::view() const
{
  if (empty())
    return {};
  return MaterialStateView(
    elasticField_, plasticField_,
    std::span<const double>(elasticValues_->data(), elasticValues_->size()),
    std::span<const double>(plasticValues_->data(), plasticValues_->size()),
    elasticValues_, plasticValues_);
}

MaterialState::operator MaterialStateView() const
{
  return view();
}

MaterialState
MaterialState::withElasticValues(
  std::span<const double> elasticValues) const
{
  if (empty())
    throw std::invalid_argument("MaterialState is empty.");
  validateStateSize(
    "elastic", elasticValues.size(),
    elasticField_->layout().numGlobalParameters());
  return MaterialState(
    elasticField_, plasticField_,
    makeValueOwner(EigenSupport::VXd(
      Eigen::Map<const EigenSupport::VXd>(
        elasticValues.data(), elasticValues.size()))),
    plasticValues_);
}

MaterialState
MaterialState::withPlasticValues(
  std::span<const double> plasticValues) const
{
  if (empty())
    throw std::invalid_argument("MaterialState is empty.");
  validateStateSize(
    "plastic", plasticValues.size(),
    plasticField_->layout().numGlobalParameters());
  return MaterialState(
    elasticField_, plasticField_, elasticValues_,
    makeValueOwner(EigenSupport::VXd(
      Eigen::Map<const EigenSupport::VXd>(
        plasticValues.data(), plasticValues.size()))));
}

MaterialState MaterialState::withValues(
  std::span<const double> elasticValues,
  std::span<const double> plasticValues) const
{
  if (empty())
    throw std::invalid_argument("MaterialState is empty.");
  validateStateSize(
    "elastic", elasticValues.size(),
    elasticField_->layout().numGlobalParameters());
  validateStateSize(
    "plastic", plasticValues.size(),
    plasticField_->layout().numGlobalParameters());
  return MaterialState(
    elasticField_, plasticField_,
    EigenSupport::VXd(Eigen::Map<const EigenSupport::VXd>(
      elasticValues.data(), elasticValues.size())),
    EigenSupport::VXd(Eigen::Map<const EigenSupport::VXd>(
      plasticValues.data(), plasticValues.size())));
}

const OptimizableParameterField &
MaterialStateView::elasticField() const
{
  if (empty())
    throw std::logic_error(
      "An empty optimizable parameter evaluation view has no elastic field.");
  return *elasticField_;
}

const OptimizableParameterField &
MaterialStateView::plasticField() const
{
  if (empty())
    throw std::logic_error(
      "An empty optimizable parameter evaluation view has no plastic field.");
  return *plasticField_;
}

std::span<const double> MaterialStateView::values(
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

void MaterialStateView::evaluateElement(
  const OptimizableParameterField &field,
  int element,
  int numMaterialLocations,
  std::span<double> localParameterScratch,
  std::span<double> materialValues) const
{
  const ParameterLayout &layout = field.layout();
  const DifferentiableMaterialChannelMapping &mapping = field.mapping();
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
    static_cast<std::size_t>(mapping.numChannels());
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
    mapping.evaluate(
      element, q, localParameters,
      materialValues.subspan(
        static_cast<std::size_t>(q) * numChannels, numChannels));
  }
}

std::span<const double> MaterialStateView::evaluateElement(
  const OptimizableParameterField &field,
  int element,
  int numMaterialLocations,
  MaterialStateEvaluationScratch &scratch) const
{
  scratch.prepare(field, numMaterialLocations);
  evaluateElement(
    field, element, numMaterialLocations,
    scratch.local, scratch.material);
  return scratch.material;
}

}  // namespace pgo::SolidDeformationModel
