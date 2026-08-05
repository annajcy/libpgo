#include "material/runtime/materialState.h"

#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace
{
std::shared_ptr<const EigenSupport::VXd> makeValueOwner(
  EigenSupport::VXd values)
{
  if (!values.allFinite())
    throw std::invalid_argument("MaterialState values must be finite.");
  return std::make_shared<const EigenSupport::VXd>(std::move(values));
}

EigenSupport::VXd copyValues(std::span<const double> values)
{
  return EigenSupport::VXd(
    Eigen::Map<const EigenSupport::VXd>(values.data(), values.size()));
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

MaterialState::MaterialState():
  MaterialState(EigenSupport::VXd{}, EigenSupport::VXd{})
{
}

MaterialState::MaterialState(
  EigenSupport::VXd elasticValues,
  EigenSupport::VXd plasticValues):
  MaterialState(
    makeValueOwner(std::move(elasticValues)),
    makeValueOwner(std::move(plasticValues)))
{
}

MaterialState::MaterialState(
  std::shared_ptr<const EigenSupport::VXd> elasticValues,
  std::shared_ptr<const EigenSupport::VXd> plasticValues):
  elasticValues_(std::move(elasticValues)),
  plasticValues_(std::move(plasticValues))
{
  if (!elasticValues_ || !plasticValues_)
    throw std::invalid_argument(
      "MaterialState requires complete value storage.");
}

MaterialStateView MaterialState::view() const
{
  return MaterialStateView(
    std::span<const double>(elasticValues_->data(), elasticValues_->size()),
    std::span<const double>(plasticValues_->data(), plasticValues_->size()));
}

MaterialState::operator MaterialStateView() const
{
  return view();
}

MaterialState MaterialState::withElasticValues(
  std::span<const double> elasticValues) const
{
  return MaterialState(
    makeValueOwner(copyValues(elasticValues)), plasticValues_);
}

MaterialState MaterialState::withPlasticValues(
  std::span<const double> plasticValues) const
{
  return MaterialState(
    elasticValues_, makeValueOwner(copyValues(plasticValues)));
}

MaterialState MaterialState::withValues(
  std::span<const double> elasticValues,
  std::span<const double> plasticValues) const
{
  return MaterialState(
    copyValues(elasticValues), copyValues(plasticValues));
}

void MaterialStateView::evaluateElement(
  const OptimizableParameterField &field,
  std::span<const double> globalValues,
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
  std::span<const double> globalValues,
  int element,
  int numMaterialLocations,
  MaterialStateEvaluationScratch &scratch) const
{
  scratch.prepare(field, numMaterialLocations);
  evaluateElement(
    field, globalValues, element, numMaterialLocations,
    scratch.local, scratch.material);
  return scratch.material;
}

}  // namespace pgo::SolidDeformationModel
