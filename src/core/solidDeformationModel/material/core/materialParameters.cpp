#include "materialParameters.h"

#include <algorithm>
#include <stdexcept>
#include <unordered_set>

namespace pgo::SolidDeformationModel
{
namespace
{
void validateStateSize(const char *name, std::size_t actual, int expected)
{
  if (actual != static_cast<std::size_t>(expected))
    throw std::invalid_argument(
      std::string(name) + " value count does not match its parameter space.");
}

std::shared_ptr<const EigenSupport::VXd> makeValueOwner(EigenSupport::VXd values)
{
  return std::make_shared<const EigenSupport::VXd>(std::move(values));
}
}  // namespace

MaterialParameterField::MaterialParameterField(
  std::vector<std::string> channelNames,
  std::shared_ptr<const ParameterDofLayout> dofLayout,
  std::shared_ptr<const MaterialChannelMapping> mapping):
  channelNames_(std::move(channelNames)),
  dofLayout_(std::move(dofLayout)),
  mapping_(std::move(mapping))
{
  if (!dofLayout_)
    throw std::invalid_argument("MaterialParameterField requires a DOF layout.");
  if (!mapping_)
    throw std::invalid_argument("MaterialParameterField requires a channel mapping.");
  if (mapping_->numInputDofs() != dofLayout_->numLocalDofs())
    throw std::invalid_argument(
      "MaterialParameterField mapping input count does not match its layout.");
  if (mapping_->numChannels() != static_cast<int>(channelNames_.size()))
    throw std::invalid_argument(
      "MaterialParameterField mapping channel count does not match its channel names.");

  std::unordered_set<std::string> names;
  for (const std::string &name : channelNames_) {
    if (name.empty())
      throw std::invalid_argument("MaterialParameterField channel names must be non-empty.");
    if (!names.insert(name).second)
      throw std::invalid_argument("MaterialParameterField channel names must be unique.");
  }
}

std::shared_ptr<const MaterialParameterField> MaterialParameterField::create(
  std::vector<std::string> channelNames,
  std::shared_ptr<const ParameterDofLayout> dofLayout,
  std::shared_ptr<const MaterialChannelMapping> mapping)
{
  return std::shared_ptr<const MaterialParameterField>(
    new MaterialParameterField(
      std::move(channelNames), std::move(dofLayout), std::move(mapping)));
}

void MaterialParameterEvaluationScratch::prepare(
  const MaterialParameterField &field,
  int numMaterialLocations)
{
  if (numMaterialLocations < 0)
    throw std::invalid_argument(
      "Material parameter location count must be non-negative.");
  const std::size_t localDofs = static_cast<std::size_t>(
    field.dofLayout().numLocalDofs());
  const std::size_t channels = static_cast<std::size_t>(
    field.channelMapping().numChannels());
  local.resize(localDofs);
  material.resize(
    static_cast<std::size_t>(numMaterialLocations) * channels);
  jacobian.resize(static_cast<Eigen::Index>(channels),
    static_cast<Eigen::Index>(localDofs));
}

MaterialParameterRef MaterialParameterField::parameter(std::string_view name) const
{
  for (int i = 0; i < static_cast<int>(channelNames_.size()); i++) {
    if (channelNames_[i] == name)
      return MaterialParameterRef(shared_from_this(), i);
  }
  throw std::invalid_argument(
    "MaterialParameterField has no channel named '" + std::string(name) + "'.");
}

MaterialParameterSpace::MaterialParameterSpace(
  std::shared_ptr<const MaterialParameterField> elastic,
  std::shared_ptr<const MaterialParameterField> plastic):
  elastic_(std::move(elastic)),
  plastic_(std::move(plastic))
{
  if (!elastic_ || !plastic_)
    throw std::invalid_argument("MaterialParameterSpace requires two fields.");
  if (elastic_->dofLayout().numElements() != plastic_->dofLayout().numElements())
    throw std::invalid_argument("MaterialParameterSpace field element counts do not match.");
}

MaterialParameterSnapshot::MaterialParameterSnapshot(
  std::shared_ptr<const MaterialParameterSpace> space,
  std::shared_ptr<const EigenSupport::VXd> elasticValues,
  std::shared_ptr<const EigenSupport::VXd> plasticValues):
  space_(std::move(space)),
  elasticValues_(std::move(elasticValues)),
  plasticValues_(std::move(plasticValues))
{
  if (!space_ || !elasticValues_ || !plasticValues_)
    throw std::invalid_argument("MaterialParameterSnapshot requires complete state handles.");
  validateStateSize(
    "elastic", elasticValues_->size(), space_->elastic().dofLayout().numGlobalDofs());
  validateStateSize(
    "plastic", plasticValues_->size(), space_->plastic().dofLayout().numGlobalDofs());
}

MaterialParameterEvaluationView MaterialParameterSnapshot::view() const
{
  if (!space_)
    return {};
  return MaterialParameterEvaluationView(
    space_,
    std::span<const double>(elasticValues_->data(), elasticValues_->size()),
    std::span<const double>(plasticValues_->data(), plasticValues_->size()),
    elasticValues_, plasticValues_);
}

MaterialParameterEvaluationView MaterialParameterSnapshot::withElasticValues(
  std::span<const double> elasticValues) const
{
  if (!space_)
    throw std::invalid_argument("MaterialParameterSnapshot is empty.");
  validateStateSize(
    "elastic", elasticValues.size(), space_->elastic().dofLayout().numGlobalDofs());
  return MaterialParameterEvaluationView(
    space_, elasticValues,
    std::span<const double>(plasticValues_->data(), plasticValues_->size()),
    {}, plasticValues_);
}

MaterialParameterEvaluationView MaterialParameterSnapshot::withPlasticValues(
  std::span<const double> plasticValues) const
{
  if (!space_)
    throw std::invalid_argument("MaterialParameterSnapshot is empty.");
  validateStateSize(
    "plastic", plasticValues.size(), space_->plastic().dofLayout().numGlobalDofs());
  return MaterialParameterEvaluationView(
    space_,
    std::span<const double>(elasticValues_->data(), elasticValues_->size()),
    plasticValues, elasticValues_, {});
}

MaterialParameterEvaluationView MaterialParameterSnapshot::withValues(
  std::span<const double> elasticValues,
  std::span<const double> plasticValues) const
{
  if (!space_)
    throw std::invalid_argument("MaterialParameterSnapshot is empty.");
  validateStateSize(
    "elastic", elasticValues.size(), space_->elastic().dofLayout().numGlobalDofs());
  validateStateSize(
    "plastic", plasticValues.size(), space_->plastic().dofLayout().numGlobalDofs());
  return MaterialParameterEvaluationView(
    space_, elasticValues, plasticValues);
}

const MaterialParameterSpace &MaterialParameterEvaluationView::space() const
{
  if (!space_)
    throw std::logic_error("An empty material parameter evaluation view has no space.");
  return *space_;
}

std::span<const double> MaterialParameterEvaluationView::values(
  const MaterialParameterField &field) const
{
  if (!space_)
    throw std::invalid_argument("Material parameter evaluation view is empty.");
  if (&field == &space_->elastic())
    return elasticValues_;
  if (&field == &space_->plastic())
    return plasticValues_;
  throw std::invalid_argument(
    "Material parameter field does not belong to the evaluation space.");
}

void MaterialParameterEvaluationView::evaluateElement(
  const MaterialParameterField &field,
  int element,
  int numMaterialLocations,
  std::span<double> localDofScratch,
  std::span<double> materialValues) const
{
  const ParameterDofLayout &layout = field.dofLayout();
  const MaterialChannelMapping &mapping = field.channelMapping();
  if (element < 0 || element >= layout.numElements())
    throw std::out_of_range("Material parameter element is out of range.");
  if (numMaterialLocations < 0)
    throw std::invalid_argument(
      "Material parameter location count must be non-negative.");

  const std::span<const double> globalValues = values(field);
  const std::size_t numLocalDofs =
    static_cast<std::size_t>(layout.numLocalDofs());
  const std::size_t numChannels =
    static_cast<std::size_t>(mapping.numChannels());
  const std::size_t requiredMaterialValues =
    static_cast<std::size_t>(numMaterialLocations) * numChannels;
  if (localDofScratch.size() < numLocalDofs)
    throw std::invalid_argument(
      "Material parameter local-DOF scratch is too small.");
  if (materialValues.size() < requiredMaterialValues)
    throw std::invalid_argument(
      "Material parameter output is too small.");
  if (numMaterialLocations == 0 || numChannels == 0)
    return;

  const std::span<double> localDofs =
    localDofScratch.first(numLocalDofs);
  layout.gather(element, globalValues, localDofs);
  for (int q = 0; q < numMaterialLocations; q++) {
    mapping.evaluate(
      element, q, localDofs,
      materialValues.subspan(
        static_cast<std::size_t>(q) * numChannels, numChannels));
  }
}

std::span<const double> MaterialParameterEvaluationView::evaluateElement(
  const MaterialParameterField &field,
  int element,
  int numMaterialLocations,
  MaterialParameterEvaluationScratch &scratch) const
{
  scratch.prepare(field, numMaterialLocations);
  evaluateElement(
    field, element, numMaterialLocations,
    scratch.local, scratch.material);
  return scratch.material;
}

MaterialParameters::MaterialParameters(
  std::shared_ptr<const MaterialParameterSpace> space,
  EigenSupport::VXd elasticValues,
  EigenSupport::VXd plasticValues):
  space_(std::move(space))
{
  if (!space_)
    throw std::invalid_argument("MaterialParameters requires a parameter space.");
  validateStateSize(
    "elastic", elasticValues.size(), space_->elastic().dofLayout().numGlobalDofs());
  validateStateSize(
    "plastic", plasticValues.size(), space_->plastic().dofLayout().numGlobalDofs());
  auto committed = std::make_shared<CommittedValues>();
  committed->elastic = makeValueOwner(std::move(elasticValues));
  committed->plastic = makeValueOwner(std::move(plasticValues));
  committed_ = std::move(committed);
}

MaterialParameterSnapshot MaterialParameters::snapshot() const
{
  std::lock_guard lock(mutex_);
  return MaterialParameterSnapshot(
    space_, committed_->elastic, committed_->plastic);
}

EigenSupport::VXd MaterialParameters::elasticSnapshot() const
{
  return snapshot().elasticValues();
}

EigenSupport::VXd MaterialParameters::plasticSnapshot() const
{
  return snapshot().plasticValues();
}

void MaterialParameters::setElasticValues(EigenSupport::ConstRefVecXd values)
{
  validateStateSize(
    "elastic", static_cast<std::size_t>(values.size()),
    space_->elastic().dofLayout().numGlobalDofs());
  auto replacement = makeValueOwner(EigenSupport::VXd(values));
  std::lock_guard lock(mutex_);
  auto next = std::make_shared<CommittedValues>();
  next->elastic = std::move(replacement);
  next->plastic = committed_->plastic;
  committed_ = std::move(next);
}

void MaterialParameters::setPlasticValues(EigenSupport::ConstRefVecXd values)
{
  validateStateSize(
    "plastic", static_cast<std::size_t>(values.size()),
    space_->plastic().dofLayout().numGlobalDofs());
  auto replacement = makeValueOwner(EigenSupport::VXd(values));
  std::lock_guard lock(mutex_);
  auto next = std::make_shared<CommittedValues>();
  next->elastic = committed_->elastic;
  next->plastic = std::move(replacement);
  committed_ = std::move(next);
}

MaterialParameterRef::MaterialParameterRef(
  std::shared_ptr<const MaterialParameterField> field, int channel):
  field_(std::move(field)),
  channel_(channel)
{
  if (!field_)
    throw std::invalid_argument("MaterialParameterRef requires a field.");
  if (channel < 0 || channel >= field_->channelMapping().numChannels())
    throw std::out_of_range("MaterialParameterRef channel is out of range.");
}

const MaterialParameterField &MaterialParameterRef::field() const
{
  if (!field_)
    throw std::logic_error("MaterialParameterRef is empty.");
  return *field_;
}

std::string_view MaterialParameterRef::name() const
{
  const auto names = field().channelNames();
  return names[static_cast<std::size_t>(channel_)];
}

double MaterialParameterRef::value(
  int element,
  int quadrature,
  const MaterialParameterEvaluationView &state) const
{
  MaterialParameterEvaluationScratch scratch;
  return value(element, quadrature, state, scratch);
}

double MaterialParameterRef::value(
  int element,
  int quadrature,
  const MaterialParameterEvaluationView &state,
  MaterialParameterEvaluationScratch &scratch) const
{
  const MaterialParameterField &f = field();
  const ParameterDofLayout &layout = f.dofLayout();
  const MaterialChannelMapping &mapping = f.channelMapping();
  scratch.prepare(f);
  layout.gather(element, state.values(f), scratch.local);
  mapping.evaluate(element, quadrature, scratch.local, scratch.material);
  return scratch.material[static_cast<std::size_t>(channel_)];
}

void MaterialParameterRef::localDerivative(
  int element,
  int quadrature,
  const MaterialParameterEvaluationView &state,
  EigenSupport::RefVecXd output) const
{
  MaterialParameterEvaluationScratch scratch;
  localDerivative(element, quadrature, state, scratch, output);
}

void MaterialParameterRef::localDerivative(
  int element,
  int quadrature,
  const MaterialParameterEvaluationView &state,
  MaterialParameterEvaluationScratch &scratch,
  EigenSupport::RefVecXd output) const
{
  const MaterialParameterField &f = field();
  const ParameterDofLayout &layout = f.dofLayout();
  const MaterialChannelMapping &mapping = f.channelMapping();
  if (output.size() != layout.numLocalDofs())
    throw std::invalid_argument("MaterialParameterRef derivative output has the wrong size.");
  scratch.prepare(f);
  layout.gather(element, state.values(f), scratch.local);
  mapping.evaluateJacobian(
    element, quadrature, scratch.local,
    scratch.jacobian);
  for (int k = 0; k < layout.numLocalDofs(); k++)
    output[k] = scratch.jacobian(channel_, k);
}

}  // namespace pgo::SolidDeformationModel
