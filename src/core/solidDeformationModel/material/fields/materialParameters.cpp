#include "materialParameters.h"

#include <algorithm>
#include <stdexcept>
#include <unordered_set>

namespace pgo::SolidDeformationModel
{
namespace
{
void validateStateSize(
  const char *name,
  Eigen::Index actual,
  int expected)
{
  if (actual != expected)
    throw std::invalid_argument(
      std::string(name) + " value count does not match its parameter space.");
}

void validateStateSize(
  const char *name,
  std::size_t actual,
  int expected)
{
  if (actual != static_cast<std::size_t>(expected))
    throw std::invalid_argument(
      std::string(name) + " value count does not match its parameter space.");
}
}  // namespace

MaterialParameterBlock::MaterialParameterBlock(
  std::vector<std::string> channelNames,
  std::shared_ptr<const ParameterDofLayout> dofLayout,
  std::shared_ptr<const ParameterFieldMapping> mapping):
  channelNames_(std::move(channelNames)),
  dofLayout_(std::move(dofLayout)),
  mapping_(std::move(mapping))
{
  if (!dofLayout_)
    throw std::invalid_argument("MaterialParameterBlock requires a DOF layout.");
  if (!mapping_)
    throw std::invalid_argument("MaterialParameterBlock requires a field mapping.");
  if (mapping_->numInputDofs() != dofLayout_->numLocalDofs())
    throw std::invalid_argument("MaterialParameterBlock mapping input count does not match its layout.");
  if (mapping_->numChannels() != static_cast<int>(channelNames_.size()))
    throw std::invalid_argument("MaterialParameterBlock mapping channel count does not match its channel names.");

  std::unordered_set<std::string> names;
  for (const std::string &name : channelNames_) {
    if (name.empty())
      throw std::invalid_argument("MaterialParameterBlock channel names must be non-empty.");
    if (!names.insert(name).second)
      throw std::invalid_argument("MaterialParameterBlock channel names must be unique.");
  }
}

MaterialParameterRef MaterialParameterBlock::parameter(std::string_view name) const
{
  for (int i = 0; i < static_cast<int>(channelNames_.size()); i++) {
    if (channelNames_[i] == name)
      return MaterialParameterRef(*this, i);
  }
  throw std::invalid_argument(
    "MaterialParameterBlock has no channel named '" + std::string(name) + "'.");
}

MaterialParameterSpace::MaterialParameterSpace(
  MaterialParameterBlock elastic,
  MaterialParameterBlock plastic):
  elastic_(std::move(elastic)),
  plastic_(std::move(plastic))
{
  if (elastic_.dofLayout().numElements() != plastic_.dofLayout().numElements())
    throw std::invalid_argument("MaterialParameterSpace block element counts do not match.");
  elastic_.space_ = this;
  plastic_.space_ = this;
}

MaterialStateView MaterialParameterSpace::makeStateView(
  std::span<const double> elasticValues,
  std::span<const double> plasticValues) const
{
  validateStateSize(
    "elastic", elasticValues.size(), elastic_.dofLayout().numGlobalDofs());
  validateStateSize(
    "plastic", plasticValues.size(), plastic_.dofLayout().numGlobalDofs());
  return MaterialStateView(this, elasticValues, plasticValues);
}

MaterialState::MaterialState(
  std::shared_ptr<const MaterialParameterSpace> space,
  EigenSupport::VXd elasticValues,
  EigenSupport::VXd plasticValues):
  space_(std::move(space)),
  elasticValues_(std::move(elasticValues)),
  plasticValues_(std::move(plasticValues))
{
  if (!space_)
    throw std::invalid_argument("MaterialState requires a parameter space.");
  validateStateSize(
    "elastic", elasticValues_.size(), space_->elastic().dofLayout().numGlobalDofs());
  validateStateSize(
    "plastic", plasticValues_.size(), space_->plastic().dofLayout().numGlobalDofs());
}

MaterialStateView MaterialState::view() const
{
  return space_->makeStateView(
    std::span<const double>(elasticValues_.data(), elasticValues_.size()),
    std::span<const double>(plasticValues_.data(), plasticValues_.size()));
}

const MaterialParameterSpace &MaterialStateView::space() const
{
  if (!space_)
    throw std::logic_error("An empty MaterialStateView has no parameter space.");
  return *space_;
}

std::span<const double> MaterialStateView::values(
  const MaterialParameterBlock &block) const
{
  if (!space_)
    throw std::invalid_argument("MaterialStateView is empty.");
  if (block.space_ != space_)
    throw std::invalid_argument("MaterialStateView and parameter block belong to different spaces.");
  if (&block == &space_->elastic())
    return elasticValues_;
  if (&block == &space_->plastic())
    return plasticValues_;
  throw std::invalid_argument("MaterialStateView received an unknown parameter block.");
}

MaterialParameters::MaterialParameters(
  std::shared_ptr<const MaterialParameterSpace> space,
  EigenSupport::VXd elasticValues,
  EigenSupport::VXd plasticValues):
  space_(std::move(space)),
  elasticValues_(std::move(elasticValues)),
  plasticValues_(std::move(plasticValues))
{
  if (!space_)
    throw std::invalid_argument("MaterialParameters requires a parameter space.");
  validateStateSize(
    "elastic", elasticValues_.size(), space_->elastic().dofLayout().numGlobalDofs());
  validateStateSize(
    "plastic", plasticValues_.size(), space_->plastic().dofLayout().numGlobalDofs());
}

MaterialState MaterialParameters::snapshot() const
{
  return MaterialState(space_, elasticValues_, plasticValues_);
}

MaterialStateView MaterialParameters::committedView() const
{
  return space_->makeStateView(
    std::span<const double>(elasticValues_.data(), elasticValues_.size()),
    std::span<const double>(plasticValues_.data(), plasticValues_.size()));
}

void MaterialParameters::setElasticValues(EigenSupport::ConstRefVecXd values)
{
  validateStateSize(
    "elastic", values.size(), space_->elastic().dofLayout().numGlobalDofs());
  elasticValues_ = values;
}

void MaterialParameters::setPlasticValues(EigenSupport::ConstRefVecXd values)
{
  validateStateSize(
    "plastic", values.size(), space_->plastic().dofLayout().numGlobalDofs());
  plasticValues_ = values;
}

MaterialParameterRef::MaterialParameterRef(
  const MaterialParameterBlock &block, int channel):
  block_(&block), channel_(channel)
{
  if (channel < 0 || channel >= block.mapping().numChannels())
    throw std::out_of_range("MaterialParameterRef channel is out of range.");
}

const MaterialParameterBlock &MaterialParameterRef::block() const
{
  if (!block_)
    throw std::logic_error("MaterialParameterRef is empty.");
  return *block_;
}

std::string_view MaterialParameterRef::name() const
{
  const auto names = block().channelNames();
  return names[static_cast<std::size_t>(channel_)];
}

double MaterialParameterRef::value(
  int element,
  int quadrature,
  MaterialStateView state) const
{
  const MaterialParameterBlock &b = block();
  const ParameterDofLayout &layout = b.dofLayout();
  const ParameterFieldMapping &mapping = b.mapping();
  std::vector<double> local(static_cast<std::size_t>(layout.numLocalDofs()));
  std::vector<double> material(static_cast<std::size_t>(mapping.numChannels()));
  layout.gather(element, state.values(b), local);
  mapping.evaluate(element, quadrature, local, material);
  return material[static_cast<std::size_t>(channel_)];
}

void MaterialParameterRef::localDerivative(
  int element,
  int quadrature,
  MaterialStateView state,
  double *output) const
{
  const MaterialParameterBlock &b = block();
  const ParameterDofLayout &layout = b.dofLayout();
  const ParameterFieldMapping &mapping = b.mapping();
  if (layout.numLocalDofs() > 0 && output == nullptr)
    throw std::invalid_argument("MaterialParameterRef requires a derivative output buffer.");
  std::vector<double> local(static_cast<std::size_t>(layout.numLocalDofs()));
  std::vector<double> jacobian(
    static_cast<std::size_t>(mapping.numChannels()) * layout.numLocalDofs());
  layout.gather(element, state.values(b), local);
  mapping.evaluateJacobian(element, quadrature, local, jacobian.data());
  for (int k = 0; k < layout.numLocalDofs(); k++)
    output[k] = jacobian[static_cast<std::size_t>(k) * mapping.numChannels() + channel_];
}

}  // namespace pgo::SolidDeformationModel
