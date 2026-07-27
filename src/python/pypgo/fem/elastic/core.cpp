#include "core.h"

namespace pgo
{
namespace SD = SolidDeformationModel;

std::vector<std::string> PyElasticModelDefinition::fixedChannelNames() const
{
  const auto schema = definition_->fixedChannelSchema();
  const auto names = schema.channelNames();
  return std::vector<std::string>(names.begin(), names.end());
}

std::vector<std::string> PyElasticModelDefinition::optimizableChannelNames() const
{
  const auto schema = definition_->optimizableChannelSchema();
  const auto names = schema.channelNames();
  return std::vector<std::string>(names.begin(), names.end());
}

std::string PyElasticModelDefinition::frameRequirement() const
{
  switch (definition_->frameRequirement()) {
  case SD::MaterialFrameRequirement::None: return "none";
  case SD::MaterialFrameRequirement::PrimaryAxis: return "primary_axis";
  case SD::MaterialFrameRequirement::FullFrame: return "full_frame";
  }
  return "unknown";
}

PyStableNeoDefinition::PyStableNeoDefinition(): PyElasticModelDefinition(std::make_shared<SD::StableNeoDefinition>()) {}
PyStVKDefinition::PyStVKDefinition(): PyElasticModelDefinition(std::make_shared<SD::StVKDefinition>()) {}
PyStVKVolumeDefinition::PyStVKVolumeDefinition(): PyElasticModelDefinition(std::make_shared<SD::StVKVolumeDefinition>()) {}
PyLinearElasticDefinition::PyLinearElasticDefinition(): PyElasticModelDefinition(std::make_shared<SD::LinearElasticDefinition>()) {}
PyMooneyRivlinDefinition::PyMooneyRivlinDefinition(): PyElasticModelDefinition(std::make_shared<SD::MooneyRivlinDefinition>()) {}
PyKoiterStVKDefinition::PyKoiterStVKDefinition(): PyElasticModelDefinition(std::make_shared<SD::KoiterStVKDefinition>()) {}

}  // namespace pgo
