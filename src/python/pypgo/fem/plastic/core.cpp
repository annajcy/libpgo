#include "core.h"

namespace pgo
{
namespace SD = SolidDeformationModel;

std::vector<std::string> PyPlasticModelDefinition::fixedChannelNames() const
{
  const auto schema = definition_->fixedChannelSchema();
  const auto names = schema.channelNames();
  return std::vector<std::string>(names.begin(), names.end());
}

std::vector<std::string> PyPlasticModelDefinition::optimizableChannelNames() const
{
  const auto schema = definition_->optimizableChannelSchema();
  const auto names = schema.channelNames();
  return std::vector<std::string>(names.begin(), names.end());
}

std::string PyPlasticModelDefinition::frameRequirement() const
{
  switch (definition_->frameRequirement()) {
  case SD::MaterialFrameRequirement::None: return "none";
  case SD::MaterialFrameRequirement::PrimaryAxis: return "primary_axis";
  case SD::MaterialFrameRequirement::FullFrame: return "full_frame";
  }
  return "unknown";
}

PyVolumetricPlasticity0Definition::PyVolumetricPlasticity0Definition(): PyPlasticModelDefinition(std::make_shared<SD::VolumetricPlasticity0Definition>(), 0) {}
PyVolumetricPlasticity3Definition::PyVolumetricPlasticity3Definition(): PyPlasticModelDefinition(std::make_shared<SD::VolumetricPlasticity3Definition>(), 3) {}
PyVolumetricPlasticity6Definition::PyVolumetricPlasticity6Definition(): PyPlasticModelDefinition(std::make_shared<SD::VolumetricPlasticity6Definition>(), 6) {}
PyShellPlasticity0Definition::PyShellPlasticity0Definition(): PyPlasticModelDefinition(std::make_shared<SD::ShellPlasticity0Definition>(), 0) {}
PyShellPlasticity1Definition::PyShellPlasticity1Definition(): PyPlasticModelDefinition(std::make_shared<SD::ShellPlasticity1Definition>(), 1) {}

}  // namespace pgo
