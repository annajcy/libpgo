#include "core.h"

namespace pgo
{
namespace SD = SolidDeformationModel;

PyVolumetricPlasticity0Config::PyVolumetricPlasticity0Config(): PyPlasticModelConfig(std::make_shared<SD::VolumetricPlasticity0Config>(), 0) {}
PyVolumetricPlasticity3Config::PyVolumetricPlasticity3Config(): PyPlasticModelConfig(std::make_shared<SD::VolumetricPlasticity3Config>(), 3) {}
PyVolumetricPlasticity6Config::PyVolumetricPlasticity6Config(): PyPlasticModelConfig(std::make_shared<SD::VolumetricPlasticity6Config>(), 6) {}
PyShellPlasticity0Config::PyShellPlasticity0Config(): PyPlasticModelConfig(std::make_shared<SD::ShellPlasticity0Config>(), 0) {}
PyShellPlasticity1Config::PyShellPlasticity1Config(): PyPlasticModelConfig(std::make_shared<SD::ShellPlasticity1Config>(), 1) {}

}  // namespace pgo
