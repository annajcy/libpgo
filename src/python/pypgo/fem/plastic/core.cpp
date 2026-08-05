#include "core.h"

namespace pgo
{
namespace SD = SolidDeformationModel;

PyVolumetricPlasticity0Definition::PyVolumetricPlasticity0Definition(): PyPlasticModelDefinition(std::make_shared<SD::VolumetricPlasticity0Definition>(), 0) {}
PyVolumetricPlasticity3Definition::PyVolumetricPlasticity3Definition(): PyPlasticModelDefinition(std::make_shared<SD::VolumetricPlasticity3Definition>(), 3) {}
PyVolumetricPlasticity6Definition::PyVolumetricPlasticity6Definition(): PyPlasticModelDefinition(std::make_shared<SD::VolumetricPlasticity6Definition>(), 6) {}
PyShellPlasticity0Definition::PyShellPlasticity0Definition(): PyPlasticModelDefinition(std::make_shared<SD::ShellPlasticity0Definition>(), 0) {}
PyShellPlasticity1Definition::PyShellPlasticity1Definition(): PyPlasticModelDefinition(std::make_shared<SD::ShellPlasticity1Definition>(), 1) {}

}  // namespace pgo
