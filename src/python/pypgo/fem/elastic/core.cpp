#include "core.h"

namespace pgo
{
namespace SD = SolidDeformationModel;

PyStableNeoDefinition::PyStableNeoDefinition(): PyElasticModelDefinition(std::make_shared<SD::StableNeoDefinition>()) {}
PyNeoHookeanDefinition::PyNeoHookeanDefinition(): PyElasticModelDefinition(std::make_shared<SD::NeoHookeanDefinition>()) {}
PyStVKDefinition::PyStVKDefinition(): PyElasticModelDefinition(std::make_shared<SD::StVKDefinition>()) {}
PyStVKVolumeDefinition::PyStVKVolumeDefinition(): PyElasticModelDefinition(std::make_shared<SD::StVKVolumeDefinition>()) {}
PyLinearElasticDefinition::PyLinearElasticDefinition(): PyElasticModelDefinition(std::make_shared<SD::LinearElasticDefinition>()) {}
PyMooneyRivlinDefinition::PyMooneyRivlinDefinition(): PyElasticModelDefinition(std::make_shared<SD::MooneyRivlinDefinition>()) {}
PyKoiterStVKDefinition::PyKoiterStVKDefinition(): PyElasticModelDefinition(std::make_shared<SD::KoiterStVKDefinition>()) {}
PySystematicPokingDefinition::PySystematicPokingDefinition(
  const std::vector<double> &stretchKnots,
  int stretchRestKnotIndex,
  const std::vector<double> &volumeKnots,
  int volumeRestKnotIndex):
  PyElasticModelDefinition(
    std::make_shared<SD::SystematicPokingDefinition>(
      stretchKnots,
      stretchRestKnotIndex,
      volumeKnots,
      volumeRestKnotIndex))
{
}

}  // namespace pgo
