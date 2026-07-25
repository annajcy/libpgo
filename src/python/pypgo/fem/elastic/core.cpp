#include "core.h"

#include "../../simulation/core.h"

namespace pgo
{
namespace SD = SolidDeformationModel;

int PyElasticModelConfig::numChannels(const SD::SimulationMesh &) const
{
  return static_cast<int>(config_->parameterChannelNames().size());
}

int PyElasticModelConfig::numChannels(const PySimulationMesh &mesh) const
{
  return numChannels(mesh.mesh());
}

PyStableNeoConfig::PyStableNeoConfig(): PyElasticModelConfig(std::make_shared<SD::StableNeoConfig>()) {}
PyStVKConfig::PyStVKConfig(): PyElasticModelConfig(std::make_shared<SD::StVKConfig>()) {}
PyStVKVolumeConfig::PyStVKVolumeConfig(): PyElasticModelConfig(std::make_shared<SD::StVKVolumeConfig>()) {}
PyLinearElasticConfig::PyLinearElasticConfig(): PyElasticModelConfig(std::make_shared<SD::LinearElasticConfig>()) {}
PyMooneyRivlinConfig::PyMooneyRivlinConfig(): PyElasticModelConfig(std::make_shared<SD::MooneyRivlinConfig>()) {}
PyKoiterStVKConfig::PyKoiterStVKConfig(): PyElasticModelConfig(std::make_shared<SD::KoiterStVKConfig>()) {}

}  // namespace pgo
