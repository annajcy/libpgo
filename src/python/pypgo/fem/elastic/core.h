#pragma once

#include "material/elastic/elasticModel.h"
#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModel3DSTVKMaterial.h"
#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModelLinearMaterial.h"
#include "material/elastic/elasticModel3DMooneyRivlin.h"
#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"
#include "simulation/simulationMesh.h"

#include <memory>
#include <string>

namespace pgo
{

class PySimulationMesh;

// Python-facing immutable elastic model configuration wrapper.
class PyElasticModelConfig
{
public:
  explicit PyElasticModelConfig(
    std::shared_ptr<const SolidDeformationModel::ElasticModelConfig> config):
    config_(std::move(config)) {}

  std::string name() const { return std::string(config_->id()); }

  // Number of parameter channels per element for the given mesh.
  int numChannels(const SolidDeformationModel::SimulationMesh &mesh) const;
  int numChannels(const PySimulationMesh &mesh) const;
  std::shared_ptr<const SolidDeformationModel::ElasticModelConfig> config() const { return config_; }

protected:
  std::shared_ptr<const SolidDeformationModel::ElasticModelConfig> config_;
};

class PyStableNeoConfig final : public PyElasticModelConfig { public: PyStableNeoConfig(); };
class PyStVKConfig final : public PyElasticModelConfig { public: PyStVKConfig(); };
class PyStVKVolumeConfig final : public PyElasticModelConfig { public: PyStVKVolumeConfig(); };
class PyLinearElasticConfig final : public PyElasticModelConfig { public: PyLinearElasticConfig(); };
class PyMooneyRivlinConfig final : public PyElasticModelConfig { public: PyMooneyRivlinConfig(); };
class PyKoiterStVKConfig final : public PyElasticModelConfig { public: PyKoiterStVKConfig(); };

}  // namespace pgo
