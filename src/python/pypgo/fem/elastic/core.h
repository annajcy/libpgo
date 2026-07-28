#pragma once

#include "material/model/elasticModelDefinition.h"
#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModel3DNeoHookean.h"
#include "material/elastic/elasticModel3DSTVKMaterial.h"
#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModelLinearMaterial.h"
#include "material/elastic/elasticModel3DMooneyRivlin.h"
#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"
#include "material/elastic/deformationGradient/spectral/elasticModel3DSystematicPoking.h"
#include <memory>
#include <string>
#include <vector>

namespace pgo
{

// Python-facing immutable elastic model definition wrapper.
class PyElasticModelDefinition
{
public:
  explicit PyElasticModelDefinition(
    std::shared_ptr<const SolidDeformationModel::ElasticModelDefinition> definition):
    definition_(std::move(definition)) {}

  std::string name() const { return std::string(definition_->id()); }
  std::vector<std::string> fixedChannelNames() const;
  std::vector<std::string> optimizableChannelNames() const;
  std::shared_ptr<const SolidDeformationModel::ElasticModelDefinition> definition() const { return definition_; }

protected:
  std::shared_ptr<const SolidDeformationModel::ElasticModelDefinition> definition_;
};

class PyStableNeoDefinition final : public PyElasticModelDefinition { public: PyStableNeoDefinition(); };
class PyNeoHookeanDefinition final : public PyElasticModelDefinition { public: PyNeoHookeanDefinition(); };
class PyStVKDefinition final : public PyElasticModelDefinition { public: PyStVKDefinition(); };
class PyStVKVolumeDefinition final : public PyElasticModelDefinition { public: PyStVKVolumeDefinition(); };
class PyLinearElasticDefinition final : public PyElasticModelDefinition { public: PyLinearElasticDefinition(); };
class PyMooneyRivlinDefinition final : public PyElasticModelDefinition { public: PyMooneyRivlinDefinition(); };
class PyKoiterStVKDefinition final : public PyElasticModelDefinition { public: PyKoiterStVKDefinition(); };
class PySystematicPokingDefinition final : public PyElasticModelDefinition
{
public:
  PySystematicPokingDefinition(
    const std::vector<double> &stretchKnots,
    int stretchRestKnotIndex,
    const std::vector<double> &volumeKnots,
    int volumeRestKnotIndex);
};

}  // namespace pgo
