#pragma once

#include "material/elastic/elasticModel.h"
#include "material/elastic/elasticModelStableNeoHookeanMaterial.h"
#include "material/elastic/elasticModel3DSTVKMaterial.h"
#include "material/elastic/elasticModelCombinedMaterial.h"
#include "material/elastic/elasticModelLinearMaterial.h"
#include "material/elastic/elasticModel3DMooneyRivlin.h"
#include "material/elastic/elasticModel2DFundamentalFormsSTVK.h"
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
  std::string frameRequirement() const;
  std::shared_ptr<const SolidDeformationModel::ElasticModelDefinition> definition() const { return definition_; }

protected:
  std::shared_ptr<const SolidDeformationModel::ElasticModelDefinition> definition_;
};

class PyStableNeoDefinition final : public PyElasticModelDefinition { public: PyStableNeoDefinition(); };
class PyStVKDefinition final : public PyElasticModelDefinition { public: PyStVKDefinition(); };
class PyStVKVolumeDefinition final : public PyElasticModelDefinition { public: PyStVKVolumeDefinition(); };
class PyLinearElasticDefinition final : public PyElasticModelDefinition { public: PyLinearElasticDefinition(); };
class PyMooneyRivlinDefinition final : public PyElasticModelDefinition { public: PyMooneyRivlinDefinition(); };
class PyKoiterStVKDefinition final : public PyElasticModelDefinition { public: PyKoiterStVKDefinition(); };

}  // namespace pgo
