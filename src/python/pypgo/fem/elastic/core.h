#pragma once

#include "material/elastic/elasticModelDefinition.h"
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
  int numFixedChannels() const { return definition_->numFixedChannels(); }
  int numOptimizableChannels() const
  {
    return definition_->numOptimizableChannels();
  }
  std::shared_ptr<const SolidDeformationModel::ElasticModelDefinition> definition() const { return definition_; }

protected:
  std::shared_ptr<const SolidDeformationModel::ElasticModelDefinition> definition_;
};

// Default-constructible wrapper around one concrete C++ definition.  Each
// template instantiation is a distinct C++ type, so nanobind can register it
// as its own Python class while sharing the base wrapper implementation.
template<class DefinitionT>
class PyElasticModelDefinitionT final : public PyElasticModelDefinition
{
public:
  PyElasticModelDefinitionT():
    PyElasticModelDefinition(std::make_shared<DefinitionT>())
  {
  }
};

using PyStableNeoDefinition =
  PyElasticModelDefinitionT<SolidDeformationModel::StableNeoDefinition>;
using PyNeoHookeanDefinition =
  PyElasticModelDefinitionT<SolidDeformationModel::NeoHookeanDefinition>;
using PyStVKDefinition =
  PyElasticModelDefinitionT<SolidDeformationModel::StVKDefinition>;
using PyStVKVolumeDefinition =
  PyElasticModelDefinitionT<SolidDeformationModel::StVKVolumeDefinition>;
using PyLinearElasticDefinition =
  PyElasticModelDefinitionT<SolidDeformationModel::LinearElasticDefinition>;
using PyMooneyRivlinDefinition =
  PyElasticModelDefinitionT<SolidDeformationModel::MooneyRivlinDefinition>;
using PyKoiterStVKDefinition =
  PyElasticModelDefinitionT<SolidDeformationModel::KoiterStVKDefinition>;
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
