#pragma once

#include "material/frame/materialFrames.h"
#include "material/elastic/elasticModelDefinition.h"
#include "material/plastic/plasticModelDefinition.h"
#include "EigenSupport.h"

#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

namespace pgo::SolidDeformationModel
{

/// One material domain with element-major fixed physical channel values.
/// Optimizable physical channels are supplied separately through MaterialState.
template<class ModelDefinition>
class MaterialDomainBinding final
{
public:
  MaterialDomainBinding(
    std::shared_ptr<const ModelDefinition> definition,
    int numElements,
    EigenSupport::VXd fixedValues):
    definition_(std::move(definition)),
    numElements_(numElements),
    fixedValues_(std::move(fixedValues))
  {
    if (!definition_)
      throw std::invalid_argument(
        "MaterialDomainBinding requires a model definition.");
    if (numElements_ < 0)
      throw std::invalid_argument(
        "MaterialDomainBinding element count must be non-negative.");
    const int expected =
      numElements_ * definition_->numFixedChannels();
    if (fixedValues_.size() != expected)
      throw std::invalid_argument(
        "MaterialDomainBinding fixed value count does not match the element and channel counts.");
    if (!fixedValues_.allFinite())
      throw std::invalid_argument(
        "MaterialDomainBinding fixed values must be finite.");
  }

  const std::shared_ptr<const ModelDefinition> &definition() const
  {
    return definition_;
  }
  int numElements() const { return numElements_; }
  int numFixedChannels() const
  {
    return definition_->numFixedChannels();
  }
  int numOptimizableChannels() const
  {
    return definition_->numOptimizableChannels();
  }
  const EigenSupport::VXd &fixedValues() const { return fixedValues_; }

private:
  std::shared_ptr<const ModelDefinition> definition_;
  int numElements_ = 0;
  EigenSupport::VXd fixedValues_;
};

using ElasticMaterialBinding =
  MaterialDomainBinding<ElasticModelDefinition>;
using PlasticMaterialBinding =
  MaterialDomainBinding<PlasticModelDefinition>;

/// Complete immutable material configuration for one deformation operator.
class MaterialBinding final
{
public:
  MaterialBinding(
    ElasticMaterialBinding elastic,
    PlasticMaterialBinding plastic,
    std::optional<MaterialFrames> materialFrames = std::nullopt):
    elastic_(std::move(elastic)),
    plastic_(std::move(plastic)),
    materialFrames_(materialFrames ? std::move(*materialFrames) :
      MaterialFrames::identity(elastic_.numElements()))
  {
    if (elastic_.numElements() != plastic_.numElements())
      throw std::invalid_argument(
        "MaterialBinding domains must share an element count.");
    if (materialFrames_.numElements() != elastic_.numElements())
      throw std::invalid_argument(
        "MaterialBinding frame count does not match its material domains.");
  }

  const ElasticMaterialBinding &elastic() const { return elastic_; }
  const PlasticMaterialBinding &plastic() const { return plastic_; }
  const MaterialFrames &materialFrames() const { return materialFrames_; }
  int numElements() const { return elastic_.numElements(); }

private:
  ElasticMaterialBinding elastic_;
  PlasticMaterialBinding plastic_;
  MaterialFrames materialFrames_;
};

}  // namespace pgo::SolidDeformationModel
