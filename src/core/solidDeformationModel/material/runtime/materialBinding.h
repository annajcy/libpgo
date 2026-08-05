#pragma once

#include "material/frame/materialFrameField.h"
#include "material/model/elasticModelDefinition.h"
#include "material/model/plasticModelDefinition.h"
#include "material/parameterization/materialParameterField.h"
#include "EigenSupport.h"

#include <memory>
#include <stdexcept>
#include <utility>

namespace pgo::SolidDeformationModel
{

/// Immutable fixed material parameter field and its global values.
class FixedMaterialParameters final
{
public:
  FixedMaterialParameters(
    std::shared_ptr<const FixedParameterField> field,
    EigenSupport::VXd values):
    field_(std::move(field)), values_(std::move(values))
  {
    if (!field_)
      throw std::invalid_argument(
        "FixedMaterialParameters requires a parameter field.");
    if (values_.size() != field_->numGlobalParameters())
      throw std::invalid_argument(
        "Fixed material value count does not match its field layout.");
    if (!values_.allFinite())
      throw std::invalid_argument(
        "Fixed material values must be finite.");
  }

  const std::shared_ptr<const FixedParameterField> &field() const
  {
    return field_;
  }
  const EigenSupport::VXd &values() const { return values_; }

private:
  std::shared_ptr<const FixedParameterField> field_;
  EigenSupport::VXd values_;
};

/// Immutable material definition plus independent fixed and optimizable fields.
template<class ModelDefinition>
class MaterialDomainBinding final
{
public:
  MaterialDomainBinding(
    std::shared_ptr<const ModelDefinition> definition,
    FixedMaterialParameters fixed,
    std::shared_ptr<const OptimizableParameterField> optimizableField):
    definition_(std::move(definition)),
    fixed_(std::move(fixed)),
    optimizableField_(std::move(optimizableField))
  {
    if (!definition_ || !optimizableField_)
      throw std::invalid_argument(
        "MaterialDomainBinding requires a definition and optimizable field.");
    if (fixed_.field()->numElements() != optimizableField_->numElements())
      throw std::invalid_argument(
        "MaterialDomainBinding fields must share an element count.");
    if (fixed_.field()->numMaterialChannels() !=
      definition_->fixedChannelSchema().numChannels())
      throw std::invalid_argument(
        "MaterialDomainBinding fixed channel schema does not match definition.");
    if (optimizableField_->numMaterialChannels() !=
      definition_->optimizableChannelSchema().numChannels())
      throw std::invalid_argument(
        "MaterialDomainBinding optimizable channel schema does not match definition.");
  }

  const std::shared_ptr<const ModelDefinition> &definition() const
  {
    return definition_;
  }
  const FixedMaterialParameters &fixed() const { return fixed_; }
  const std::shared_ptr<const OptimizableParameterField> &optimizableField() const
  {
    return optimizableField_;
  }
  int numElements() const { return optimizableField_->numElements(); }

private:
  std::shared_ptr<const ModelDefinition> definition_;
  FixedMaterialParameters fixed_;
  std::shared_ptr<const OptimizableParameterField> optimizableField_;
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
    std::shared_ptr<const MaterialFrameField> materialFrames):
    elastic_(std::move(elastic)),
    plastic_(std::move(plastic)),
    materialFrames_(std::move(materialFrames))
  {
    if (!materialFrames_)
      throw std::invalid_argument(
        "MaterialBinding requires material frames.");
    if (elastic_.numElements() != plastic_.numElements())
      throw std::invalid_argument(
        "MaterialBinding domains must share an element count.");
    if (materialFrames_->numElements() != elastic_.numElements())
      throw std::invalid_argument(
        "MaterialBinding frame count does not match its parameter fields.");
  }

  const ElasticMaterialBinding &elastic() const { return elastic_; }
  const PlasticMaterialBinding &plastic() const { return plastic_; }
  const std::shared_ptr<const MaterialFrameField> &materialFrames() const
  {
    return materialFrames_;
  }
  int numElements() const { return elastic_.numElements(); }

private:
  ElasticMaterialBinding elastic_;
  PlasticMaterialBinding plastic_;
  std::shared_ptr<const MaterialFrameField> materialFrames_;
};

}  // namespace pgo::SolidDeformationModel
