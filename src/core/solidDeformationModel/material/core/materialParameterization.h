#pragma once

#include "materialParameterField.h"
#include "material/elastic/elasticModel.h"
#include "material/plastic/plasticModel.h"

#include <memory>
#include <stdexcept>
#include <algorithm>
#include <string_view>

namespace pgo::SolidDeformationModel
{

/// A material-domain parameterization: parameter values are gathered by the
/// field layouts and converted to physical channels by their evaluators.
template<class ModelDefinition>
class MaterialDomainParameterization final
{
public:
  MaterialDomainParameterization(
    std::shared_ptr<const ModelDefinition> definition,
    std::shared_ptr<const FixedParameterField> fixedField,
    std::shared_ptr<const OptimizableParameterField> optimizableField):
    definition_(std::move(definition)),
    fixedField_(std::move(fixedField)),
    optimizableField_(std::move(optimizableField)),
    fixedChannelSchema_(definition_ ? definition_->fixedChannelSchema() :
                                      MaterialChannelSchema{}),
    optimizableChannelSchema_(
      definition_ ? definition_->optimizableChannelSchema() :
                    MaterialChannelSchema{})
  {
    if (!definition_ || !fixedField_ || !optimizableField_)
      throw std::invalid_argument(
        "MaterialDomainParameterization requires definition and fields.");
    if (fixedField_->numElements() != optimizableField_->numElements())
      throw std::invalid_argument(
        "MaterialDomainParameterization fields must share an element count.");
    if (fixedField_->numMaterialChannels() !=
      fixedChannelSchema_.numChannels())
      throw std::invalid_argument(
        "MaterialDomainParameterization fixed channel schema does not match definition.");
    if (optimizableField_->evaluator().numChannels() !=
      optimizableChannelSchema_.numChannels())
      throw std::invalid_argument(
        "MaterialDomainParameterization optimizable channel schema does not match definition.");
  }

  const std::shared_ptr<const ModelDefinition> &definition() const
  {
    return definition_;
  }
  const std::shared_ptr<const FixedParameterField> &fixedField() const
  {
    return fixedField_;
  }
  const std::shared_ptr<const OptimizableParameterField> &optimizableField() const
  {
    return optimizableField_;
  }
  const MaterialChannelSchema &fixedChannelSchema() const
  {
    return fixedChannelSchema_;
  }
  const MaterialChannelSchema &optimizableChannelSchema() const
  {
    return optimizableChannelSchema_;
  }
  OptimizableMaterialChannelRef optimizableChannel(
    std::string_view name) const
  {
    return OptimizableMaterialChannelRef(
      optimizableField_, optimizableChannelSchema_,
      optimizableChannelSchema_.channelIndex(name));
  }

private:
  std::shared_ptr<const ModelDefinition> definition_;
  std::shared_ptr<const FixedParameterField> fixedField_;
  std::shared_ptr<const OptimizableParameterField> optimizableField_;
  MaterialChannelSchema fixedChannelSchema_;
  MaterialChannelSchema optimizableChannelSchema_;
};

using ElasticParameterization =
  MaterialDomainParameterization<ElasticModelDefinition>;
using PlasticParameterization =
  MaterialDomainParameterization<PlasticModelDefinition>;

class MaterialParameterization final
{
public:
  MaterialParameterization(
    ElasticParameterization elastic,
    PlasticParameterization plastic):
    elastic_(std::move(elastic)), plastic_(std::move(plastic))
  {
    if (elastic_.optimizableField()->layout().numElements() !=
      plastic_.optimizableField()->layout().numElements())
      throw std::invalid_argument(
        "MaterialParameterization elastic and plastic fields must share an element count.");
    if (elastic_.fixedField()->numElements() !=
      plastic_.fixedField()->numElements())
      throw std::invalid_argument(
        "MaterialParameterization fixed fields must share an element count.");
  }

  const ElasticParameterization &elastic() const { return elastic_; }
  const PlasticParameterization &plastic() const { return plastic_; }
  int numElements() const
  {
    return elastic_.optimizableField()->layout().numElements();
  }

private:
  ElasticParameterization elastic_;
  PlasticParameterization plastic_;
};

}  // namespace pgo::SolidDeformationModel
