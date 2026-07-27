#pragma once

#include "material/parameterization/materialParameterField.h"
#include "material/model/elasticModelDefinition.h"
#include "material/model/plasticModelDefinition.h"

#include <memory>
#include <stdexcept>
#include <algorithm>
#include <string_view>

namespace pgo::SolidDeformationModel
{

struct MaterialParameterData;

/// A material-domain parameterization: parameter values are gathered by the
/// field layouts and converted to physical channels by their mappings.
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
    optimizableField_(std::move(optimizableField))
  {
    if (!definition_ || !fixedField_ || !optimizableField_)
      throw std::invalid_argument(
        "MaterialDomainParameterization requires definition and fields.");
    const MaterialChannelSchema fixedChannelSchema =
      definition_->fixedChannelSchema();
    const MaterialChannelSchema optimizableChannelSchema =
      definition_->optimizableChannelSchema();
    if (fixedField_->numElements() != optimizableField_->numElements())
      throw std::invalid_argument(
        "MaterialDomainParameterization fields must share an element count.");
    if (fixedField_->numMaterialChannels() !=
      fixedChannelSchema.numChannels())
      throw std::invalid_argument(
        "MaterialDomainParameterization fixed channel schema does not match definition.");
    if (optimizableField_->mapping().numChannels() !=
      optimizableChannelSchema.numChannels())
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
  MaterialChannelSchema fixedChannelSchema() const
  {
    return definition_->fixedChannelSchema();
  }
  MaterialChannelSchema optimizableChannelSchema() const
  {
    return definition_->optimizableChannelSchema();
  }

private:
  std::shared_ptr<const ModelDefinition> definition_;
  std::shared_ptr<const FixedParameterField> fixedField_;
  std::shared_ptr<const OptimizableParameterField> optimizableField_;
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

  /// Validate numeric values against this immutable parameterization.
  void validate(const MaterialParameterData &data) const;

private:
  ElasticParameterization elastic_;
  PlasticParameterization plastic_;
};

}  // namespace pgo::SolidDeformationModel
