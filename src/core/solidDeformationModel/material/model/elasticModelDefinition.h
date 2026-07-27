#pragma once

#include "material/frame/materialFrameField.h"
#include "material/model/materialSchema.h"

#include <memory>
#include <span>
#include <string_view>

namespace pgo::SolidDeformationModel
{

class ElasticModel;

/// Immutable, shareable definition of one elastic constitutive model.
///
/// The definition owns the physical channel contract and constructs the
/// per-element evaluator from fixed channels and the material frame.
class ElasticModelDefinition
{
public:
  virtual ~ElasticModelDefinition() = default;

  virtual std::string_view id() const = 0;
  virtual MaterialChannelSchema fixedChannelSchema() const = 0;
  virtual MaterialChannelSchema optimizableChannelSchema() const = 0;
  virtual std::unique_ptr<ElasticModel> createModel(
    std::span<const double> fixedChannels,
    const MaterialFrame &materialFrame) const = 0;
};

}  // namespace pgo::SolidDeformationModel
