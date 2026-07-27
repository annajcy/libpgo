#pragma once

#include "material/frame/materialFrameField.h"
#include "material/model/materialSchema.h"

#include <memory>
#include <span>
#include <string_view>

namespace pgo::SolidDeformationModel
{

class PlasticModel;

/// Immutable, shareable definition of one plastic constitutive model.
///
/// The definition owns the physical channel contract and constructs the
/// per-element evaluator from fixed channels and the material frame.
class PlasticModelDefinition
{
public:
  virtual ~PlasticModelDefinition() = default;

  virtual std::string_view id() const = 0;
  virtual MaterialChannelSchema fixedChannelSchema() const = 0;
  virtual MaterialChannelSchema optimizableChannelSchema() const = 0;
  virtual std::unique_ptr<PlasticModel> createModel(
    std::span<const double> fixedChannels,
    const MaterialFrame &materialFrame) const = 0;
};

}  // namespace pgo::SolidDeformationModel
