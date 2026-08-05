#pragma once

#include "material/frame/materialFrames.h"

#include <memory>
#include <span>
#include <string_view>

namespace pgo::SolidDeformationModel
{

class PlasticModel;

/// Immutable, shareable definition of one plastic constitutive model.
///
/// The definition owns the physical channel contract and constructs the
/// per-element constitutive model from fixed channels and the material frame.
class PlasticModelDefinition
{
public:
  virtual ~PlasticModelDefinition() = default;

  virtual std::string_view id() const = 0;
  virtual int numFixedChannels() const = 0;
  virtual int numOptimizableChannels() const = 0;
  virtual std::unique_ptr<PlasticModel> createModel(
    std::span<const double> fixedChannels,
    const MaterialFrame &materialFrame) const = 0;
};

}  // namespace pgo::SolidDeformationModel
