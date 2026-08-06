#pragma once

#include "material/frame/materialFrames.h"

#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace pgo::SolidDeformationModel
{

/// Validate that a fixed-channel span matches the definition's channel count.
///
/// createModel implementations call this instead of hand-rolling the same
/// size check and error message in every material definition.
inline void requireFixedChannels(
  std::span<const double> fixedChannels,
  int expected,
  std::string_view modelId,
  std::string_view channelNames = {})
{
  if (fixedChannels.size() == static_cast<std::size_t>(expected))
    return;

  if (expected == 0) {
    throw std::invalid_argument(
      std::string(modelId) + " has no fixed channels");
  }

  std::string message = std::string(modelId) + " requires ";
  if (channelNames.empty())
    message += std::to_string(expected) + " fixed channels";
  else
    message += "fixed channels " + std::string(channelNames);
  throw std::invalid_argument(std::move(message));
}

/// Immutable, shareable definition of one constitutive model.
///
/// The definition owns the physical channel contract and constructs the
/// per-element constitutive model from fixed channels and the material frame.
template<class Model>
class ModelDefinition
{
public:
  virtual ~ModelDefinition() = default;

  virtual std::string_view id() const = 0;
  virtual int numFixedChannels() const = 0;
  virtual int numOptimizableChannels() const = 0;
  virtual std::unique_ptr<Model> createModel(
    std::span<const double> fixedChannels,
    const MaterialFrame &materialFrame) const = 0;
};

}  // namespace pgo::SolidDeformationModel
