#pragma once

#include "material/core/materialParameters.h"
#include "material/core/materialFrameField.h"

#include <memory>
#include <span>
#include <stdexcept>
#include <string_view>

namespace pgo
{
namespace SolidDeformationModel
{
class SimulationMesh;
class DeformationModelManager;

class PlasticModel
{
public:
  PlasticModel() {}
  virtual ~PlasticModel() {}

  // Every evaluator must declare its parameter dimension.  A missing
  // declaration must fail at compile time instead of silently creating a
  // zero-dimensional parameter block.
  virtual int getNumParameters() const = 0;

  // Write default (rest-configuration) plastic parameters into param.
  // Zero-parameter evaluator bases override this explicitly as a no-op.
  virtual void defaultParams(std::span<double> param) const = 0;
};

// Immutable, shareable definition used to instantiate per-element PlasticModel
// evaluators. The private construction hook keeps evaluator data internal.
class PlasticModelConfig
{
public:
  virtual ~PlasticModelConfig() = default;
  virtual std::string_view id() const = 0;
  virtual std::span<const std::string_view> parameterChannelNames() const = 0;
  virtual MaterialFrameRequirement frameRequirement() const = 0;
  virtual void initializeDefaultElementChannels(
    const SimulationMesh &, int, std::span<double>) const = 0;

private:
  friend class DeformationModelManager;
  virtual std::unique_ptr<PlasticModel> createModel(
    const SimulationMesh &, int, const MaterialFrame &) const = 0;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
