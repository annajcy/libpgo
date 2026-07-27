#pragma once

#include "material/core/materialSchema.h"
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

class ElasticModel
{
public:
  ElasticModel() {}

  virtual ~ElasticModel() {}
  // Every evaluator must declare its parameter dimension.  A missing
  // declaration must fail at compile time instead of silently creating a
  // zero-dimensional parameter block.
  virtual int getNumParameters() const = 0;
};

// Immutable, shareable definition used to instantiate per-element ElasticModel
// evaluators. The private construction hook keeps evaluator data internal.
class ElasticModelDefinition
{
public:
  virtual ~ElasticModelDefinition() = default;
  virtual std::string_view id() const = 0;
  /// Physical channels supplied outside the optimizer.
  virtual MaterialChannelSchema fixedChannelSchema() const = 0;
  /// Physical channels driven by optimizer inputs.
  virtual MaterialChannelSchema optimizableChannelSchema() const = 0;
  virtual MaterialFrameRequirement frameRequirement() const = 0;

private:
  friend class DeformationModelManager;
  virtual std::unique_ptr<ElasticModel> createModelFromFixed(
    std::span<const double>, const MaterialFrame &) const = 0;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
