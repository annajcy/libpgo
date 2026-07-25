#pragma once

#include "material/fields/materialParameters.h"
#include "material/fields/materialFrameField.h"

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
class ElasticModelConfig
{
public:
  virtual ~ElasticModelConfig() = default;
  virtual std::string_view id() const = 0;
  virtual MaterialParameterSpec parameterSpec() const = 0;
  virtual MaterialFrameRequirement frameRequirement() const = 0;
  virtual void initializeDefaultElementChannels(
    const SimulationMesh &, int, std::span<double>) const
  {
    throw std::logic_error("elastic config does not provide default element channels");
  }

private:
  friend class DeformationModelManager;
  virtual std::unique_ptr<ElasticModel> createModel(
    const SimulationMesh &, int, const MaterialFrame &) const = 0;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
