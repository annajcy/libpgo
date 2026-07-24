#pragma once

#include "material/fields/materialParameters.h"
#include "material/fields/materialFrameField.h"

#include <memory>
#include <span>
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
  virtual int getNumParameters() const { return 0; }
  virtual void enableSPD(int /*enable*/) {}
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
  virtual void initializeDefaultParameters(
    const SimulationMesh &, int, std::span<double>) const = 0;

private:
  friend class DeformationModelManager;
  virtual std::unique_ptr<ElasticModel> createModel(
    const SimulationMesh &, int, const MaterialFrame &) const = 0;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
