#pragma once

#include "material/parameterization/materialChannelMapping.h"
#include "material/model/materialSchema.h"
#include "material/parameterization/parameterLayout.h"
#include "EigenSupport.h"

#include <memory>
#include <span>
#include <string_view>

namespace pgo::SolidDeformationModel
{

/// Immutable fixed parameterization structure. Numerical values are supplied
/// separately by MaterialParameterData.
class FixedParameterField final
{
public:
  FixedParameterField(
    ParameterInputSchema inputSchema,
    std::shared_ptr<const ParameterLayout> layout,
    std::shared_ptr<const MaterialChannelMapping> mapping);

  const ParameterInputSchema &inputSchema() const { return inputSchema_; }
  const ParameterLayout &layout() const { return *layout_; }
  const MaterialChannelMapping &mapping() const { return *mapping_; }
  const std::shared_ptr<const ParameterLayout> &layoutHandle() const
  {
    return layout_;
  }
  const std::shared_ptr<const MaterialChannelMapping> &mappingHandle() const
  {
    return mapping_;
  }

  int numElements() const { return layout_->numElements(); }
  int numLocalParameters() const { return inputSchema_.numParameters(); }
  int numGlobalParameters() const { return layout_->numGlobalParameters(); }
  int numMaterialChannels() const { return mapping_->numChannels(); }

  void evaluate(
    int element,
    int quadrature,
    std::span<const double> parameterValues,
    std::span<double> materialChannels) const;

private:
  ParameterInputSchema inputSchema_;
  std::shared_ptr<const ParameterLayout> layout_;
  std::shared_ptr<const MaterialChannelMapping> mapping_;
};

/// Immutable schema for one elastic or plastic optimizable parameter field.
/// Copies share the same immutable state and therefore preserve field identity.
class OptimizableParameterField final
{
public:
  OptimizableParameterField(
    ParameterInputSchema inputSchema,
    std::shared_ptr<const ParameterLayout> layout,
    std::shared_ptr<const DifferentiableMaterialChannelMapping> mapping);

  const ParameterInputSchema &inputSchema() const
  {
    return state_->inputSchema;
  }
  const ParameterLayout &layout() const { return *state_->layout; }
  const DifferentiableMaterialChannelMapping &mapping() const
  {
    return *state_->mapping;
  }
  const std::shared_ptr<const ParameterLayout> &layoutHandle() const
  {
    return state_->layout;
  }
  const std::shared_ptr<const DifferentiableMaterialChannelMapping> &
  mappingHandle() const
  {
    return state_->mapping;
  }

  int numElements() const { return layout().numElements(); }
  int numLocalParameters() const { return inputSchema().numParameters(); }
  int numGlobalParameters() const { return layout().numGlobalParameters(); }
  int numMaterialChannels() const { return mapping().numChannels(); }

  bool sharesStateWith(const OptimizableParameterField &other) const
  {
    return state_ == other.state_;
  }

private:
  struct State
  {
    ParameterInputSchema inputSchema;
    std::shared_ptr<const ParameterLayout> layout;
    std::shared_ptr<const DifferentiableMaterialChannelMapping> mapping;
  };

  std::shared_ptr<const State> state_;
};

}  // namespace pgo::SolidDeformationModel
