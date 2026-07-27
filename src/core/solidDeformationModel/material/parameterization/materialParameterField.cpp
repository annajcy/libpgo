#include "material/parameterization/materialParameterField.h"

#include <stdexcept>
#include <vector>

namespace pgo::SolidDeformationModel
{

FixedParameterField::FixedParameterField(
  ParameterInputSchema inputSchema,
  std::shared_ptr<const ParameterLayout> layout,
  std::shared_ptr<const MaterialChannelMapping> mapping):
  inputSchema_(std::move(inputSchema)),
  layout_(std::move(layout)),
  mapping_(std::move(mapping))
{
  if (!layout_ || !mapping_)
    throw std::invalid_argument(
      "FixedParameterField requires a layout and mapping.");
  if (layout_->numLocalParameters() != inputSchema_.numParameters())
    throw std::invalid_argument(
      "FixedParameterField parameter schema does not match its layout.");
  if (mapping_->numInputs() != layout_->numLocalParameters())
    throw std::invalid_argument(
      "FixedParameterField mapping parameter count does not match its layout.");
}

void FixedParameterField::evaluate(
  int element,
  int quadrature,
  std::span<const double> parameterValues,
  std::span<double> materialChannels) const
{
  if (parameterValues.size() !=
    static_cast<std::size_t>(layout_->numGlobalParameters()))
    throw std::invalid_argument(
      "FixedParameterField parameter value count does not match its layout.");
  if (materialChannels.size() !=
    static_cast<std::size_t>(mapping_->numChannels()))
    throw std::invalid_argument(
      "FixedParameterField material channel count does not match its mapping.");
  std::vector<double> local(
    static_cast<std::size_t>(layout_->numLocalParameters()));
  layout_->gather(element, parameterValues, local);
  mapping_->evaluate(element, quadrature, local, materialChannels);
}

OptimizableParameterField::OptimizableParameterField(
  ParameterInputSchema inputSchema,
  std::shared_ptr<const ParameterLayout> layout,
  std::shared_ptr<const DifferentiableMaterialChannelMapping> mapping)
{
  if (!layout)
    throw std::invalid_argument(
      "OptimizableParameterField requires a parameter layout.");
  if (!mapping)
    throw std::invalid_argument(
      "OptimizableParameterField requires a differentiable mapping.");
  if (mapping->numInputs() != layout->numLocalParameters())
    throw std::invalid_argument(
      "OptimizableParameterField mapping parameter count does not match its layout.");
  if (inputSchema.numParameters() != layout->numLocalParameters())
    throw std::invalid_argument(
      "OptimizableParameterField parameter schema does not match its layout.");
  state_ = std::make_shared<const State>(State{
    std::move(inputSchema), std::move(layout), std::move(mapping) });
}

}  // namespace pgo::SolidDeformationModel
