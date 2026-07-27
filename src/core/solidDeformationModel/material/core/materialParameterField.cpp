#include "materialParameterField.h"

#include "optimizableParameters.h"

#include <stdexcept>
#include <vector>

namespace pgo::SolidDeformationModel
{

FixedParameterField::FixedParameterField(
  ParameterSchema parameterSchema,
  std::shared_ptr<const ParameterLayout> layout,
  std::shared_ptr<const MaterialEvaluator> evaluator):
  parameterSchema_(std::move(parameterSchema)),
  layout_(std::move(layout)),
  evaluator_(std::move(evaluator))
{
  if (!layout_ || !evaluator_)
    throw std::invalid_argument(
      "FixedParameterField requires a layout and evaluator.");
  if (layout_->numLocalParameters() != parameterSchema_.numParameters())
    throw std::invalid_argument(
      "FixedParameterField parameter schema does not match its layout.");
  if (evaluator_->numParameters() != layout_->numLocalParameters())
    throw std::invalid_argument(
      "FixedParameterField evaluator parameter count does not match its layout.");
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
    static_cast<std::size_t>(evaluator_->numChannels()))
    throw std::invalid_argument(
      "FixedParameterField material channel count does not match its evaluator.");
  std::vector<double> local(
    static_cast<std::size_t>(layout_->numLocalParameters()));
  layout_->gather(element, parameterValues, local);
  evaluator_->evaluate(element, quadrature, local, materialChannels);
}

OptimizableParameterField::OptimizableParameterField(
  ParameterSchema parameterSchema,
  std::shared_ptr<const ParameterLayout> layout,
  std::shared_ptr<const DifferentiableMaterialEvaluator> evaluator)
{
  if (!layout)
    throw std::invalid_argument(
      "OptimizableParameterField requires a parameter layout.");
  if (!evaluator)
    throw std::invalid_argument(
      "OptimizableParameterField requires a differentiable evaluator.");
  if (evaluator->numParameters() != layout->numLocalParameters())
    throw std::invalid_argument(
      "OptimizableParameterField evaluator parameter count does not match its layout.");
  if (parameterSchema.numParameters() != layout->numLocalParameters())
    throw std::invalid_argument(
      "OptimizableParameterField parameter schema does not match its layout.");
  state_ = std::make_shared<const State>(State{
    std::move(parameterSchema), std::move(layout), std::move(evaluator) });
}

OptimizableParameterRef OptimizableParameterField::parameter(
  std::string_view name) const
{
  return OptimizableParameterRef(
    std::make_shared<const OptimizableParameterField>(*this),
    parameterSchema().parameterIndex(name));
}

OptimizableParameterRef::OptimizableParameterRef(
  std::shared_ptr<const OptimizableParameterField> field,
  int parameterIndex):
  field_(std::move(field)),
  parameterIndex_(parameterIndex)
{
  if (!field_)
    throw std::invalid_argument("OptimizableParameterRef requires a field.");
  if (parameterIndex < 0 ||
    parameterIndex >= field_->parameterSchema().numParameters())
    throw std::out_of_range(
      "OptimizableParameterRef parameter index is out of range.");
}

const OptimizableParameterField &OptimizableParameterRef::field() const
{
  if (!field_)
    throw std::logic_error("OptimizableParameterRef is empty.");
  return *field_;
}

std::string_view OptimizableParameterRef::name() const
{
  const auto names = field().parameterSchema().parameterNames();
  return names[static_cast<std::size_t>(parameterIndex_)];
}

double OptimizableParameterRef::value(
  int element,
  int quadrature,
  const OptimizableParameterEvaluationView &state) const
{
  OptimizableParameterEvaluationScratch scratch;
  return value(element, quadrature, state, scratch);
}

double OptimizableParameterRef::value(
  int element,
  int quadrature,
  const OptimizableParameterEvaluationView &state,
  OptimizableParameterEvaluationScratch &scratch) const
{
  (void)quadrature;
  const OptimizableParameterField &f = field();
  const ParameterLayout &layout = f.layout();
  scratch.prepare(f);
  layout.gather(element, state.values(f), scratch.local);
  return scratch.local[static_cast<std::size_t>(parameterIndex_)];
}

void OptimizableParameterRef::localDerivative(
  int element,
  int quadrature,
  const OptimizableParameterEvaluationView &state,
  EigenSupport::RefVecXd output) const
{
  OptimizableParameterEvaluationScratch scratch;
  localDerivative(element, quadrature, state, scratch, output);
}

void OptimizableParameterRef::localDerivative(
  int element,
  int quadrature,
  const OptimizableParameterEvaluationView &state,
  OptimizableParameterEvaluationScratch &scratch,
  EigenSupport::RefVecXd output) const
{
  (void)quadrature;
  (void)scratch;
  const OptimizableParameterField &f = field();
  const ParameterLayout &layout = f.layout();
  if (element < 0 || element >= layout.numElements())
    throw std::out_of_range("Optimizable parameter element is out of range.");
  (void)state.values(f);
  if (output.size() != layout.numLocalParameters())
    throw std::invalid_argument(
      "OptimizableParameterRef derivative output has the wrong size.");
  output.setZero();
  output[parameterIndex_] = 1.0;
}

OptimizableMaterialChannelRef::OptimizableMaterialChannelRef(
  std::shared_ptr<const OptimizableParameterField> field,
  MaterialChannelSchema outputSchema,
  int channelIndex):
  field_(std::move(field)),
  outputSchema_(std::move(outputSchema)),
  channelIndex_(channelIndex)
{
  if (!field_)
    throw std::invalid_argument(
      "OptimizableMaterialChannelRef requires a field.");
  if (channelIndex_ < 0 ||
    channelIndex_ >= outputSchema_.numChannels())
    throw std::out_of_range(
      "OptimizableMaterialChannelRef channel index is out of range.");
  if (field_->evaluator().numChannels() != outputSchema_.numChannels())
    throw std::invalid_argument(
      "OptimizableMaterialChannelRef output schema does not match evaluator.");
}

const OptimizableParameterField &OptimizableMaterialChannelRef::field() const
{
  if (!field_)
    throw std::logic_error("OptimizableMaterialChannelRef is empty.");
  return *field_;
}

std::string_view OptimizableMaterialChannelRef::name() const
{
  const auto names = outputSchema_.channelNames();
  return names[static_cast<std::size_t>(channelIndex_)];
}

double OptimizableMaterialChannelRef::value(
  int element,
  int quadrature,
  const OptimizableParameterEvaluationView &state) const
{
  OptimizableParameterEvaluationScratch scratch;
  return value(element, quadrature, state, scratch);
}

double OptimizableMaterialChannelRef::value(
  int element,
  int quadrature,
  const OptimizableParameterEvaluationView &state,
  OptimizableParameterEvaluationScratch &scratch) const
{
  const OptimizableParameterField &f = field();
  scratch.prepare(f);
  f.layout().gather(element, state.values(f), scratch.local);
  f.evaluator().evaluate(element, quadrature, scratch.local, scratch.material);
  return scratch.material[static_cast<std::size_t>(channelIndex_)];
}

void OptimizableMaterialChannelRef::localDerivative(
  int element,
  int quadrature,
  const OptimizableParameterEvaluationView &state,
  OptimizableParameterEvaluationScratch &scratch,
  EigenSupport::RefVecXd output) const
{
  const OptimizableParameterField &f = field();
  if (output.size() != f.layout().numLocalParameters())
    throw std::invalid_argument(
      "OptimizableMaterialChannelRef derivative output has the wrong size.");
  scratch.prepare(f);
  f.layout().gather(element, state.values(f), scratch.local);
  f.evaluator().evaluateJacobian(
    element, quadrature, scratch.local, scratch.jacobian);
  output = scratch.jacobian.row(channelIndex_).transpose();
}

void OptimizableMaterialChannelRef::localHessian(
  int element,
  int quadrature,
  const OptimizableParameterEvaluationView &state,
  EigenSupport::RefMatXd output) const
{
  const OptimizableParameterField &f = field();
  const int numParameters = f.layout().numLocalParameters();
  if (output.rows() != numParameters || output.cols() != numParameters)
    throw std::invalid_argument(
      "OptimizableMaterialChannelRef Hessian output has the wrong shape.");

  OptimizableParameterEvaluationScratch scratch;
  scratch.prepare(f);
  f.layout().gather(element, state.values(f), scratch.local);
  std::vector<EigenSupport::MXd> channelHessians(
    static_cast<std::size_t>(f.evaluator().numChannels()),
    EigenSupport::MXd(numParameters, numParameters));
  f.evaluator().evaluateHessians(
    element, quadrature, scratch.local, channelHessians);
  output = channelHessians[static_cast<std::size_t>(channelIndex_)];
}

}  // namespace pgo::SolidDeformationModel
