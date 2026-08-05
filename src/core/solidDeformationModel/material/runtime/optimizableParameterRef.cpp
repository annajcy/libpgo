#include "material/runtime/optimizableParameterRef.h"

#include <stdexcept>
#include <vector>

namespace pgo::SolidDeformationModel
{

OptimizableParameterRef::OptimizableParameterRef(
  std::shared_ptr<const OptimizableParameterField> field,
  int parameterIndex):
  field_(std::move(field)),
  parameterIndex_(parameterIndex)
{
  if (!field_)
    throw std::invalid_argument("OptimizableParameterRef requires a field.");
  if (parameterIndex < 0 ||
    parameterIndex >= field_->inputSchema().numParameters())
    throw std::out_of_range(
      "OptimizableParameterRef parameter index is out of range.");
}

OptimizableParameterRef::OptimizableParameterRef(
  std::shared_ptr<const OptimizableParameterField> field,
  std::string_view parameterName):
  OptimizableParameterRef(
    field, field ? field->inputSchema().parameterIndex(parameterName) : -1)
{
}

const OptimizableParameterField &OptimizableParameterRef::field() const
{
  if (!field_)
    throw std::logic_error("OptimizableParameterRef is empty.");
  return *field_;
}

std::string_view OptimizableParameterRef::name() const
{
  const auto names = field().inputSchema().parameterNames();
  return names[static_cast<std::size_t>(parameterIndex_)];
}

double OptimizableParameterRef::value(
  int element,
  int quadrature,
  const MaterialStateView &state) const
{
  MaterialStateEvaluationScratch scratch;
  return value(element, quadrature, state, scratch);
}

double OptimizableParameterRef::value(
  int element,
  int quadrature,
  const MaterialStateView &state,
  MaterialStateEvaluationScratch &scratch) const
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
  const MaterialStateView &state,
  EigenSupport::RefVecXd output) const
{
  MaterialStateEvaluationScratch scratch;
  localDerivative(element, quadrature, state, scratch, output);
}

void OptimizableParameterRef::localDerivative(
  int element,
  int quadrature,
  const MaterialStateView &state,
  MaterialStateEvaluationScratch &scratch,
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
  std::string_view channelName):
  field_(std::move(field)),
  channelName_(channelName)
{
  if (!field_)
    throw std::invalid_argument(
      "OptimizableMaterialChannelRef requires a field.");
  channelIndex_ = outputSchema.channelIndex(channelName_);
  if (field_->mapping().numChannels() != outputSchema.numChannels())
    throw std::invalid_argument(
      "OptimizableMaterialChannelRef output schema does not match mapping.");
}

const OptimizableParameterField &OptimizableMaterialChannelRef::field() const
{
  if (!field_)
    throw std::logic_error("OptimizableMaterialChannelRef is empty.");
  return *field_;
}

std::string_view OptimizableMaterialChannelRef::name() const
{
  return channelName_;
}

double OptimizableMaterialChannelRef::value(
  int element,
  int quadrature,
  const MaterialStateView &state) const
{
  MaterialStateEvaluationScratch scratch;
  return value(element, quadrature, state, scratch);
}

double OptimizableMaterialChannelRef::value(
  int element,
  int quadrature,
  const MaterialStateView &state,
  MaterialStateEvaluationScratch &scratch) const
{
  const OptimizableParameterField &f = field();
  scratch.prepare(f);
  f.layout().gather(element, state.values(f), scratch.local);
  f.mapping().evaluate(element, quadrature, scratch.local, scratch.material);
  return scratch.material[static_cast<std::size_t>(channelIndex_)];
}

void OptimizableMaterialChannelRef::localDerivative(
  int element,
  int quadrature,
  const MaterialStateView &state,
  MaterialStateEvaluationScratch &scratch,
  EigenSupport::RefVecXd output) const
{
  const OptimizableParameterField &f = field();
  if (output.size() != f.layout().numLocalParameters())
    throw std::invalid_argument(
      "OptimizableMaterialChannelRef derivative output has the wrong size.");
  scratch.prepare(f);
  f.layout().gather(element, state.values(f), scratch.local);
  f.mapping().evaluateJacobian(
    element, quadrature, scratch.local, scratch.jacobian);
  output = scratch.jacobian.row(channelIndex_).transpose();
}

void OptimizableMaterialChannelRef::localHessian(
  int element,
  int quadrature,
  const MaterialStateView &state,
  EigenSupport::RefMatXd output) const
{
  const OptimizableParameterField &f = field();
  const int numParameters = f.layout().numLocalParameters();
  if (output.rows() != numParameters || output.cols() != numParameters)
    throw std::invalid_argument(
      "OptimizableMaterialChannelRef Hessian output has the wrong shape.");

  MaterialStateEvaluationScratch scratch;
  scratch.prepare(f);
  f.layout().gather(element, state.values(f), scratch.local);
  std::vector<EigenSupport::MXd> channelHessians(
    static_cast<std::size_t>(f.mapping().numChannels()),
    EigenSupport::MXd(numParameters, numParameters));
  f.mapping().evaluateHessians(
    element, quadrature, scratch.local, channelHessians);
  output = channelHessians[static_cast<std::size_t>(channelIndex_)];
}

}  // namespace pgo::SolidDeformationModel
