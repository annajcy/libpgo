#pragma once

#include "EigenDef.h"

namespace pgo
{
namespace Contact
{
namespace SampledPenalty
{

struct SampledPenaltyEvaluationBundle;

class SampledPenaltyContactEvaluator
{
public:
  double func(
    const SampledPenaltyEvaluationBundle &bundle,
    EigenSupport::ConstRefVecXd surfacePositions) const;
  void gradient(
    const SampledPenaltyEvaluationBundle &bundle,
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::RefVecXd surfaceGradient) const;
  void hessian(
    const SampledPenaltyEvaluationBundle &bundle,
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::SpMatD &surfaceHessian) const;
  double func_grad(
    const SampledPenaltyEvaluationBundle &bundle,
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::RefVecXd surfaceGradient) const;
  double func_grad_hessian(
    const SampledPenaltyEvaluationBundle &bundle,
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::RefVecXd surfaceGradient,
    EigenSupport::SpMatD &surfaceHessian) const;
  void gradient_hessian(
    const SampledPenaltyEvaluationBundle &bundle,
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::RefVecXd surfaceGradient,
    EigenSupport::SpMatD &surfaceHessian) const;
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
