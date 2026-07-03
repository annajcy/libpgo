#include "sampled_penalty/sampledPenaltyContactEvaluator.h"

#include "sampled_penalty/sampledPenaltyEvaluationBundle.h"
#include "sampled_penalty/kernels/pointPenetrationEnergy.h"
#include "sampled_penalty/kernels/pointTrianglePairCouplingEnergyWithCollision.h"

#include <stdexcept>

namespace pgo
{
namespace Contact
{
namespace SampledPenalty
{
namespace
{
void validateGradientSize(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::RefVecXd surfaceGradient)
{
  if (surfaceGradient.size() != surfacePositions.size())
    throw std::invalid_argument("SampledPenaltyContactEvaluator gradient vector has unexpected size.");
}

void addSparse(EigenSupport::SpMatD &dst, const EigenSupport::SpMatD &src)
{
  if (src.nonZeros() == 0)
    return;
  dst = dst + src;
}
}  // namespace

double SampledPenaltyContactEvaluator::func(
  const SampledPenaltyEvaluationBundle &bundle,
  EigenSupport::ConstRefVecXd surfacePositions) const
{
  double value = 0.0;
  if (bundle.externalEnergy)
    value += bundle.externalEnergy->func(surfacePositions);
  if (bundle.selfEnergy)
    value += bundle.selfEnergy->func(surfacePositions);
  return value;
}

void SampledPenaltyContactEvaluator::gradient(
  const SampledPenaltyEvaluationBundle &bundle,
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::RefVecXd surfaceGradient) const
{
  validateGradientSize(surfacePositions, surfaceGradient);

  surfaceGradient.setZero();
  EigenSupport::VXd childGradient(surfaceGradient.size());
  if (bundle.externalEnergy) {
    childGradient.setZero();
    bundle.externalEnergy->gradient(surfacePositions, childGradient);
    surfaceGradient += childGradient;
  }
  if (bundle.selfEnergy) {
    childGradient.setZero();
    bundle.selfEnergy->gradient(surfacePositions, childGradient);
    surfaceGradient += childGradient;
  }
}

void SampledPenaltyContactEvaluator::hessian(
  const SampledPenaltyEvaluationBundle &bundle,
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::SpMatD &surfaceHessian) const
{
  surfaceHessian.resize(surfacePositions.size(), surfacePositions.size());
  surfaceHessian.setZero();

  EigenSupport::SpMatD childHessian;
  if (bundle.externalEnergy) {
    bundle.externalEnergy->hessian(surfacePositions, childHessian);
    addSparse(surfaceHessian, childHessian);
  }
  if (bundle.selfEnergy) {
    bundle.selfEnergy->hessian(surfacePositions, childHessian);
    addSparse(surfaceHessian, childHessian);
  }
}

double SampledPenaltyContactEvaluator::func_grad(
  const SampledPenaltyEvaluationBundle &bundle,
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::RefVecXd surfaceGradient) const
{
  const double value = func(bundle, surfacePositions);
  gradient(bundle, surfacePositions, surfaceGradient);
  return value;
}

double SampledPenaltyContactEvaluator::func_grad_hessian(
  const SampledPenaltyEvaluationBundle &bundle,
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::RefVecXd surfaceGradient,
  EigenSupport::SpMatD &surfaceHessian) const
{
  const double value = func_grad(bundle, surfacePositions, surfaceGradient);
  hessian(bundle, surfacePositions, surfaceHessian);
  return value;
}

void SampledPenaltyContactEvaluator::gradient_hessian(
  const SampledPenaltyEvaluationBundle &bundle,
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::RefVecXd surfaceGradient,
  EigenSupport::SpMatD &surfaceHessian) const
{
  gradient(bundle, surfacePositions, surfaceGradient);
  hessian(bundle, surfacePositions, surfaceHessian);
}

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
