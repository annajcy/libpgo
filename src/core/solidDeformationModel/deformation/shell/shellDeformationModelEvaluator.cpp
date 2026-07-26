#include "shellDeformationModelEvaluator.h"

#include "shellDeformationModel.h"

namespace pgo
{
namespace SolidDeformationModel
{

ShellDeformationModelEvaluator::ShellDeformationModelEvaluator(
  const ShellDeformationModel &model):
  model_(model),
  state_(model.getNumPlasticParameters(), model.getNumElasticParameters())
{
}

void ShellDeformationModelEvaluator::prepare(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams)
{
  markUnprepared();
  model_.prepareData(x, elasticParams, plasticParams, state_);
  markPrepared();
}

double ShellDeformationModelEvaluator::compute_E() const
{
  ensurePrepared();
  return model_.compute_E(state_);
}

void ShellDeformationModelEvaluator::compute_dE_dx(
  EigenSupport::RefVecXd grad) const
{
  ensurePrepared();
  model_.compute_dE_dx(state_, grad);
}

void ShellDeformationModelEvaluator::compute_d2E_dx2(
  EigenSupport::RefMatXd hess) const
{
  ensurePrepared();
  model_.compute_d2E_dx2(state_, hess);
}

void ShellDeformationModelEvaluator::compute_d2E_dudp(
  EigenSupport::RefMatXd hess, int materialLocation) const
{
  ensurePrepared();
  model_.compute_d2E_dudp(state_, hess, materialLocation);
}

void ShellDeformationModelEvaluator::compute_d2E_dude(
  EigenSupport::RefMatXd hess, int materialLocation) const
{
  ensurePrepared();
  model_.compute_d2E_dude(state_, hess, materialLocation);
}

void ShellDeformationModelEvaluator::compute_dE_dp(
  EigenSupport::RefVecXd grad, int materialLocation) const
{
  ensurePrepared();
  model_.compute_dE_dp(state_, grad, materialLocation);
}

void ShellDeformationModelEvaluator::compute_d2E_dp2(
  EigenSupport::RefMatXd hess, int materialLocation) const
{
  ensurePrepared();
  model_.compute_d2E_dp2(state_, hess, materialLocation);
}

void ShellDeformationModelEvaluator::compute_dE_de(
  EigenSupport::RefVecXd grad, int materialLocation) const
{
  ensurePrepared();
  model_.compute_dE_de(state_, grad, materialLocation);
}

void ShellDeformationModelEvaluator::compute_d2E_de2(
  EigenSupport::RefMatXd hess, int materialLocation) const
{
  ensurePrepared();
  model_.compute_d2E_de2(state_, hess, materialLocation);
}

void ShellDeformationModelEvaluator::compute_d2E_dpde(
  EigenSupport::RefMatXd hess, int materialLocation) const
{
  ensurePrepared();
  model_.compute_d2E_dpde(state_, hess, materialLocation);
}

int ShellDeformationModelEvaluator::computeVonMisesStress(
  std::span<double> stresses, int capacity) const
{
  ensurePrepared();
  return model_.computeVonMisesStress(state_, stresses, capacity);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
