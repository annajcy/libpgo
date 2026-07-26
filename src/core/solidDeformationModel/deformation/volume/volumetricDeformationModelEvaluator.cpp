#include "volumetricDeformationModelEvaluator.h"

#include "volumetricDeformationModel.h"

namespace ES = pgo::EigenSupport;

namespace pgo
{
namespace SolidDeformationModel
{

VolumetricDeformationModelEvaluator::VolumetricDeformationModelEvaluator(
  const VolumetricDeformationModel &model):
  model_(model),
  state_(model.getNumVertices(), model.getNumMaterialLocations(),
    model.getNumPlasticParameters(), model.getNumElasticParameters())
{
}

void VolumetricDeformationModelEvaluator::prepare(
  std::span<const double> x, std::span<const double> elasticParams,
  std::span<const double> plasticParams)
{
  markUnprepared();
  model_.prepareData(x, elasticParams, plasticParams, state_);
  markPrepared();
}

double VolumetricDeformationModelEvaluator::compute_E() const
{
  ensurePrepared();
  return model_.compute_E(state_);
}

void VolumetricDeformationModelEvaluator::compute_dE_dx(
  EigenSupport::RefVecXd grad) const
{
  ensurePrepared();
  model_.compute_dE_dx(state_, grad);
}

void VolumetricDeformationModelEvaluator::compute_d2E_dx2(
  EigenSupport::RefMatXd hess) const
{
  ensurePrepared();
  model_.compute_d2E_dx2(state_, hess);
}

void VolumetricDeformationModelEvaluator::compute_d2E_dudp(
  EigenSupport::RefMatXd hess, int materialLocation) const
{
  ensurePrepared();
  model_.compute_d2E_dudp(state_, hess, materialLocation);
}

void VolumetricDeformationModelEvaluator::compute_d2E_dude(
  EigenSupport::RefMatXd hess, int materialLocation) const
{
  ensurePrepared();
  model_.compute_d2E_dude(state_, hess, materialLocation);
}

void VolumetricDeformationModelEvaluator::compute_dE_dp(
  EigenSupport::RefVecXd grad, int materialLocation) const
{
  ensurePrepared();
  model_.compute_dE_dp(state_, grad, materialLocation);
}

void VolumetricDeformationModelEvaluator::compute_d2E_dp2(
  EigenSupport::RefMatXd hess, int materialLocation) const
{
  ensurePrepared();
  model_.compute_d2E_dp2(state_, hess, materialLocation);
}

void VolumetricDeformationModelEvaluator::compute_dE_de(
  EigenSupport::RefVecXd grad, int materialLocation) const
{
  ensurePrepared();
  model_.compute_dE_de(state_, grad, materialLocation);
}

void VolumetricDeformationModelEvaluator::compute_d2E_de2(
  EigenSupport::RefMatXd hess, int materialLocation) const
{
  ensurePrepared();
  model_.compute_d2E_de2(state_, hess, materialLocation);
}

void VolumetricDeformationModelEvaluator::compute_d2E_dpde(
  EigenSupport::RefMatXd hess, int materialLocation) const
{
  ensurePrepared();
  model_.compute_d2E_dpde(state_, hess, materialLocation);
}

int VolumetricDeformationModelEvaluator::computeVonMisesStress(
  std::span<double> stresses, int capacity) const
{
  ensurePrepared();
  return model_.computeVonMisesStress(state_, stresses, capacity);
}

int VolumetricDeformationModelEvaluator::computeMaxStrain(
  std::span<double> strains, int capacity) const
{
  ensurePrepared();
  return model_.computeMaxStrain(state_, strains, capacity);
}

ES::M3d VolumetricDeformationModelEvaluator::compute_Fe(
  int materialLocationID) const
{
  ensurePrepared();
  return model_.compute_Fe(state_, materialLocationID);
}

ES::M3d VolumetricDeformationModelEvaluator::compute_P(
  int materialLocationID) const
{
  ensurePrepared();
  return model_.compute_P(state_, materialLocationID);
}

ES::M9d VolumetricDeformationModelEvaluator::compute_dP_dF(
  int materialLocationID) const
{
  ensurePrepared();
  return model_.compute_dP_dF(state_, materialLocationID);
}

void VolumetricDeformationModelEvaluator::compute_dF_dx(
  int materialLocationID, EigenSupport::RefMatXd dFdxOut) const
{
  ensurePrepared();
  model_.compute_dF_dx(state_, materialLocationID, dFdxOut);
}

void VolumetricDeformationModelEvaluator::computeForceFromP(
  int materialLocationID, const ES::M3d &P, EigenSupport::RefVecXd f) const
{
  ensurePrepared();
  model_.computeForceFromP(state_, materialLocationID, P, f);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
