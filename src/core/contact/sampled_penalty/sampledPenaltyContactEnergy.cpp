/*
  Long-lived sampled penalty contact energies.
*/

#include "sampled_penalty/sampledPenaltyContactEnergy.h"

#include "sampled_penalty/sampledPenaltyActiveSet.h"
#include "sampled_penalty/kernels/pointPenetrationEnergy.h"
#include "sampled_penalty/kernels/pointTrianglePairCouplingEnergyWithCollision.h"

#include <stdexcept>
#include <string>

namespace pgo
{
namespace Contact
{
namespace SampledPenalty
{
namespace
{
void addSparse(EigenSupport::SpMatD &dst, const EigenSupport::SpMatD &src)
{
  if (src.nonZeros() == 0)
    return;
  dst = dst + src;
}
}  // namespace

SampledPenaltySurfaceContactEnergy::SampledPenaltySurfaceContactEnergy(
  const Mesh::TriMeshGeo &surfaceMesh,
  const ParametersSpec &params,
  std::vector<Mesh::TriMeshGeo> externalSurfaces):
  surfaceDofCount_(surfaceMesh.numVertices() * 3),
  params_(params),
  detector_(
    surfaceMesh,
    surfaceDofCount_,
    params,
    std::move(externalSurfaces),
    {},
    {})
{
  dofs_.resize(surfaceDofCount_);
  for (int i = 0; i < surfaceDofCount_; ++i)
    dofs_[i] = i;
}

SampledPenaltySurfaceContactEnergy::~SampledPenaltySurfaceContactEnergy() = default;

void SampledPenaltySurfaceContactEnergy::validateSurfacePositionVector(EigenSupport::ConstRefVecXd surfacePositions) const
{
  if (surfacePositions.size() != surfaceDofCount_)
    throw std::invalid_argument("SampledPenaltySurfaceContactEnergy surface position vector has unexpected size.");
}

void SampledPenaltySurfaceContactEnergy::prepareActiveSet(EigenSupport::ConstRefVecXd surfacePositions) const
{
  validateSurfacePositionVector(surfacePositions);
  activeSetCache_.prepareExact(
    surfacePositions,
    [this](EigenSupport::ConstRefVecXd state) { return buildActiveSet(state); });
}

void SampledPenaltySurfaceContactEnergy::clearPreparedActiveSet() const
{
  activeSetCache_.clearExact();
}

void SampledPenaltySurfaceContactEnergy::resetActiveSets() const
{
  activeSetCache_.clearAll();
}

void SampledPenaltySurfaceContactEnergy::updateExternalSurface(int index, const Mesh::TriMeshGeo &surface)
{
  detector_.updateExternalSurface(index, surface);
  resetActiveSets();
}

void SampledPenaltySurfaceContactEnergy::beginActiveSetLineSearch(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::ConstRefVecXd) const
{
  validateSurfacePositionVector(surfacePositions);
  activeSetCache_.beginLineSearch(
    surfacePositions,
    [this](EigenSupport::ConstRefVecXd state) { return buildActiveSet(state); });
}

void SampledPenaltySurfaceContactEnergy::endActiveSetLineSearch() const
{
  activeSetCache_.endLineSearch();
}

const SampledPenaltyActiveSet &SampledPenaltySurfaceContactEnergy::evaluationActiveSet(EigenSupport::ConstRefVecXd surfacePositions) const
{
  validateSurfacePositionVector(surfacePositions);
  return activeSetCache_.forEvaluation(
    surfacePositions,
    [this](EigenSupport::ConstRefVecXd state) { return buildActiveSet(state); });
}

std::unique_ptr<SampledPenaltyActiveSet> SampledPenaltySurfaceContactEnergy::buildActiveSet(EigenSupport::ConstRefVecXd surfacePositions) const
{
  SampledPenaltyActiveEnergyConfigurator configurator;
  configurator.configureExternal = [this](PointPenetrationEnergy &energy) {
    configureExternalActiveEnergy(energy);
  };
  configurator.configureSelf = [this](PointTrianglePairCouplingEnergyWithCollision &energy, EigenSupport::ConstRefVecXd state) {
    configureSelfActiveEnergy(energy, state);
  };
  return detector_.buildActiveSet(surfacePositions, configurator);
}

void SampledPenaltySurfaceContactEnergy::configureExternalActiveEnergy(PointPenetrationEnergy &energy) const
{
  energy.setComputePosFunction([](const EigenSupport::V3d &x, EigenSupport::V3d &p, int) {
    p = x;
  });
  energy.setComputeLastPosFunction([](const EigenSupport::V3d &x, EigenSupport::V3d &p, int) {
    p = x;
  });
  energy.setCoeff(params_.stiffness);
  energy.setFrictionCoeff(0.0);
  energy.setTimestep(0.0);
  energy.setVelEps(0.0);
  if (frictionState_)
    frictionState_->configureExternalSurfacePositions(energy);
}

void SampledPenaltySurfaceContactEnergy::configureSelfActiveEnergy(
  PointTrianglePairCouplingEnergyWithCollision &energy,
  EigenSupport::ConstRefVecXd surfacePositions) const
{
  energy.setToPosFunction([](const EigenSupport::V3d &x, EigenSupport::V3d &p, int) {
    p = x;
  });
  energy.setToLastPosFunction([](const EigenSupport::V3d &x, EigenSupport::V3d &p, int) {
    p = x;
  });
  energy.setCoeff(params_.stiffness);
  energy.setFrictionCoeff(0.0);
  energy.setTimestep(0.0);
  energy.setVelEps(0.0);
  if (frictionState_)
    frictionState_->configureSelfSurfacePositions(energy);
  energy.computeClosestPosition(surfacePositions.data());
}

double SampledPenaltySurfaceContactEnergy::func(EigenSupport::ConstRefVecXd surfacePositions) const
{
  const SampledPenaltyActiveSet &activeSet = evaluationActiveSet(surfacePositions);
  double value = 0.0;
  if (activeSet.externalEnergy)
    value += activeSet.externalEnergy->func(surfacePositions);
  if (activeSet.selfEnergy)
    value += activeSet.selfEnergy->func(surfacePositions);
  return value;
}

void SampledPenaltySurfaceContactEnergy::gradient(EigenSupport::ConstRefVecXd surfacePositions, EigenSupport::RefVecXd grad) const
{
  if (grad.size() != surfaceDofCount_)
    throw std::invalid_argument("SampledPenaltySurfaceContactEnergy gradient vector has unexpected size.");

  const SampledPenaltyActiveSet &activeSet = evaluationActiveSet(surfacePositions);
  grad.setZero();
  EigenSupport::VXd childGrad(grad.size());
  if (activeSet.externalEnergy) {
    childGrad.setZero();
    activeSet.externalEnergy->gradient(surfacePositions, childGrad);
    grad += childGrad;
  }
  if (activeSet.selfEnergy) {
    childGrad.setZero();
    activeSet.selfEnergy->gradient(surfacePositions, childGrad);
    grad += childGrad;
  }
}

void SampledPenaltySurfaceContactEnergy::hessianAlloc(EigenSupport::SpMatD &hess) const
{
  hess.resize(getNumDOFs(), getNumDOFs());
  hess.setZero();
}

void SampledPenaltySurfaceContactEnergy::hessian(EigenSupport::ConstRefVecXd surfacePositions, EigenSupport::SpMatD &hess) const
{
  hessianInPlace(surfacePositions, hess);
}

void SampledPenaltySurfaceContactEnergy::hessianInPlace(EigenSupport::ConstRefVecXd surfacePositions, EigenSupport::SpMatD &hess) const
{
  const SampledPenaltyActiveSet &activeSet = evaluationActiveSet(surfacePositions);
  hess.resize(getNumDOFs(), getNumDOFs());
  hess.setZero();

  EigenSupport::SpMatD childHess;
  if (activeSet.externalEnergy) {
    activeSet.externalEnergy->hessian(surfacePositions, childHess);
    addSparse(hess, childHess);
  }
  if (activeSet.selfEnergy) {
    activeSet.selfEnergy->hessian(surfacePositions, childHess);
    addSparse(hess, childHess);
  }
}

void SampledPenaltySurfaceContactEnergy::getDOFs(std::vector<int> &dofs) const
{
  dofs = dofs_;
}

FrictionalSampledPenaltySurfaceContactEnergy::FrictionalSampledPenaltySurfaceContactEnergy(
  const Mesh::TriMeshGeo &surfaceMesh,
  const ParametersSpec &params,
  const FrictionParametersSpec &frictionParams,
  std::vector<Mesh::TriMeshGeo> externalSurfaces):
  SampledPenaltySurfaceContactEnergy(surfaceMesh, params, std::move(externalSurfaces))
{
  frictionState_.emplace(frictionParams);
}

void FrictionalSampledPenaltySurfaceContactEnergy::beginStep(const NonlinearOptimization::StepState &state)
{
  frictionState_->beginStep(state, getNumDOFs());
  resetActiveSets();
}

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
