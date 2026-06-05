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

SampledPenaltyContactEnergy::SampledPenaltyContactEnergy(
  const Mesh::TriMeshGeo &surfaceMesh,
  EigenSupport::ConstRefVecXd simulationRestPositions,
  const ParametersSpec &params,
  std::vector<Mesh::TriMeshGeo> externalSurfaces,
  std::vector<int> vertexEmbeddingIndices,
  std::vector<double> vertexEmbeddingWeights):
  simulationRestPositions_(simulationRestPositions),
  params_(params),
  detector_(
    surfaceMesh,
    static_cast<int>(simulationRestPositions.size()),
    params,
    std::move(externalSurfaces),
    std::move(vertexEmbeddingIndices),
    std::move(vertexEmbeddingWeights))
{
  dofs_.resize(simulationRestPositions_.size());
  for (int i = 0; i < static_cast<int>(dofs_.size()); ++i)
    dofs_[i] = i;
}

SampledPenaltyContactEnergy::~SampledPenaltyContactEnergy() = default;

void SampledPenaltyContactEnergy::validateStateVector(EigenSupport::ConstRefVecXd x) const
{
  if (x.size() != simulationRestPositions_.size())
    throw std::invalid_argument("SampledPenaltyContactEnergy state vector has unexpected size.");
}

void SampledPenaltyContactEnergy::prepareActiveSet(EigenSupport::ConstRefVecXd x) const
{
  validateStateVector(x);
  activeSetCache_.prepareExact(
    x,
    [this](EigenSupport::ConstRefVecXd state) { return buildActiveSet(state); });
}

void SampledPenaltyContactEnergy::clearPreparedActiveSet() const
{
  activeSetCache_.clearExact();
}

void SampledPenaltyContactEnergy::resetActiveSets() const
{
  activeSetCache_.clearAll();
}

void SampledPenaltyContactEnergy::updateExternalSurface(int index, const Mesh::TriMeshGeo &surface)
{
  detector_.updateExternalSurface(index, surface);
  resetActiveSets();
}

void SampledPenaltyContactEnergy::beginActiveSetLineSearch(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd) const
{
  validateStateVector(x);
  activeSetCache_.beginLineSearch(
    x,
    [this](EigenSupport::ConstRefVecXd state) { return buildActiveSet(state); });
}

void SampledPenaltyContactEnergy::endActiveSetLineSearch() const
{
  activeSetCache_.endLineSearch();
}

const SampledPenaltyActiveSet &SampledPenaltyContactEnergy::evaluationActiveSet(EigenSupport::ConstRefVecXd x) const
{
  validateStateVector(x);
  return activeSetCache_.forEvaluation(
    x,
    [this](EigenSupport::ConstRefVecXd state) { return buildActiveSet(state); });
}

std::unique_ptr<SampledPenaltyActiveSet> SampledPenaltyContactEnergy::buildActiveSet(EigenSupport::ConstRefVecXd x) const
{
  SampledPenaltyActiveEnergyConfigurator configurator;
  configurator.configureExternal = [this](PointPenetrationEnergy &energy) {
    configureExternalActiveEnergy(energy);
  };
  configurator.configureSelf = [this](PointTrianglePairCouplingEnergyWithCollision &energy, EigenSupport::ConstRefVecXd state) {
    configureSelfActiveEnergy(energy, state);
  };
  return detector_.buildActiveSet(x, configurator);
}

void SampledPenaltyContactEnergy::configureExternalActiveEnergy(PointPenetrationEnergy &energy) const
{
  energy.setComputePosFunction([this](const EigenSupport::V3d &u, EigenSupport::V3d &p, int dofStart) {
    p = u + simulationRestPositions_.segment<3>(dofStart);
  });
  energy.setComputeLastPosFunction([](const EigenSupport::V3d &u, EigenSupport::V3d &p, int) {
    p = u;
  });
  energy.setCoeff(params_.stiffness);
  energy.setFrictionCoeff(0.0);
  energy.setTimestep(0.0);
  energy.setVelEps(0.0);
  if (frictionState_)
    frictionState_->configureExternal(energy, simulationRestPositions_);
}

void SampledPenaltyContactEnergy::configureSelfActiveEnergy(PointTrianglePairCouplingEnergyWithCollision &energy, EigenSupport::ConstRefVecXd x) const
{
  energy.setToPosFunction([this](const EigenSupport::V3d &u, EigenSupport::V3d &p, int dofStart) {
    p = u + simulationRestPositions_.segment<3>(dofStart);
  });
  energy.setToLastPosFunction([](const EigenSupport::V3d &u, EigenSupport::V3d &p, int) {
    p = u;
  });
  energy.setCoeff(params_.stiffness);
  energy.setFrictionCoeff(0.0);
  energy.setTimestep(0.0);
  energy.setVelEps(0.0);
  if (frictionState_)
    frictionState_->configureSelf(energy, simulationRestPositions_);
  energy.computeClosestPosition(x.data());
}

double SampledPenaltyContactEnergy::func(EigenSupport::ConstRefVecXd x) const
{
  const SampledPenaltyActiveSet &activeSet = evaluationActiveSet(x);
  double value = 0.0;
  if (activeSet.externalEnergy)
    value += activeSet.externalEnergy->func(x);
  if (activeSet.selfEnergy)
    value += activeSet.selfEnergy->func(x);
  return value;
}

void SampledPenaltyContactEnergy::gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const
{
  if (grad.size() != simulationRestPositions_.size())
    throw std::invalid_argument("SampledPenaltyContactEnergy gradient vector has unexpected size.");

  const SampledPenaltyActiveSet &activeSet = evaluationActiveSet(x);
  grad.setZero();
  EigenSupport::VXd childGrad(grad.size());
  if (activeSet.externalEnergy) {
    childGrad.setZero();
    activeSet.externalEnergy->gradient(x, childGrad);
    grad += childGrad;
  }
  if (activeSet.selfEnergy) {
    childGrad.setZero();
    activeSet.selfEnergy->gradient(x, childGrad);
    grad += childGrad;
  }
}

void SampledPenaltyContactEnergy::hessianAlloc(EigenSupport::SpMatD &hess) const
{
  hess.resize(getNumDOFs(), getNumDOFs());
  hess.setZero();
}

void SampledPenaltyContactEnergy::hessian(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const
{
  hessianInPlace(x, hess);
}

void SampledPenaltyContactEnergy::hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const
{
  const SampledPenaltyActiveSet &activeSet = evaluationActiveSet(x);
  hess.resize(getNumDOFs(), getNumDOFs());
  hess.setZero();

  EigenSupport::SpMatD childHess;
  if (activeSet.externalEnergy) {
    activeSet.externalEnergy->hessian(x, childHess);
    addSparse(hess, childHess);
  }
  if (activeSet.selfEnergy) {
    activeSet.selfEnergy->hessian(x, childHess);
    addSparse(hess, childHess);
  }
}

void SampledPenaltyContactEnergy::getDOFs(std::vector<int> &dofs) const
{
  dofs = dofs_;
}

FrictionalSampledPenaltyContactEnergy::FrictionalSampledPenaltyContactEnergy(
  const Mesh::TriMeshGeo &surfaceMesh,
  EigenSupport::ConstRefVecXd simulationRestPositions,
  const ParametersSpec &params,
  const FrictionParametersSpec &frictionParams,
  std::vector<Mesh::TriMeshGeo> externalSurfaces,
  std::vector<int> vertexEmbeddingIndices,
  std::vector<double> vertexEmbeddingWeights):
  SampledPenaltyContactEnergy(
    surfaceMesh, simulationRestPositions, params, std::move(externalSurfaces),
    std::move(vertexEmbeddingIndices), std::move(vertexEmbeddingWeights))
{
  frictionState_.emplace(frictionParams);
}

void FrictionalSampledPenaltyContactEnergy::beginStep(const NonlinearOptimization::StepState &state)
{
  frictionState_->beginStep(state, getNumDOFs());
  resetActiveSets();
}

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
