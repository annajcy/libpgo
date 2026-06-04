/*
  Long-lived sampled penalty contact energies.
*/

#include "sampled_penalty/sampledPenaltyContactEnergy.h"

#include "sampled_penalty/kernels/pointPenetrationEnergy.h"
#include "sampled_penalty/kernels/pointTrianglePairCouplingEnergyWithCollision.h"
#include "sampled_penalty/kernels/triangleMeshExternalContactHandler.h"
#include "sampled_penalty/kernels/triangleMeshSelfContactHandler.h"

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
std::vector<Mesh::TriMeshRef> makeSurfaceRefs(std::vector<Mesh::TriMeshGeo> &surfaces)
{
  std::vector<Mesh::TriMeshRef> refs;
  refs.reserve(surfaces.size());
  for (auto &surface : surfaces)
    refs.emplace_back(surface);
  return refs;
}

void addSparse(EigenSupport::SpMatD &dst, const EigenSupport::SpMatD &src)
{
  if (src.nonZeros() == 0)
    return;
  dst = dst + src;
}
}  // namespace

struct SampledPenaltyActiveSet
{
  std::shared_ptr<PointPenetrationEnergy> externalEnergy;
  PointPenetrationEnergyBuffer *externalBuffer = nullptr;
  std::shared_ptr<PointTrianglePairCouplingEnergyWithCollision> selfEnergy;
  PointTrianglePairCouplingEnergyWithCollisionBuffer *selfBuffer = nullptr;

  ~SampledPenaltyActiveSet() { clear(); }

  bool empty() const
  {
    return !externalEnergy && !selfEnergy;
  }

  void clear()
  {
    if (externalEnergy && externalBuffer) {
      externalEnergy->freeBuffer(externalBuffer);
      externalEnergy->setBuffer(nullptr);
    }
    externalBuffer = nullptr;
    externalEnergy.reset();

    if (selfEnergy && selfBuffer) {
      selfEnergy->freeBuffer(selfBuffer);
      selfEnergy->setBuffer(nullptr);
    }
    selfBuffer = nullptr;
    selfEnergy.reset();
  }
};

SampledPenaltyContactEnergy::SampledPenaltyContactEnergy(
  const Mesh::TriMeshGeo &surfaceMesh,
  EigenSupport::ConstRefVecXd simulationRestPositions,
  const ParametersSpec &params,
  std::vector<Mesh::TriMeshGeo> externalSurfaces,
  std::vector<int> vertexEmbeddingIndices,
  std::vector<double> vertexEmbeddingWeights):
  surfaceMesh_(surfaceMesh),
  simulationRestPositions_(simulationRestPositions),
  params_(params),
  externalSurfaces_(std::move(externalSurfaces)),
  vertexEmbeddingIndices_(std::move(vertexEmbeddingIndices)),
  vertexEmbeddingWeights_(std::move(vertexEmbeddingWeights))
{
  if (simulationRestPositions_.size() <= 0 || simulationRestPositions_.size() % 3 != 0)
    throw std::invalid_argument("SampledPenaltyContactEnergy requires simulation rest positions with size 3*n.");
  if (params_.samples <= 0)
    throw std::invalid_argument("SampledPenaltyContactEnergy requires a positive sample count.");
  if (params_.stiffness < 0.0)
    throw std::invalid_argument("SampledPenaltyContactEnergy requires non-negative stiffness.");
  if (vertexEmbeddingIndices_.empty() != vertexEmbeddingWeights_.empty())
    throw std::invalid_argument("SampledPenaltyContactEnergy embedding indices and weights must be provided together.");
  if (!vertexEmbeddingIndices_.empty() && vertexEmbeddingIndices_.size() != vertexEmbeddingWeights_.size())
    throw std::invalid_argument("SampledPenaltyContactEnergy embedding indices and weights must have matching sizes.");
  if (vertexEmbeddingIndices_.empty() && surfaceMesh_.numVertices() * 3 > simulationRestPositions_.size())
    throw std::invalid_argument("SampledPenaltyContactEnergy surface has more vertex DOFs than the unembedded simulation state.");

  dofs_.resize(simulationRestPositions_.size());
  for (int i = 0; i < static_cast<int>(dofs_.size()); ++i)
    dofs_[i] = i;

  const std::vector<int> *embeddingIndices = vertexEmbeddingIndices_.empty() ? nullptr : &vertexEmbeddingIndices_;
  const std::vector<double> *embeddingWeights = vertexEmbeddingWeights_.empty() ? nullptr : &vertexEmbeddingWeights_;
  const int simulationDofCount = static_cast<int>(simulationRestPositions_.size());

  if (params_.enableExternalContact && !externalSurfaces_.empty()) {
    std::vector<Mesh::TriMeshRef> externalRefs = makeSurfaceRefs(externalSurfaces_);
    externalHandler_ = std::make_shared<TriangleMeshExternalContactHandler>(
      surfaceMesh_.positions(), surfaceMesh_.triangles(), simulationDofCount,
      externalRefs, params_.samples, embeddingIndices, embeddingWeights);
  }

  if (params_.enableSelfContact) {
    selfHandler_ = std::make_shared<TriangleMeshSelfContactHandler>(
      surfaceMesh_.positions(), surfaceMesh_.triangles(), simulationDofCount,
      params_.samples, embeddingIndices, embeddingWeights);
  }
}

SampledPenaltyContactEnergy::~SampledPenaltyContactEnergy() = default;

void SampledPenaltyContactEnergy::validateStateVector(EigenSupport::ConstRefVecXd x) const
{
  if (x.size() != simulationRestPositions_.size())
    throw std::invalid_argument("SampledPenaltyContactEnergy state vector has unexpected size.");
}

void SampledPenaltyContactEnergy::beginStep(const NonlinearOptimization::StepState &)
{
  clearActiveSet();
}

void SampledPenaltyContactEnergy::refreshActiveSet(EigenSupport::ConstRefVecXd x) const
{
  validateStateVector(x);
  activeSet_ = buildActiveSet(x);
}

void SampledPenaltyContactEnergy::clearActiveSet() const
{
  activeSet_.reset();
  lineSearchActiveSet_.reset();
}

void SampledPenaltyContactEnergy::updateExternalSurface(int index, const Mesh::TriMeshGeo &surface)
{
  if (index < 0 || index >= static_cast<int>(externalSurfaces_.size()))
    throw std::invalid_argument("SampledPenaltyContactEnergy external surface index is out of range.");

  externalSurfaces_[index] = surface;
  if (externalHandler_)
    externalHandler_->updateExternalSurface(index, Mesh::TriMeshRef(externalSurfaces_[index]));
  clearActiveSet();
}

void SampledPenaltyContactEnergy::beginLineSearch(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd) const
{
  validateStateVector(x);
  lineSearchActiveSet_ = buildActiveSet(x);
}

void SampledPenaltyContactEnergy::endLineSearch() const
{
  lineSearchActiveSet_.reset();
}

const SampledPenaltyActiveSet &SampledPenaltyContactEnergy::evaluationActiveSet(EigenSupport::ConstRefVecXd x, const char *reason) const
{
  validateStateVector(x);
  if (lineSearchActiveSet_)
    return *lineSearchActiveSet_;
  if (!activeSet_) {
    throw std::logic_error(
      std::string("SampledPenaltyContactEnergy requires refreshActiveSet(x) or prepareEvaluationState(x) before ") +
      reason + " outside line search.");
  }
  return *activeSet_;
}

std::unique_ptr<SampledPenaltyActiveSet> SampledPenaltyContactEnergy::buildActiveSet(EigenSupport::ConstRefVecXd x) const
{
  auto activeSet = std::make_unique<SampledPenaltyActiveSet>();

  if (params_.stiffness <= 0.0)
    return activeSet;

  if (externalHandler_) {
    externalHandler_->execute(x.data());
    if (externalHandler_->getNumCollidingSamples() > 0) {
      activeSet->externalEnergy = externalHandler_->buildContactEnergy();
      activeSet->externalBuffer = activeSet->externalEnergy->allocateBuffer();
      activeSet->externalEnergy->setBuffer(activeSet->externalBuffer);
      configureExternalActiveEnergy(*activeSet->externalEnergy);
    }
  }

  if (selfHandler_) {
    selfHandler_->execute(x.data());
    if (!selfHandler_->getCollidingTrianglePair().empty()) {
      selfHandler_->handleContactDCD(0.0, 100);
      activeSet->selfEnergy = selfHandler_->buildContactEnergy();
      activeSet->selfBuffer = activeSet->selfEnergy->allocateBuffer();
      activeSet->selfEnergy->setBuffer(activeSet->selfBuffer);
      configureSelfActiveEnergy(*activeSet->selfEnergy, x);
    }
  }

  return activeSet;
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
  energy.computeClosestPosition(x.data());
}

double SampledPenaltyContactEnergy::func(EigenSupport::ConstRefVecXd x) const
{
  const SampledPenaltyActiveSet &activeSet = evaluationActiveSet(x, "value evaluation");
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

  const SampledPenaltyActiveSet &activeSet = evaluationActiveSet(x, "gradient evaluation");
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
  const SampledPenaltyActiveSet &activeSet = evaluationActiveSet(x, "hessian evaluation");
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
    std::move(vertexEmbeddingIndices), std::move(vertexEmbeddingWeights)),
  frictionParams_(frictionParams)
{
  if (frictionParams_.frictionCoeff < 0.0)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy requires non-negative friction coefficient.");
  if (frictionParams_.velocityEps <= 0.0)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy requires positive velocity epsilon.");
}

void FrictionalSampledPenaltyContactEnergy::beginStep(const NonlinearOptimization::StepState &state)
{
  if (state.previousX == nullptr)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy::beginStep requires previousX.");
  if (state.timestep <= 0.0)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy::beginStep requires a positive timestep.");
  if (state.previousX->size() != simulationRestPositions_.size())
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy::beginStep previousX has unexpected size.");

  previousX_ = *state.previousX;
  timestep_ = state.timestep;
  hasStepState_ = true;
  SampledPenaltyContactEnergy::beginStep(state);
}

void FrictionalSampledPenaltyContactEnergy::configureExternalActiveEnergy(PointPenetrationEnergy &energy) const
{
  if (!hasStepState_)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy requires beginStep before refreshing active contact.");

  SampledPenaltyContactEnergy::configureExternalActiveEnergy(energy);
  energy.setComputeLastPosFunction([this](const EigenSupport::V3d &, EigenSupport::V3d &p, int dofStart) {
    p = previousX_.segment<3>(dofStart) + simulationRestPositions_.segment<3>(dofStart);
  });
  energy.setFrictionCoeff(frictionParams_.frictionCoeff);
  energy.setTimestep(timestep_);
  energy.setVelEps(frictionParams_.velocityEps);
}

void FrictionalSampledPenaltyContactEnergy::configureSelfActiveEnergy(PointTrianglePairCouplingEnergyWithCollision &energy, EigenSupport::ConstRefVecXd x) const
{
  if (!hasStepState_)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy requires beginStep before refreshing active contact.");

  SampledPenaltyContactEnergy::configureSelfActiveEnergy(energy, x);
  energy.setToLastPosFunction([this](const EigenSupport::V3d &, EigenSupport::V3d &p, int dofStart) {
    p = previousX_.segment<3>(dofStart) + simulationRestPositions_.segment<3>(dofStart);
  });
  energy.setFrictionCoeff(frictionParams_.frictionCoeff);
  energy.setTimestep(timestep_);
  energy.setVelEps(frictionParams_.velocityEps);
}

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
