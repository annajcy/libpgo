/*
copyright to Bohan Wang
*/

#include "ipc/ipcContactEnergy.h"

#include "ipc/profiling/surfaceIPCProfiling.h"
#include "scopedProfileSection.h"

#include <stdexcept>
#include <string>
#include <utility>

namespace pgo
{
namespace Contact
{
namespace IPC
{

IPCContactEnergy::IPCContactEnergy(
  const EigenSupport::MXd &surfaceRestVertices,
  const EigenSupport::MXi &surfaceTriangles,
  const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
  const SurfaceIPCCore::Parameters &ipcParams,
  std::vector<std::unique_ptr<ObstacleSurface>> obstacleSurfaces):
  MappedSurfacePotentialEnergy(surfaceRestVertices, surfaceFromSimulationDispMap),
  surfaceIPCCore_(ipcParams, std::move(obstacleSurfaces))
{
  if (surfaceTriangles.cols() != 3)
    throw std::invalid_argument("surfaceTriangles must be an M x 3 triangle index matrix.");
  if (surfaceTriangles.size() > 0) {
    if (surfaceTriangles.minCoeff() < 0 || surfaceTriangles.maxCoeff() >= surfaceRestVertices.rows()) {
      throw std::invalid_argument("surfaceTriangles contains an out-of-range vertex index.");
    }
  }

  surfaceIPCCore_.setMesh(surfaceRestVertices, surfaceTriangles);
  surfaceIPCCore_.setMovingObstacleTime(0.0);
}

void IPCContactEnergy::cacheEnergyActiveSet(SurfaceIPCActiveSet activeSet) const
{
  cachedEnergyActiveSet_ = std::move(activeSet);
  hasCachedEnergyActiveSet_ = true;
}

const SurfaceIPCActiveSet *IPCContactEnergy::cachedEnergyActiveSetFor(
  EigenSupport::ConstRefVecXd surfacePositions) const
{
  if (!hasCachedEnergyActiveSet_)
    return nullptr;
  if (cachedEnergyActiveSet_.positions.size() != surfacePositions.size())
    return nullptr;
  if (!(cachedEnergyActiveSet_.positions.array() == surfacePositions.array()).all())
    return nullptr;
  return &cachedEnergyActiveSet_;
}

const SurfaceIPCActiveSet &IPCContactEnergy::evaluationActiveSetFor(
  EigenSupport::ConstRefVecXd surfacePositions,
  const char *reason) const
{
  if (hasLineSearchActiveSet_) {
    lineSearchActiveSet_.positions = surfacePositions;
    return lineSearchActiveSet_;
  }

  if (const SurfaceIPCActiveSet *cachedActiveSet = cachedEnergyActiveSetFor(surfacePositions))
    return *cachedActiveSet;

  throw std::logic_error(
    std::string("IPCContactEnergy requires refreshActiveSet(x) or prepareEvaluationState(x) before ") +
    reason + " outside line search.");
}

void IPCContactEnergy::clearCachedEnergyActiveSet() const
{
  cachedEnergyActiveSet_.clear();
  hasCachedEnergyActiveSet_ = false;
}

double IPCContactEnergy::computeSurfaceEnergy(EigenSupport::ConstRefVecXd surfacePositions) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kEnergy);
  return surfaceIPCCore_.computeEnergy(evaluationActiveSetFor(surfacePositions, "value evaluation"));
}

void IPCContactEnergy::computeSurfaceGradient(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::RefVecXd surfaceGradient) const
{
  if (const SurfaceIPCActiveSet *cachedActiveSet = cachedEnergyActiveSetFor(surfacePositions)) {
    surfaceIPCCore_.computeGradient(*cachedActiveSet, surfaceGradient);
    return;
  }

  surfaceIPCCore_.computeGradient(evaluationActiveSetFor(surfacePositions, "gradient evaluation"), surfaceGradient);
}

void IPCContactEnergy::computeSurfaceHessian(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::SpMatD &surfaceHessian) const
{
  if (const SurfaceIPCActiveSet *cachedActiveSet = cachedEnergyActiveSetFor(surfacePositions)) {
    surfaceIPCCore_.computeHessian(*cachedActiveSet, surfaceHessian);
    return;
  }

  surfaceIPCCore_.computeHessian(evaluationActiveSetFor(surfacePositions, "hessian evaluation"), surfaceHessian);
}

void IPCContactEnergy::computeSurfaceGradHessian(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::RefVecXd surfaceGradient,
  EigenSupport::SpMatD &surfaceHessian) const
{
  if (const SurfaceIPCActiveSet *cachedActiveSet = cachedEnergyActiveSetFor(surfacePositions)) {
    surfaceIPCCore_.computeGradient(*cachedActiveSet, surfaceGradient);
    surfaceIPCCore_.computeHessian(*cachedActiveSet, surfaceHessian);
    return;
  }

  const SurfaceIPCActiveSet &activeSet = evaluationActiveSetFor(surfacePositions, "gradient/hessian evaluation");
  surfaceIPCCore_.computeGradient(activeSet, surfaceGradient);
  surfaceIPCCore_.computeHessian(activeSet, surfaceHessian);
}

void IPCContactEnergy::computeSurfaceFuncGrad(
  EigenSupport::ConstRefVecXd surfacePositions,
  double &surfaceEnergy,
  EigenSupport::RefVecXd surfaceGradient) const
{
  if (const SurfaceIPCActiveSet *cachedActiveSet = cachedEnergyActiveSetFor(surfacePositions)) {
    surfaceEnergy = surfaceIPCCore_.computeEnergy(*cachedActiveSet);
    surfaceIPCCore_.computeGradient(*cachedActiveSet, surfaceGradient);
    return;
  }

  const SurfaceIPCActiveSet &activeSet = evaluationActiveSetFor(surfacePositions, "value/gradient evaluation");
  surfaceEnergy = surfaceIPCCore_.computeEnergy(activeSet);
  surfaceIPCCore_.computeGradient(activeSet, surfaceGradient);
}

void IPCContactEnergy::computeSurfaceAll(
  EigenSupport::ConstRefVecXd surfacePositions,
  double &surfaceEnergy,
  EigenSupport::RefVecXd surfaceGradient,
  EigenSupport::SpMatD &surfaceHessian) const
{
  if (const SurfaceIPCActiveSet *cachedActiveSet = cachedEnergyActiveSetFor(surfacePositions)) {
    EigenSupport::VXd localGradient = EigenSupport::VXd::Zero(surfaceGradient.size());
    surfaceIPCCore_.computeAll(*cachedActiveSet, surfaceEnergy, localGradient, surfaceHessian);
    surfaceGradient = localGradient;
    return;
  }

  const SurfaceIPCActiveSet &activeSet = evaluationActiveSetFor(surfacePositions, "value/gradient/hessian evaluation");
  EigenSupport::VXd localGradient = EigenSupport::VXd::Zero(surfaceGradient.size());
  surfaceIPCCore_.computeAll(activeSet, surfaceEnergy, localGradient, surfaceHessian);
  surfaceGradient = localGradient;
}

NonlinearOptimization::StepConstraint IPCContactEnergy::computeSurfaceMaxStepLimit(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::ConstRefVecXd surfaceDisplacements,
  NonlinearOptimization::StepConstraintSink *sink) const
{
  return surfaceIPCCore_.computeMaxStepLimit(surfacePositions, surfaceDisplacements, sink);
}

void IPCContactEnergy::beginSurfaceLineSearch(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::ConstRefVecXd surfaceDisplacements) const
{
  clearCachedEnergyActiveSet();
  lineSearchActiveSet_ = surfaceIPCCore_.buildLineSearchActiveSetSuperset(surfacePositions, surfaceDisplacements);
  hasLineSearchActiveSet_ = true;
}

void IPCContactEnergy::endSurfaceLineSearch() const
{
  lineSearchActiveSet_.clear();
  hasLineSearchActiveSet_ = false;
}

void IPCContactEnergy::beginStep(const NonlinearOptimization::StepState &state)
{
  setMovingObstacleTime(state.time + state.timestep);
}

void IPCContactEnergy::refreshActiveSet(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  cacheEnergyActiveSet(surfaceIPCCore_.buildActiveSet(
    computeSurfacePositionsFromSimulationDisplacements(simulationDisplacements)));
}

void IPCContactEnergy::clearActiveSet() const
{
  clearCachedEnergyActiveSet();
  lineSearchActiveSet_.clear();
  hasLineSearchActiveSet_ = false;
}

void IPCContactEnergy::setMovingObstacleTime(double t)
{
  clearActiveSet();
  surfaceIPCCore_.setMovingObstacleTime(t);
}

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
