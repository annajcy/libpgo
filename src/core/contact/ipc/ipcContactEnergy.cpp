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

SurfaceIPCActiveSet IPCContactEnergy::buildExactActiveSet(EigenSupport::ConstRefVecXd surfacePositions) const
{
  return surfaceIPCCore_.buildActiveSet(surfacePositions);
}

const SurfaceIPCActiveSet &IPCContactEnergy::activeSetForEvaluation(EigenSupport::ConstRefVecXd surfacePositions) const
{
  return activeSetCache_.forEvaluation(
    surfacePositions,
    [this](EigenSupport::ConstRefVecXd x) { return buildExactActiveSet(x); });
}

double IPCContactEnergy::computeSurfaceEnergy(EigenSupport::ConstRefVecXd surfacePositions) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kEnergy);
  return surfaceIPCCore_.computeEnergy(activeSetForEvaluation(surfacePositions));
}

void IPCContactEnergy::computeSurfaceGradient(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::RefVecXd surfaceGradient) const
{
  const SurfaceIPCActiveSet &activeSet = activeSetForEvaluation(surfacePositions);
  surfaceIPCCore_.computeGradient(activeSet, surfaceGradient);
}

void IPCContactEnergy::computeSurfaceHessian(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::SpMatD &surfaceHessian) const
{
  const SurfaceIPCActiveSet &activeSet = activeSetForEvaluation(surfacePositions);
  surfaceIPCCore_.computeHessian(activeSet, surfaceHessian);
}

void IPCContactEnergy::computeSurfaceGradHessian(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::RefVecXd surfaceGradient,
  EigenSupport::SpMatD &surfaceHessian) const
{
  const SurfaceIPCActiveSet &activeSet = activeSetForEvaluation(surfacePositions);
  surfaceIPCCore_.computeGradient(activeSet, surfaceGradient);
  surfaceIPCCore_.computeHessian(activeSet, surfaceHessian);
}

void IPCContactEnergy::computeSurfaceFuncGrad(
  EigenSupport::ConstRefVecXd surfacePositions,
  double &surfaceEnergy,
  EigenSupport::RefVecXd surfaceGradient) const
{
  const SurfaceIPCActiveSet &activeSet = activeSetForEvaluation(surfacePositions);
  surfaceEnergy = surfaceIPCCore_.computeEnergy(activeSet);
  surfaceIPCCore_.computeGradient(activeSet, surfaceGradient);
}

void IPCContactEnergy::computeSurfaceAll(
  EigenSupport::ConstRefVecXd surfacePositions,
  double &surfaceEnergy,
  EigenSupport::RefVecXd surfaceGradient,
  EigenSupport::SpMatD &surfaceHessian) const
{
  const SurfaceIPCActiveSet &activeSet = activeSetForEvaluation(surfacePositions);
  EigenSupport::VXd localGradient = EigenSupport::VXd::Zero(surfaceGradient.size());
  surfaceIPCCore_.computeAll(activeSet, surfaceEnergy, localGradient, surfaceHessian);
  surfaceGradient = localGradient;
}

NonlinearOptimization::StepConstraint IPCContactEnergy::computeMaxStepLimit(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::ConstRefVecXd trialSimulationDisplacements,
  NonlinearOptimization::StepConstraintSink *sink) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterMaxStep);
  const VXd surfacePositions = computeSurfacePositionsFromSimulationDisplacements(simulationDisplacements);
  const VXd trialSurfaceDisplacements = computeSurfaceDisplacementsFromSimulationDisplacements(trialSimulationDisplacements);
  return surfaceIPCCore_.computeMaxStepLimit(surfacePositions, trialSurfaceDisplacements, sink);
}

void IPCContactEnergy::beginStep(const NonlinearOptimization::StepState &state)
{
  setMovingObstacleTime(state.time + state.timestep);
}

void IPCContactEnergy::prepareActiveSet(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  const VXd surfacePositions = computeSurfacePositionsFromSimulationDisplacements(simulationDisplacements);
  activeSetCache_.prepareExact(
    surfacePositions,
    [this](EigenSupport::ConstRefVecXd x) { return buildExactActiveSet(x); });
}

void IPCContactEnergy::clearPreparedActiveSet() const
{
  activeSetCache_.clearExact();
}

void IPCContactEnergy::beginActiveSetLineSearch(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::ConstRefVecXd trialSimulationDisplacements) const
{
  const VXd surfacePositions = computeSurfacePositionsFromSimulationDisplacements(simulationDisplacements);
  const VXd trialSurfaceDisplacements = computeSurfaceDisplacementsFromSimulationDisplacements(trialSimulationDisplacements);
  activeSetCache_.beginLineSearch(
    surfaceIPCCore_.buildLineSearchActiveSetSuperset(surfacePositions, trialSurfaceDisplacements));
}

void IPCContactEnergy::endActiveSetLineSearch() const
{
  activeSetCache_.endLineSearch();
}

void IPCContactEnergy::setMovingObstacleTime(double t)
{
  activeSetCache_.clearAll();
  surfaceIPCCore_.setMovingObstacleTime(t);
}

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
