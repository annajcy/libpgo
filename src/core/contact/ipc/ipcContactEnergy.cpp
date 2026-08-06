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
  const IPCPairGenerator::Parameters &pairParams,
  const IPCContactAssembler::Parameters &assemblerParams,
  std::vector<std::unique_ptr<ObstacleSurface>> obstacleSurfaces):
  dofMap_(surfaceRestVertices, surfaceFromSimulationDispMap),
  pairGenerator_(pairParams, std::move(obstacleSurfaces)),
  assembler_(assemblerParams)
{
  if (surfaceTriangles.cols() != 3)
    throw std::invalid_argument("surfaceTriangles must be an M x 3 triangle index matrix.");
  if (surfaceTriangles.size() > 0) {
    if (surfaceTriangles.minCoeff() < 0 || surfaceTriangles.maxCoeff() >= surfaceRestVertices.rows()) {
      throw std::invalid_argument("surfaceTriangles contains an out-of-range vertex index.");
    }
  }

  pairGenerator_.setMesh(surfaceRestVertices, surfaceTriangles);
  pairGenerator_.setMovingObstacleTime(0.0);
}

SurfaceIPCActiveSet IPCContactEnergy::buildExactActiveSet(EigenSupport::ConstRefVecXd surfacePositions) const
{
  return pairGenerator_.buildActiveSet(surfacePositions);
}

const SurfaceIPCActiveSet &IPCContactEnergy::activeSetForEvaluation(EigenSupport::ConstRefVecXd surfacePositions) const
{
  return activeSetCache_.forEvaluation(
    surfacePositions,
    [this](EigenSupport::ConstRefVecXd x) { return buildExactActiveSet(x); });
}

double IPCContactEnergy::func(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterFunc);
  EigenSupport::VXd surfacePositions;
  {
    Profiling::ScopedProfileSection mapProfile(SurfaceIPCProfileSections::kAdapterMapToSurface);
    surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  }

  Profiling::ScopedProfileSection energyProfile(SurfaceIPCProfileSections::kEnergy);
  const SurfaceIPCActiveSet &activeSet = activeSetForEvaluation(surfacePositions);
  const std::vector<ObstacleSurfaceView> obstacleViews = pairGenerator_.obstacleViews();
  return assembler_.computeEnergy(pairGenerator_.topology(), obstacleViews, activeSet);
}

void IPCContactEnergy::gradient(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterGradient);
  EigenSupport::VXd surfacePositions;
  {
    Profiling::ScopedProfileSection mapProfile(SurfaceIPCProfileSections::kAdapterMapToSurface);
    surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  }

  EigenSupport::VXd surfaceGradient = EigenSupport::VXd::Zero(dofMap_.numSurfaceDofs());
  const SurfaceIPCActiveSet &activeSet = activeSetForEvaluation(surfacePositions);
  const std::vector<ObstacleSurfaceView> obstacleViews = pairGenerator_.obstacleViews();
  assembler_.computeGradient(pairGenerator_.topology(), obstacleViews, activeSet, surfaceGradient);

  Profiling::ScopedProfileSection pullbackProfile(SurfaceIPCProfileSections::kAdapterPullbackGradient);
  simulationGradient = dofMap_.pullbackGradient(surfaceGradient);
}

void IPCContactEnergy::hessian(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::SpMatD &simulationHessian) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterHessianDirect);
  EigenSupport::VXd surfacePositions;
  {
    Profiling::ScopedProfileSection mapProfile(SurfaceIPCProfileSections::kAdapterMapToSurface);
    surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  }

  EigenSupport::SpMatD surfaceHessian(dofMap_.numSurfaceDofs(), dofMap_.numSurfaceDofs());
  const SurfaceIPCActiveSet &activeSet = activeSetForEvaluation(surfacePositions);
  const std::vector<ObstacleSurfaceView> obstacleViews = pairGenerator_.obstacleViews();
  assembler_.computeHessian(pairGenerator_.topology(), obstacleViews, activeSet, surfaceHessian);

  Profiling::ScopedProfileSection pullbackProfile(SurfaceIPCProfileSections::kAdapterPullbackHessian);
  dofMap_.pullbackHessian(surfaceHessian, simulationHessian);
}

void IPCContactEnergy::hessianInPlace(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::SpMatD &simulationHessian) const
{
  hessian(simulationDisplacements, simulationHessian);
}

void IPCContactEnergy::hessianAlloc(EigenSupport::SpMatD &simulationHessian) const
{
  simulationHessian.resize(getNumDOFs(), getNumDOFs());
  simulationHessian.setZero();
}

double IPCContactEnergy::funcGradient(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterGradient);
  EigenSupport::VXd surfacePositions;
  {
    Profiling::ScopedProfileSection mapProfile(SurfaceIPCProfileSections::kAdapterMapToSurface);
    surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  }

  EigenSupport::VXd surfaceGradient = EigenSupport::VXd::Zero(dofMap_.numSurfaceDofs());
  const SurfaceIPCActiveSet &activeSet = activeSetForEvaluation(surfacePositions);
  const std::vector<ObstacleSurfaceView> obstacleViews = pairGenerator_.obstacleViews();
  const double surfaceEnergy = assembler_.computeEnergy(pairGenerator_.topology(), obstacleViews, activeSet);
  assembler_.computeGradient(pairGenerator_.topology(), obstacleViews, activeSet, surfaceGradient);

  Profiling::ScopedProfileSection pullbackProfile(SurfaceIPCProfileSections::kAdapterPullbackGradient);
  simulationGradient = dofMap_.pullbackGradient(surfaceGradient);
  return surfaceEnergy;
}

double IPCContactEnergy::funcGradientHessian(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient,
  EigenSupport::SpMatD &simulationHessian) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterHessianDirect);
  EigenSupport::VXd surfacePositions;
  {
    Profiling::ScopedProfileSection mapProfile(SurfaceIPCProfileSections::kAdapterMapToSurface);
    surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  }

  double surfaceEnergy = 0.0;
  EigenSupport::VXd surfaceGradient = EigenSupport::VXd::Zero(dofMap_.numSurfaceDofs());
  EigenSupport::SpMatD surfaceHessian(dofMap_.numSurfaceDofs(), dofMap_.numSurfaceDofs());
  const SurfaceIPCActiveSet &activeSet = activeSetForEvaluation(surfacePositions);
  const std::vector<ObstacleSurfaceView> obstacleViews = pairGenerator_.obstacleViews();
  assembler_.computeAll(pairGenerator_.topology(), obstacleViews, activeSet, surfaceEnergy, surfaceGradient, surfaceHessian);

  {
    Profiling::ScopedProfileSection pullbackProfile(SurfaceIPCProfileSections::kAdapterPullbackGradient);
    simulationGradient = dofMap_.pullbackGradient(surfaceGradient);
  }
  {
    Profiling::ScopedProfileSection pullbackProfile(SurfaceIPCProfileSections::kAdapterPullbackHessian);
    dofMap_.pullbackHessian(surfaceHessian, simulationHessian);
  }
  return surfaceEnergy;
}

void IPCContactEnergy::gradientHessian(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient,
  EigenSupport::SpMatD &simulationHessian) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterHessianDirect);
  EigenSupport::VXd surfacePositions;
  {
    Profiling::ScopedProfileSection mapProfile(SurfaceIPCProfileSections::kAdapterMapToSurface);
    surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  }

  EigenSupport::VXd surfaceGradient = EigenSupport::VXd::Zero(dofMap_.numSurfaceDofs());
  EigenSupport::SpMatD surfaceHessian(dofMap_.numSurfaceDofs(), dofMap_.numSurfaceDofs());
  const SurfaceIPCActiveSet &activeSet = activeSetForEvaluation(surfacePositions);
  const std::vector<ObstacleSurfaceView> obstacleViews = pairGenerator_.obstacleViews();
  assembler_.computeGradient(pairGenerator_.topology(), obstacleViews, activeSet, surfaceGradient);
  assembler_.computeHessian(pairGenerator_.topology(), obstacleViews, activeSet, surfaceHessian);

  {
    Profiling::ScopedProfileSection pullbackProfile(SurfaceIPCProfileSections::kAdapterPullbackGradient);
    simulationGradient = dofMap_.pullbackGradient(surfaceGradient);
  }
  {
    Profiling::ScopedProfileSection pullbackProfile(SurfaceIPCProfileSections::kAdapterPullbackHessian);
    dofMap_.pullbackHessian(surfaceHessian, simulationHessian);
  }
}

void IPCContactEnergy::getDOFs(std::vector<int> &dofs) const
{
  dofs = dofMap_.simulationDofs();
}

int IPCContactEnergy::getNumDOFs() const
{
  return dofMap_.numSimulationDofs();
}

NonlinearOptimization::StepConstraint IPCContactEnergy::computeMaxStepLimit(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::ConstRefVecXd trialSimulationDisplacements,
  NonlinearOptimization::StepConstraintSink *sink) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterMaxStep);
  EigenSupport::VXd surfacePositions;
  EigenSupport::VXd trialSurfaceDisplacements;
  {
    Profiling::ScopedProfileSection mapProfile(SurfaceIPCProfileSections::kAdapterMapToSurface);
    surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  }
  {
    Profiling::ScopedProfileSection mapProfile(SurfaceIPCProfileSections::kAdapterMapToSurface);
    trialSurfaceDisplacements = dofMap_.surfaceDisplacements(trialSimulationDisplacements);
  }
  return pairGenerator_.computeMaxStepLimit(surfacePositions, trialSurfaceDisplacements, sink);
}

void IPCContactEnergy::beginStep(const NonlinearOptimization::StepState &state)
{
  setMovingObstacleTime(state.time + state.timestep);
}

void IPCContactEnergy::beginLineSearch(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::ConstRefVecXd trialSimulationDisplacements) const
{
  EigenSupport::VXd surfacePositions;
  EigenSupport::VXd trialSurfaceDisplacements;
  {
    Profiling::ScopedProfileSection mapProfile(SurfaceIPCProfileSections::kAdapterMapToSurface);
    surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  }
  {
    Profiling::ScopedProfileSection mapProfile(SurfaceIPCProfileSections::kAdapterMapToSurface);
    trialSurfaceDisplacements = dofMap_.surfaceDisplacements(trialSimulationDisplacements);
  }
  activeSetCache_.beginLineSearch(
    pairGenerator_.buildLineSearchActiveSetSuperset(surfacePositions, trialSurfaceDisplacements));
}

void IPCContactEnergy::endLineSearch() const
{
  activeSetCache_.endLineSearch();
}

void IPCContactEnergy::setMovingObstacleTime(double t)
{
  activeSetCache_.clearAll();
  pairGenerator_.setMovingObstacleTime(t);
}

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
