#include "ipc/ipcContactAssembler.h"

#include "ipc/core/surfaceIPCExternalBarrierAssembler.h"
#include "ipc/core/surfaceIPCSelfBarrierAssembler.h"
#include "ipc/profiling/surfaceIPCProfiling.h"
#include "pgoLogging.h"
#include "scopedProfileSection.h"

namespace pgo
{
namespace Contact
{
namespace IPC
{

IPCContactAssembler::IPCContactAssembler(const Parameters &params)
{
  setParameters(params);
}

void IPCContactAssembler::setParameters(const Parameters &params)
{
  params_ = params;
}

IPCContactAssembler::Parameters IPCContactAssembler::parameters() const
{
  return params_;
}

double IPCContactAssembler::computeEnergy(
  const SurfaceIPCTopology &topology,
  const std::vector<ObstacleSurfaceView> &obstacleViews,
  const SurfaceIPCActiveSet &activeSet) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kActiveSetEnergy);
  double surfaceEnergy = computeSelfEnergy(
    activeSet.positions, activeSet.selfPairs, topology.numVerts, params_.dhat, params_.kappa, params_.epsEE);
  if (!obstacleViews.empty()) {
    surfaceEnergy += computeExternalEnergy(
      activeSet.positions, obstacleViews, activeSet.externalPairs, params_.dhatExternal, params_.kappa, params_.epsEE);
  }
  return surfaceEnergy;
}

void IPCContactAssembler::computeGradient(
  const SurfaceIPCTopology &topology,
  const std::vector<ObstacleSurfaceView> &obstacleViews,
  const SurfaceIPCActiveSet &activeSet,
  EigenSupport::RefVecXd surfaceGradient) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kActiveSetGradient);
  computeSelfGradient(
    activeSet.positions, activeSet.selfPairs, topology.numVerts, params_.dhat, params_.kappa, params_.epsEE, surfaceGradient);
  if (!obstacleViews.empty()) {
    computeExternalGradient(
      activeSet.positions, obstacleViews, activeSet.externalPairs, topology.numVerts, params_.dhatExternal, params_.kappa, params_.epsEE, surfaceGradient);
  }
}

void IPCContactAssembler::computeHessian(
  const SurfaceIPCTopology &topology,
  const std::vector<ObstacleSurfaceView> &obstacleViews,
  const SurfaceIPCActiveSet &activeSet,
  EigenSupport::SpMatD &surfaceHessian) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kActiveSetHessian);
  computeSelfHessian(
    activeSet.positions, activeSet.selfPairs, topology.numVerts, params_.dhat, params_.kappa, params_.epsEE, surfaceHessian);
  if (!obstacleViews.empty()) {
    computeExternalHessian(
      activeSet.positions, obstacleViews, activeSet.externalPairs, topology.numVerts, params_.dhatExternal, params_.kappa, params_.epsEE, surfaceHessian);
  }
  if (auto logger = Logging::lgr(); logger && logger->should_log(spdlog::level::debug))
    SPDLOG_LOGGER_DEBUG(logger, "# nonzeros in Hessian: {}", surfaceHessian.nonZeros());
}

void IPCContactAssembler::computeAll(
  const SurfaceIPCTopology &topology,
  const std::vector<ObstacleSurfaceView> &obstacleViews,
  const SurfaceIPCActiveSet &activeSet,
  double &surfaceEnergy,
  EigenSupport::VXd &surfaceGradient,
  EigenSupport::SpMatD &surfaceHessian) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kActiveSetCombined);
  computeSelfAll(
    activeSet.positions, activeSet.selfPairs, topology.numVerts, params_.dhat, params_.kappa, params_.epsEE, surfaceEnergy, surfaceGradient, surfaceHessian);
  if (!obstacleViews.empty()) {
    double externalEnergy = 0.0;
    computeExternalAll(
      activeSet.positions, obstacleViews, activeSet.externalPairs, topology.numVerts, params_.dhatExternal, params_.kappa, params_.epsEE, externalEnergy, surfaceGradient, surfaceHessian);
    surfaceEnergy += externalEnergy;
  }
  if (auto logger = Logging::lgr(); logger && logger->should_log(spdlog::level::debug))
    SPDLOG_LOGGER_DEBUG(logger, "# nonzeros in Hessian: {}", surfaceHessian.nonZeros());
}

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
