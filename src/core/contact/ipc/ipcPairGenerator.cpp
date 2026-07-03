#include "ipc/ipcPairGenerator.h"

#include "ipc/broadPhase/surfaceIPCBroadPhase.h"
#include "ipc/core/surfaceIPCMaxStep.h"
#include "ipc/profiling/surfaceIPCProfiling.h"
#include "pgoLogging.h"
#include "scopedProfileSection.h"

#include <algorithm>
#include <utility>

namespace pgo
{
namespace Contact
{
namespace IPC
{

static constexpr double kSmallContactAlphaWarnThreshold = 1e-2;

namespace
{

std::vector<std::unique_ptr<ObstacleSurface>> cloneObstacleVector(
  const std::vector<std::unique_ptr<ObstacleSurface>> &obstacles)
{
  std::vector<std::unique_ptr<ObstacleSurface>> out;
  out.reserve(obstacles.size());
  for (const auto &obstacle : obstacles)
    out.push_back(obstacle->cloneSurface());
  return out;
}

}  // namespace

IPCPairGenerator::IPCPairGenerator(const Parameters &params)
{
  setParameters(params);
}

IPCPairGenerator::IPCPairGenerator(const Parameters &params, std::vector<std::unique_ptr<ObstacleSurface>> obstacleSurfaces)
{
  setParameters(params);
  setObstacles(std::move(obstacleSurfaces));
}

IPCPairGenerator::IPCPairGenerator(const IPCPairGenerator &other):
  params_(other.params_),
  topology_(other.topology_),
  staticObstacles_(cloneObstacleVector(other.staticObstacles_)),
  movingObstacles_(cloneObstacleVector(other.movingObstacles_)),
  obstacleOrder_(other.obstacleOrder_)
{
}

IPCPairGenerator &IPCPairGenerator::operator=(const IPCPairGenerator &other)
{
  if (this == &other)
    return *this;

  params_ = other.params_;
  topology_ = other.topology_;
  staticObstacles_ = cloneObstacleVector(other.staticObstacles_);
  movingObstacles_ = cloneObstacleVector(other.movingObstacles_);
  obstacleOrder_ = other.obstacleOrder_;
  return *this;
}

void IPCPairGenerator::setParameters(const Parameters &params)
{
  params_ = params;
}

IPCPairGenerator::Parameters IPCPairGenerator::parameters() const
{
  return params_;
}

void IPCPairGenerator::setMesh(const EigenSupport::MXd &surfaceRestVertices, const EigenSupport::MXi &surfaceTriangles)
{
  topology_.setMesh(surfaceRestVertices, surfaceTriangles);
}

SurfaceIPCActiveSet IPCPairGenerator::buildActiveSet(EigenSupport::ConstRefVecXd surfacePositions) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kBuildActiveSet);

  SurfaceIPCActiveSet activeSet;
  activeSet.positions = surfacePositions;
  buildSelfPairs(topology_, activeSet.positions, params_.dhat, activeSet.selfPairs);

  const std::vector<ObstacleSurfaceView> views = obstacleViews();
  if (!views.empty())
    buildExternalPairs(topology_, activeSet.positions, views, params_.dhatExternal, activeSet.externalPairs);

  if (auto logger = Logging::lgr(); logger && logger->should_log(spdlog::level::debug)) {
    const size_t selfTotal = activeSet.selfPairs.size();
    const size_t externalTotal = activeSet.externalPairs.size();
    SPDLOG_LOGGER_DEBUG(logger,
      "IPCPairGenerator active pairs: selfPT={} selfEE={} selfTotal={} externalPT={} externalTP={} externalEE={} externalTotal={}",
      activeSet.selfPairs.ptPairs.size(), activeSet.selfPairs.eePairs.size(), selfTotal,
      activeSet.externalPairs.ptPairs.size(), activeSet.externalPairs.tpPairs.size(), activeSet.externalPairs.eePairs.size(), externalTotal);
  }

  return activeSet;
}

SurfaceIPCActiveSet IPCPairGenerator::buildLineSearchActiveSetSuperset(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::ConstRefVecXd surfaceStep) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kBuildActiveSet);

  SurfaceIPCActiveSet activeSet;
  activeSet.positions = surfacePositions;
  buildSelfPairsLineSearchSuperset(topology_, activeSet.positions, surfaceStep, params_.dhat, activeSet.selfPairs);

  const std::vector<ObstacleSurfaceView> views = obstacleViews();
  if (!views.empty()) {
    buildExternalPairsLineSearchSuperset(
      topology_, activeSet.positions, surfaceStep, views, params_.dhatExternal, activeSet.externalPairs);
  }

  return activeSet;
}

NonlinearOptimization::StepConstraint IPCPairGenerator::computeMaxStepLimit(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::ConstRefVecXd surfaceStep,
  NonlinearOptimization::StepConstraintSink *sink) const
{
  double alpha = computeSelfMaxStep(
    topology_, surfacePositions, surfaceStep, params_.dhat, params_.slackness, params_.ccdThickness);
  alpha = std::min(alpha,
    computeExternalMaxStep(topology_, surfacePositions, surfaceStep, obstacleViews(), params_.dhatExternal, params_.slackness, params_.ccdThickness));

  const double clampedAlpha = std::max(alpha, 1e-12);

  if (clampedAlpha < 1.0) {
    if (clampedAlpha > 0.0 && clampedAlpha < kSmallContactAlphaWarnThreshold) {
      SPDLOG_LOGGER_WARN(Logging::lgr(),
        "IPC contact max step produced small contactFeasibleAlpha={} (slackness={}).",
        clampedAlpha, params_.slackness);
    }

    if (auto logger = Logging::lgr(); logger && logger->should_log(spdlog::level::trace)) {
      SPDLOG_LOGGER_TRACE(logger,
        "IPC contact clamp: contactFeasibleAlpha={} slackness={}.",
        clampedAlpha, params_.slackness);
    }
  }

  NonlinearOptimization::StepConstraint constraint{NonlinearOptimization::StepSource::Contact, clampedAlpha};
  if (sink)
    sink->report(constraint);
  return constraint;
}

void IPCPairGenerator::setObstacles(std::vector<std::unique_ptr<ObstacleSurface>> obstacleSurfaces)
{
  staticObstacles_.clear();
  movingObstacles_.clear();
  obstacleOrder_.clear();

  for (std::size_t slot = 0; slot < obstacleSurfaces.size(); ++slot) {
    std::unique_ptr<ObstacleSurface> obstacle = std::move(obstacleSurfaces[slot]);
    obstacle->setObjectId(static_cast<int32_t>(slot));

    if (dynamic_cast<StaticObstacleSurface *>(obstacle.get()) != nullptr) {
      const std::size_t index = staticObstacles_.size();
      staticObstacles_.push_back(std::move(obstacle));
      obstacleOrder_.push_back({ ObstacleSlot::Kind::Static, index, static_cast<int32_t>(slot) });
    }
    else {
      const std::size_t index = movingObstacles_.size();
      movingObstacles_.push_back(std::move(obstacle));
      obstacleOrder_.push_back({ ObstacleSlot::Kind::Moving, index, static_cast<int32_t>(slot) });
    }
  }
}

std::vector<ObstacleSurfaceView> IPCPairGenerator::obstacleViews() const
{
  std::vector<ObstacleSurfaceView> views;
  views.reserve(obstacleOrder_.size());
  for (const ObstacleSlot &slot : obstacleOrder_) {
    const ObstacleSurface *obstacle = slot.kind == ObstacleSlot::Kind::Static ?
      staticObstacles_[slot.index].get() :
      movingObstacles_[slot.index].get();
    views.push_back(makeObstacleSurfaceView(*obstacle));
  }
  return views;
}

void IPCPairGenerator::setMovingObstacleTime(double time)
{
  for (std::unique_ptr<ObstacleSurface> &obstacle : movingObstacles_)
    obstacle->update(time);
}

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
