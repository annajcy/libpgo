/*
copyright to Bohan Wang
*/

// Full implementation of Codimensional IPC collision handling.
// =============================================================================

#include "ipc/core/surfaceIPCCore.h"
#include "scopedProfileSection.h"
#include "ipc/broadPhase/surfaceIPCBroadPhase.h"
#include "ipc/core/surfaceIPCSelfBarrierAssembler.h"
#include "ipc/core/surfaceIPCExternalBarrierAssembler.h"
#include "ipc/core/surfaceIPCMaxStep.h"
#include "ipc/profiling/surfaceIPCProfiling.h"

#include "pgoLogging.h"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace pgo {
namespace Contact {
namespace IPC {
using namespace pgo::EigenSupport;
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

SurfaceIPCCore::SurfaceIPCCore(const SurfaceIPCCore &other):
  dhat(other.dhat),
  dhat_external(other.dhat_external),
  kappa(other.kappa),
  eps_ee(other.eps_ee),
  slackness(other.slackness),
  ccd_thickness(other.ccd_thickness),
  topology_(other.topology_),
  staticObstacles_(cloneObstacleVector(other.staticObstacles_)),
  movingObstacles_(cloneObstacleVector(other.movingObstacles_)),
  obstacleOrder_(other.obstacleOrder_)
{
}

SurfaceIPCCore &SurfaceIPCCore::operator=(const SurfaceIPCCore &other)
{
  if (this == &other)
    return *this;

  dhat = other.dhat;
  dhat_external = other.dhat_external;
  kappa = other.kappa;
  eps_ee = other.eps_ee;
  slackness = other.slackness;
  ccd_thickness = other.ccd_thickness;
  topology_ = other.topology_;
  staticObstacles_ = cloneObstacleVector(other.staticObstacles_);
  movingObstacles_ = cloneObstacleVector(other.movingObstacles_);
  obstacleOrder_ = other.obstacleOrder_;
  return *this;
}

void SurfaceIPCCore::setParameters(const Parameters &params)
{
  dhat = params.dhat;
  dhat_external = params.dhat_external;
  kappa = params.kappa;
  eps_ee = params.eps_ee;
  slackness = params.slackness;
  ccd_thickness = params.ccd_thickness;
}

SurfaceIPCCore::Parameters SurfaceIPCCore::getParameters() const
{
  Parameters params;
  params.dhat = dhat;
  params.dhat_external = dhat_external;
  params.kappa = kappa;
  params.eps_ee = eps_ee;
  params.slackness = slackness;
  params.ccd_thickness = ccd_thickness;
  return params;
}

// =========================================================================
//  CollisionIPC  —  mesh setup
// =========================================================================

void SurfaceIPCCore::setMesh(const MXd &V, const MXi &F)
{
  topology_.setMesh(V, F);
}

// =========================================================================
//  Spatial hash grid for O(n) broad-phase collision detection
// =========================================================================

// =========================================================================
//  Broad phase: find candidate PT and EE pairs using spatial hashing
//  Uses insert-then-query: insert one type, query with the other.
// =========================================================================
SurfaceIPCActiveSet SurfaceIPCCore::buildActiveSet(EigenSupport::ConstRefVecXd x_surf) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kBuildActiveSet);

  SurfaceIPCActiveSet activeSet;
  activeSet.positions = x_surf;
  buildSelfPairs(topology_, activeSet.positions, dhat, activeSet.selfPairs);

  const std::vector<ObstacleSurfaceView> views = obstacleViews();
  if (!views.empty())
    buildExternalPairs(topology_, activeSet.positions, views, dhat_external, activeSet.externalPairs);

  if (auto logger = Logging::lgr(); logger && logger->should_log(spdlog::level::debug)) {
    const size_t selfTotal = activeSet.selfPairs.size();
    const size_t externalTotal = activeSet.externalPairs.size();
    SPDLOG_LOGGER_DEBUG(logger,
      "SurfaceIPCCore active pairs: selfPT={} selfEE={} selfTotal={} externalPT={} externalTP={} externalEE={} externalTotal={}",
      activeSet.selfPairs.ptPairs.size(), activeSet.selfPairs.eePairs.size(), selfTotal,
      activeSet.externalPairs.ptPairs.size(), activeSet.externalPairs.tpPairs.size(), activeSet.externalPairs.eePairs.size(), externalTotal);
  }

  return activeSet;
}

SurfaceIPCActiveSet SurfaceIPCCore::buildLineSearchActiveSetSuperset(
  EigenSupport::ConstRefVecXd x_surf,
  EigenSupport::ConstRefVecXd dx_surf) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kBuildActiveSet);

  SurfaceIPCActiveSet activeSet;
  activeSet.positions = x_surf;
  buildSelfPairsLineSearchSuperset(topology_, activeSet.positions, dx_surf, dhat, activeSet.selfPairs);

  const std::vector<ObstacleSurfaceView> views = obstacleViews();
  if (!views.empty())
    buildExternalPairsLineSearchSuperset(topology_, activeSet.positions, dx_surf, views, dhat_external, activeSet.externalPairs);

  return activeSet;
}

// =========================================================================
//  1)  Maximum step size  (CCD-based line search with spatial hashing)
// =========================================================================
NonlinearOptimization::StepConstraint SurfaceIPCCore::computeMaxStepLimit(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx, StepConstraintSink *sink) const
{
  double alpha = computeSelfMaxStep(topology_, x, dx, dhat, slackness, ccd_thickness);
  alpha = std::min(alpha, computeExternalMaxStep(topology_, x, dx, obstacleViews(), dhat_external, slackness, ccd_thickness));

  const double clampedAlpha = std::max(alpha, 1e-12);

  if (clampedAlpha < 1.0) {
    if (clampedAlpha > 0.0 && clampedAlpha < kSmallContactAlphaWarnThreshold) {
      SPDLOG_LOGGER_WARN(Logging::lgr(),
        "IPC contact max step produced small contactFeasibleAlpha={} (slackness={}).",
        clampedAlpha, slackness);
    }

    if (auto logger = Logging::lgr(); logger && logger->should_log(spdlog::level::trace)) {
      SPDLOG_LOGGER_TRACE(logger,
        "IPC contact clamp: contactFeasibleAlpha={} slackness={}.",
        clampedAlpha, slackness);
    }
  }

  NonlinearOptimization::StepConstraint c{NonlinearOptimization::StepSource::Contact, clampedAlpha};
  if (sink)
    sink->report(c);
  return c;
}

// =========================================================================
//  2)  Energy
// =========================================================================
double SurfaceIPCCore::computeEnergy(EigenSupport::ConstRefVecXd pos) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kEnergy);
  return computeEnergy(buildActiveSet(pos));
}

double SurfaceIPCCore::computeEnergy(const SurfaceIPCActiveSet &activeSet) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kActiveSetEnergy);
  double e = computeSelfEnergy(activeSet.positions, activeSet.selfPairs, topology_.numVerts, dhat, kappa, eps_ee);
  const std::vector<ObstacleSurfaceView> views = obstacleViews();
  if (!views.empty()) {
    e += computeExternalEnergy(
      activeSet.positions, views, activeSet.externalPairs, dhat_external, kappa, eps_ee);
  }
  return e;
}

// =========================================================================
//  2)  Gradient
// =========================================================================
void SurfaceIPCCore::computeGradient(EigenSupport::ConstRefVecXd pos, EigenSupport::RefVecXd grad) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kGradient);
  computeGradient(buildActiveSet(pos), grad);
}

void SurfaceIPCCore::computeGradient(const SurfaceIPCActiveSet &activeSet, EigenSupport::RefVecXd grad) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kActiveSetGradient);
  computeSelfGradient(activeSet.positions, activeSet.selfPairs, topology_.numVerts, dhat, kappa, eps_ee, grad);
  const std::vector<ObstacleSurfaceView> views = obstacleViews();
  if (!views.empty()) {
    computeExternalGradient(
      activeSet.positions, views, activeSet.externalPairs, topology_.numVerts, dhat_external, kappa, eps_ee, grad);
  }
}

// =========================================================================
//  2)  Sparse Hessian
// =========================================================================
void SurfaceIPCCore::computeHessian(EigenSupport::ConstRefVecXd pos, SpMatD &hess) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kHessian);
  computeHessian(buildActiveSet(pos), hess);
}

void SurfaceIPCCore::computeHessian(const SurfaceIPCActiveSet &activeSet, SpMatD &hess) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kActiveSetHessian);
  computeSelfHessian(activeSet.positions, activeSet.selfPairs, topology_.numVerts, dhat, kappa, eps_ee, hess);
  const std::vector<ObstacleSurfaceView> views = obstacleViews();
  if (!views.empty()) {
    computeExternalHessian(
      activeSet.positions, views, activeSet.externalPairs, topology_.numVerts, dhat_external, kappa, eps_ee, hess);
  }
  if (auto logger = Logging::lgr(); logger && logger->should_log(spdlog::level::debug))
    SPDLOG_LOGGER_DEBUG(logger, "# nonzeros in Hessian: {}", hess.nonZeros());
}

// =========================================================================
//  Combined computation (single broad-phase pass)
// =========================================================================
void SurfaceIPCCore::computeAll(EigenSupport::ConstRefVecXd x,
  double &energy, VXd &grad, SpMatD &hess) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kCombined);
  computeAll(buildActiveSet(x), energy, grad, hess);
}

void SurfaceIPCCore::computeAll(const SurfaceIPCActiveSet &activeSet, double &energy, VXd &grad, SpMatD &hess) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kActiveSetCombined);
  computeSelfAll(activeSet.positions, activeSet.selfPairs, topology_.numVerts, dhat, kappa, eps_ee, energy, grad, hess);
  const std::vector<ObstacleSurfaceView> views = obstacleViews();
  if (!views.empty()) {
    double extEnergy = 0.0;
    computeExternalAll(
      activeSet.positions, views, activeSet.externalPairs, topology_.numVerts, dhat_external, kappa, eps_ee, extEnergy, grad, hess);
    energy += extEnergy;
  }
  if (auto logger = Logging::lgr(); logger && logger->should_log(spdlog::level::debug))
    SPDLOG_LOGGER_DEBUG(logger, "# nonzeros in Hessian: {}", hess.nonZeros());
}

// =========================================================================
//  Obstacle (external) registration
// =========================================================================
void SurfaceIPCCore::setObstacles(std::vector<std::unique_ptr<ObstacleSurface>> obstacles)
{
  staticObstacles_.clear();
  movingObstacles_.clear();
  obstacleOrder_.clear();

  for (std::size_t slot = 0; slot < obstacles.size(); ++slot) {
    std::unique_ptr<ObstacleSurface> obstacle = std::move(obstacles[slot]);
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

std::vector<ObstacleSurfaceView> SurfaceIPCCore::obstacleViews() const
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

void SurfaceIPCCore::setMovingObstacleTime(double t)
{
  for (std::unique_ptr<ObstacleSurface> &obstacle : movingObstacles_)
    obstacle->update(t);
}

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
