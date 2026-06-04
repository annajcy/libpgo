#pragma once

#include "EigenDef.h"
#include "ipc/external/obstaclePoseCache.h"

#include <cstdint>
#include <functional>
#include <memory>

namespace pgo
{
namespace Contact
{
namespace IPC
{

class ObstacleSurface
{
public:
  using TrajectorySampler = std::function<void(double t, EigenSupport::RefVecXd out)>;

  virtual ~ObstacleSurface() = default;
  virtual std::unique_ptr<ObstacleSurface> cloneSurface() const = 0;

  // Sample the obstacle pose at absolute time t and rebuild the pose-derived
  // cache (areas, lengths, AABBs, spatial hashes, cell size). Treat the
  // obstacle as fixed at this pose for the subsequent solve; broad-phase /
  // max-step callers should read derived state via `cache()` rather than
  // rebuilding it themselves.
  void update(double t);

  virtual int32_t                  objectId()         const { return objectId_; }
  virtual const EigenSupport::VXd &restPositions()    const { return rest_; }
  virtual const EigenSupport::VXd &currentPositions() const { return current_; }
  virtual const EigenSupport::MXi &triangles()        const { return triangles_; }
  virtual const EigenSupport::MXi &uniqueEdges()      const { return uniqueEdges_; }
  virtual const EigenSupport::MXi &contactEdges()     const { return contactEdges_; }
  virtual const ObstaclePoseCache &cache()            const { return cache_; }

  virtual void setObjectId(int32_t id) { objectId_ = id; }

protected:
  ObstacleSurface(
    EigenSupport::MXd restVertices,    // num_obstacle_vertices x 3
    EigenSupport::MXi triangles,       // num_obstacle_tris   x 3, local index
    TrajectorySampler sampler);        // sampler(t, out) writes 3*num_vertices

  int32_t              objectId_ = -1;
  EigenSupport::VXd    rest_;
  EigenSupport::VXd    current_;
  EigenSupport::MXi    triangles_;       // local 0-based indices
  EigenSupport::MXi    uniqueEdges_;     // all topological edges derived from triangles_
  EigenSupport::MXi    contactEdges_;    // boundary/nonmanifold/sharp edges used for external EE
  TrajectorySampler    sampler_;
  ObstaclePoseCache    cache_;
};

class StaticObstacleSurface final : public ObstacleSurface
{
public:
  StaticObstacleSurface(EigenSupport::MXd restVertices, EigenSupport::MXi triangles);

  std::unique_ptr<StaticObstacleSurface> cloneStatic() const;
  std::unique_ptr<ObstacleSurface> cloneSurface() const override;
};

class MovingObstacleSurface : public ObstacleSurface
{
public:
  using ObstacleSurface::ObstacleSurface;
  ~MovingObstacleSurface() override = default;

  virtual void setTime(double t) = 0;
  virtual std::unique_ptr<MovingObstacleSurface> cloneMoving() const = 0;
};

class LinearMovingObstacleSurface final : public MovingObstacleSurface
{
public:
  LinearMovingObstacleSurface(
    EigenSupport::MXd restVertices,
    EigenSupport::MXi triangles,
    EigenSupport::V3d velocity,
    double referenceTime = 0.0);

  void setTime(double t) override;
  std::unique_ptr<MovingObstacleSurface> cloneMoving() const override;
  std::unique_ptr<ObstacleSurface> cloneSurface() const override;
};

class TrajectoryObstacleSurface final : public MovingObstacleSurface
{
public:
  TrajectoryObstacleSurface(
    EigenSupport::MXd restVertices,
    EigenSupport::MXi triangles,
    TrajectorySampler sampler);

  void setTime(double t) override;
  std::unique_ptr<MovingObstacleSurface> cloneMoving() const override;
  std::unique_ptr<ObstacleSurface> cloneSurface() const override;
};

struct ObstacleSurfaceView
{
  int32_t objectIdValue = -1;
  const EigenSupport::VXd *currentPositionsPtr = nullptr;
  const EigenSupport::MXi *trianglesPtr = nullptr;
  const EigenSupport::MXi *uniqueEdgesPtr = nullptr;
  const EigenSupport::MXi *contactEdgesPtr = nullptr;
  const ObstaclePoseCache *cachePtr = nullptr;

  int32_t objectId() const { return objectIdValue; }
  const EigenSupport::VXd &currentPositions() const { return *currentPositionsPtr; }
  const EigenSupport::MXi &triangles() const { return *trianglesPtr; }
  const EigenSupport::MXi &uniqueEdges() const { return *uniqueEdgesPtr; }
  const EigenSupport::MXi &contactEdges() const { return *contactEdgesPtr; }
  const ObstaclePoseCache &cache() const { return *cachePtr; }
};

ObstacleSurfaceView makeObstacleSurfaceView(const ObstacleSurface &obstacle);

ObstacleSurface::TrajectorySampler makeLinearTrajectorySampler(
  const EigenSupport::VXd &restPositions,
  const EigenSupport::V3d &velocity,
  double t0 = 0.0);

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
