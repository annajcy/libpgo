#pragma once

#include "EigenDef.h"
#include "ipc/core/surfaceIPCActiveSet.h"
#include "ipc/external/obstacleSurface.h"
#include "ipc/topology/surfaceIPCTopology.h"
#include "solver/common/solveDiagnostics.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace pgo
{
namespace Contact
{
namespace IPC
{

class IPCPairGenerator
{
public:
  struct Parameters
  {
    double dhat = 1e-1;
    double dhatExternal = 1e-1;
    double slackness = 1.0;
    double ccdThickness = 0.0;
  };

  IPCPairGenerator() = default;
  explicit IPCPairGenerator(const Parameters &params);
  IPCPairGenerator(const Parameters &params, std::vector<std::unique_ptr<ObstacleSurface>> obstacleSurfaces);

  IPCPairGenerator(const IPCPairGenerator &other);
  IPCPairGenerator &operator=(const IPCPairGenerator &other);

  void setParameters(const Parameters &params);
  Parameters parameters() const;
  void setMesh(const EigenSupport::MXd &surfaceRestVertices, const EigenSupport::MXi &surfaceTriangles);
  void setMovingObstacleTime(double time);

  SurfaceIPCActiveSet buildActiveSet(EigenSupport::ConstRefVecXd surfacePositions) const;
  SurfaceIPCActiveSet buildLineSearchActiveSetSuperset(
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::ConstRefVecXd surfaceStep) const;
  NonlinearOptimization::StepConstraint computeMaxStepLimit(
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::ConstRefVecXd surfaceStep,
    NonlinearOptimization::StepConstraintSink *sink = nullptr) const;

  const SurfaceIPCTopology &topology() const { return topology_; }
  std::vector<ObstacleSurfaceView> obstacleViews() const;

private:
  struct ObstacleSlot
  {
    enum class Kind
    {
      Static,
      Moving,
    };
    Kind kind = Kind::Static;
    std::size_t index = 0;
    int32_t objectId = -1;
  };

  void setObstacles(std::vector<std::unique_ptr<ObstacleSurface>> obstacleSurfaces);

  Parameters params_;
  SurfaceIPCTopology topology_;
  std::vector<std::unique_ptr<ObstacleSurface>> staticObstacles_;
  std::vector<std::unique_ptr<ObstacleSurface>> movingObstacles_;
  std::vector<ObstacleSlot> obstacleOrder_;
};

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
