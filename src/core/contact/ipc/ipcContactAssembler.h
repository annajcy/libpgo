#pragma once

#include "EigenDef.h"
#include "ipc/core/surfaceIPCActiveSet.h"
#include "ipc/external/obstacleSurface.h"
#include "ipc/topology/surfaceIPCTopology.h"

#include <vector>

namespace pgo
{
namespace Contact
{
namespace IPC
{

class IPCContactAssembler
{
public:
  struct Parameters
  {
    double dhat = 1e-1;
    double dhatExternal = 1e-1;
    double kappa = 0.1;
    double epsEE = 0.0;
  };

  IPCContactAssembler() = default;
  explicit IPCContactAssembler(const Parameters &params);

  void setParameters(const Parameters &params);
  Parameters parameters() const;

  // Assembly methods overwrite/reset gradient, Hessian, and fused outputs for
  // self assembly before external obstacle contributions are accumulated.
  double computeEnergy(
    const SurfaceIPCTopology &topology,
    const std::vector<ObstacleSurfaceView> &obstacleViews,
    const SurfaceIPCActiveSet &activeSet) const;
  void computeGradient(
    const SurfaceIPCTopology &topology,
    const std::vector<ObstacleSurfaceView> &obstacleViews,
    const SurfaceIPCActiveSet &activeSet,
    EigenSupport::RefVecXd surfaceGradient) const;
  void computeHessian(
    const SurfaceIPCTopology &topology,
    const std::vector<ObstacleSurfaceView> &obstacleViews,
    const SurfaceIPCActiveSet &activeSet,
    EigenSupport::SpMatD &surfaceHessian) const;
  void computeAll(
    const SurfaceIPCTopology &topology,
    const std::vector<ObstacleSurfaceView> &obstacleViews,
    const SurfaceIPCActiveSet &activeSet,
    double &surfaceEnergy,
    EigenSupport::VXd &surfaceGradient,
    EigenSupport::SpMatD &surfaceHessian) const;

private:
  Parameters params_;
};

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
