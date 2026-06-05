/*
copyright to Bohan Wang
*/

#pragma once

#include "ipc/ipcActiveSetCache.h"
#include "mappedSurfacePotentialEnergy.h"
#include "ipc/core/surfaceIPCCore.h"
#include "ipc/external/obstacleSurface.h"
#include "stepAwareEnergy.h"

#include <vector>
#include <memory>

namespace pgo
{
namespace Contact
{
namespace IPC
{

using namespace pgo::EigenSupport;

class IPCContactEnergy:
  public MappedSurfacePotentialEnergy,
  public ActiveSetContactEnergy,
  public NonlinearOptimization::StepAwareEnergy
{
public:
  IPCContactEnergy(
    const EigenSupport::MXd &surfaceRestVertices,
    const EigenSupport::MXi &surfaceTriangles,
    const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
    const SurfaceIPCCore::Parameters &ipcParams = {},
    std::vector<std::unique_ptr<ObstacleSurface>> obstacleSurfaces = {});

  virtual ContactModelKind contactModelKind() const override { return ContactModelKind::IPC; }
  virtual void beginStep(const NonlinearOptimization::StepState &state) override;
  virtual NonlinearOptimization::StepConstraint computeMaxStepLimit(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::ConstRefVecXd trialSimulationDisplacements,
    NonlinearOptimization::StepConstraintSink *sink = nullptr) const override;

  void setMovingObstacleTime(double t);

private:
  SurfaceIPCActiveSet buildExactActiveSet(EigenSupport::ConstRefVecXd surfacePositions) const;
  const SurfaceIPCActiveSet &activeSetForEvaluation(EigenSupport::ConstRefVecXd surfacePositions) const;

  virtual void prepareActiveSet(EigenSupport::ConstRefVecXd simulationDisplacements) const override;
  virtual void clearPreparedActiveSet() const override;
  virtual void beginActiveSetLineSearch(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::ConstRefVecXd trialSimulationDisplacements) const override;
  virtual void endActiveSetLineSearch() const override;

  virtual double computeSurfaceEnergy(EigenSupport::ConstRefVecXd surfacePositions) const override;
  virtual void computeSurfaceGradient(
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::RefVecXd surfaceGradient) const override;
  virtual void computeSurfaceHessian(
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::SpMatD &surfaceHessian) const override;
  virtual void computeSurfaceGradHessian(
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::RefVecXd surfaceGradient,
    EigenSupport::SpMatD &surfaceHessian) const override;
  virtual void computeSurfaceFuncGrad(
    EigenSupport::ConstRefVecXd surfacePositions,
    double &surfaceEnergy,
    EigenSupport::RefVecXd surfaceGradient) const override;
  virtual void computeSurfaceAll(
    EigenSupport::ConstRefVecXd surfacePositions,
    double &surfaceEnergy,
    EigenSupport::RefVecXd surfaceGradient,
    EigenSupport::SpMatD &surfaceHessian) const override;
  SurfaceIPCCore surfaceIPCCore_;
  mutable IPCActiveSetCache activeSetCache_;
};

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
