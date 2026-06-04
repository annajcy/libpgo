/*
copyright to Bohan Wang
*/

#pragma once

#include "mappedSurfacePotentialEnergy.h"
#include "ipc/core/surfaceIPCCore.h"
#include "ipc/external/obstacleSurface.h"

#include <vector>
#include <memory>

namespace pgo
{
namespace Contact
{
namespace IPC
{

using namespace pgo::EigenSupport;

class IPCContactEnergy : public MappedSurfacePotentialEnergy
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
  virtual void refreshActiveSet(EigenSupport::ConstRefVecXd simulationDisplacements) const override;
  virtual void clearActiveSet() const override;

  void setMovingObstacleTime(double t);

private:
  void cacheEnergyActiveSet(SurfaceIPCActiveSet activeSet) const;
  const SurfaceIPCActiveSet *cachedEnergyActiveSetFor(EigenSupport::ConstRefVecXd surfacePositions) const;
  const SurfaceIPCActiveSet &evaluationActiveSetFor(
    EigenSupport::ConstRefVecXd surfacePositions,
    const char *reason) const;
  void clearCachedEnergyActiveSet() const;

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
  virtual NonlinearOptimization::StepConstraint computeSurfaceMaxStepLimit(
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::ConstRefVecXd surfaceDisplacements,
    NonlinearOptimization::StepConstraintSink *sink = nullptr) const override;
  virtual void beginSurfaceLineSearch(
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::ConstRefVecXd surfaceDisplacements) const override;
  virtual void endSurfaceLineSearch() const override;

  SurfaceIPCCore surfaceIPCCore_;
  mutable bool hasCachedEnergyActiveSet_ = false;
  mutable SurfaceIPCActiveSet cachedEnergyActiveSet_;
  mutable bool hasLineSearchActiveSet_ = false;
  mutable SurfaceIPCActiveSet lineSearchActiveSet_;
};

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
