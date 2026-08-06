/*
copyright to Bohan Wang
*/

#pragma once

#include "ipc/ipcActiveSetCache.h"
#include "ipc/ipcContactAssembler.h"
#include "ipc/ipcPairGenerator.h"
#include "ipc/external/obstacleSurface.h"
#include "energy/lineSearchAwareEnergy.h"
#include "statefulContactEnergy.h"
#include "surfaceDofMap.h"

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
  public StatefulContactEnergy,
  public NonlinearOptimization::LineSearchAwareEnergy
{
public:
  IPCContactEnergy(
    const EigenSupport::MXd &surfaceRestVertices,
    const EigenSupport::MXi &surfaceTriangles,
    const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
    const IPCPairGenerator::Parameters &pairParams = {},
    const IPCContactAssembler::Parameters &assemblerParams = {},
    std::vector<std::unique_ptr<ObstacleSurface>> obstacleSurfaces = {});

  ContactModelKind contactModelKind() const override { return ContactModelKind::IPC; }
  bool isStepDependent() const override { return false; }
  void beginStep(const NonlinearOptimization::StepState &state) override;
  NonlinearOptimization::StepConstraint computeMaxStepLimit(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::ConstRefVecXd trialSimulationDisplacements,
    NonlinearOptimization::StepConstraintSink *sink = nullptr) const override;
  void beginLineSearch(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::ConstRefVecXd trialSimulationDisplacements) const override;
  void endLineSearch() const override;
  double maxValidLineSearchAlpha() const override { return 1.0; }

  double func(EigenSupport::ConstRefVecXd simulationDisplacements) const override;
  void gradient(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::RefVecXd simulationGradient) const override;
  void hessian(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::SpMatD &simulationHessian) const override;
  void hessianInPlace(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::SpMatD &simulationHessian) const override;
  void hessianAlloc(EigenSupport::SpMatD &simulationHessian) const override;
  double funcGradient(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::RefVecXd simulationGradient) const override;
  double funcGradientHessian(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient,
    EigenSupport::SpMatD &simulationHessian) const override;
  void gradientHessian(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient,
    EigenSupport::SpMatD &simulationHessian) const override;
  void getDOFs(std::vector<int> &dofs) const override;
  int getNumDOFs() const override;
  int isHessianTopologyFixed() const override { return 0; }

  void setMovingObstacleTime(double t);

private:
  SurfaceIPCActiveSet buildExactActiveSet(EigenSupport::ConstRefVecXd surfacePositions) const;
  const SurfaceIPCActiveSet &activeSetForEvaluation(EigenSupport::ConstRefVecXd surfacePositions) const;

  SurfaceDofMap dofMap_;
  IPCPairGenerator pairGenerator_;
  IPCContactAssembler assembler_;
  mutable IPCActiveSetCache activeSetCache_;
};

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
