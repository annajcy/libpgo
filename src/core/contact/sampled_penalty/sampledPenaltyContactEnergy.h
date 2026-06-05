/*
  Long-lived sampled penalty contact energies.
*/

#pragma once

#include "sampled_penalty/sampledPenaltyActiveSetCache.h"
#include "sampled_penalty/sampledPenaltyContactDetector.h"
#include "sampled_penalty/sampledPenaltyFrictionState.h"
#include "sampled_penalty/sampledPenaltySpecs.h"
#include "statefulContactEnergy.h"
#include "stepAwareEnergy.h"
#include "stepDependentEnergy.h"
#include "triMeshGeo.h"

#include <memory>
#include <optional>
#include <vector>

namespace pgo
{
namespace Contact
{
class PointPenetrationEnergy;
class PointTrianglePairCouplingEnergyWithCollision;

namespace SampledPenalty
{

class SampledPenaltySurfaceContactEnergy:
  public StatefulContactEnergy,
  public ActiveSetContactEnergy
{
public:
  SampledPenaltySurfaceContactEnergy(
    const Mesh::TriMeshGeo &surfaceMesh,
    const ParametersSpec &params,
    std::vector<Mesh::TriMeshGeo> externalSurfaces = {});
  ~SampledPenaltySurfaceContactEnergy() override;

  ContactModelKind contactModelKind() const override { return ContactModelKind::SampledPenalty; }

  double func(EigenSupport::ConstRefVecXd surfacePositions) const override;
  void gradient(EigenSupport::ConstRefVecXd surfacePositions, EigenSupport::RefVecXd grad) const override;
  void hessian(EigenSupport::ConstRefVecXd surfacePositions, EigenSupport::SpMatD &hess) const override;
  void hessianInPlace(EigenSupport::ConstRefVecXd surfacePositions, EigenSupport::SpMatD &hess) const override;
  void hessianAlloc(EigenSupport::SpMatD &hess) const override;
  void getDOFs(std::vector<int> &dofs) const override;
  int getNumDOFs() const override { return surfaceDofCount_; }
  int isHessianTopologyFixed() const override { return 0; }

  void updateExternalSurface(int index, const Mesh::TriMeshGeo &surface);

protected:
  void configureExternalActiveEnergy(PointPenetrationEnergy &energy) const;
  void configureSelfActiveEnergy(PointTrianglePairCouplingEnergyWithCollision &energy, EigenSupport::ConstRefVecXd surfacePositions) const;
  void resetActiveSets() const;

private:
  void validateSurfacePositionVector(EigenSupport::ConstRefVecXd surfacePositions) const;
  const SampledPenaltyActiveSet &evaluationActiveSet(EigenSupport::ConstRefVecXd surfacePositions) const;
  std::unique_ptr<SampledPenaltyActiveSet> buildActiveSet(EigenSupport::ConstRefVecXd surfacePositions) const;

  void prepareActiveSet(EigenSupport::ConstRefVecXd surfacePositions) const override;
  void clearPreparedActiveSet() const override;
  void beginActiveSetLineSearch(
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::ConstRefVecXd surfaceStep) const override;
  void endActiveSetLineSearch() const override;

protected:
  int surfaceDofCount_ = 0;
  ParametersSpec params_;
  SampledPenaltyContactDetector detector_;
  std::vector<int> dofs_;
  mutable SampledPenaltyActiveSetCache activeSetCache_;
  std::optional<SampledPenaltyFrictionState> frictionState_;
};

class FrictionalSampledPenaltySurfaceContactEnergy final:
  public SampledPenaltySurfaceContactEnergy,
  public NonlinearOptimization::StepAwareEnergy,
  public NonlinearOptimization::StepDependentEnergy
{
public:
  FrictionalSampledPenaltySurfaceContactEnergy(
    const Mesh::TriMeshGeo &surfaceMesh,
    const ParametersSpec &params,
    const FrictionParametersSpec &frictionParams,
    std::vector<Mesh::TriMeshGeo> externalSurfaces = {});

  void beginStep(const NonlinearOptimization::StepState &state) override;
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
