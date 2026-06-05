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

class SampledPenaltyContactEnergy:
  public StatefulContactEnergy,
  public ActiveSetContactEnergy
{
public:
  SampledPenaltyContactEnergy(
    const Mesh::TriMeshGeo &surfaceMesh,
    EigenSupport::ConstRefVecXd simulationRestPositions,
    const ParametersSpec &params,
    std::vector<Mesh::TriMeshGeo> externalSurfaces = {},
    std::vector<int> vertexEmbeddingIndices = {},
    std::vector<double> vertexEmbeddingWeights = {});
  ~SampledPenaltyContactEnergy() override;

  ContactModelKind contactModelKind() const override { return ContactModelKind::SampledPenalty; }

  double func(EigenSupport::ConstRefVecXd x) const override;
  void gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const override;
  void hessian(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const override;
  void hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const override;
  void hessianAlloc(EigenSupport::SpMatD &hess) const override;
  void getDOFs(std::vector<int> &dofs) const override;
  int getNumDOFs() const override { return static_cast<int>(simulationRestPositions_.size()); }
  int isHessianTopologyFixed() const override { return 0; }

  void updateExternalSurface(int index, const Mesh::TriMeshGeo &surface);

protected:
  void configureExternalActiveEnergy(PointPenetrationEnergy &energy) const;
  void configureSelfActiveEnergy(PointTrianglePairCouplingEnergyWithCollision &energy, EigenSupport::ConstRefVecXd x) const;

private:
  void validateStateVector(EigenSupport::ConstRefVecXd x) const;
  const SampledPenaltyActiveSet &evaluationActiveSet(EigenSupport::ConstRefVecXd x) const;
  std::unique_ptr<SampledPenaltyActiveSet> buildActiveSet(EigenSupport::ConstRefVecXd x) const;

  virtual void prepareActiveSet(EigenSupport::ConstRefVecXd x) const override;
  virtual void clearPreparedActiveSet() const override;
  virtual void beginActiveSetLineSearch(
    EigenSupport::ConstRefVecXd x,
    EigenSupport::ConstRefVecXd dx) const override;
  virtual void endActiveSetLineSearch() const override;

protected:
  void resetActiveSets() const;

  EigenSupport::VXd simulationRestPositions_;
  ParametersSpec params_;
  SampledPenaltyContactDetector detector_;
  std::vector<int> dofs_;

  mutable SampledPenaltyActiveSetCache activeSetCache_;
  std::optional<SampledPenaltyFrictionState> frictionState_;
};

class FrictionalSampledPenaltyContactEnergy final:
  public SampledPenaltyContactEnergy,
  public NonlinearOptimization::StepAwareEnergy,
  public NonlinearOptimization::StepDependentEnergy
{
public:
  FrictionalSampledPenaltyContactEnergy(
    const Mesh::TriMeshGeo &surfaceMesh,
    EigenSupport::ConstRefVecXd simulationRestPositions,
    const ParametersSpec &params,
    const FrictionParametersSpec &frictionParams,
    std::vector<Mesh::TriMeshGeo> externalSurfaces = {},
    std::vector<int> vertexEmbeddingIndices = {},
    std::vector<double> vertexEmbeddingWeights = {});

  void beginStep(const NonlinearOptimization::StepState &state) override;
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
