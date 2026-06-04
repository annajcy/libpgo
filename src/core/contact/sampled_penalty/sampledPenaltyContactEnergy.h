/*
  Long-lived sampled penalty contact energies.
*/

#pragma once

#include "statefulContactEnergy.h"
#include "stepDependentEnergy.h"
#include "triMeshGeo.h"

#include <memory>
#include <vector>

namespace pgo
{
namespace Contact
{
class PointPenetrationEnergy;
class PointTrianglePairCouplingEnergyWithCollision;
class TriangleMeshExternalContactHandler;
class TriangleMeshSelfContactHandler;

namespace SampledPenalty
{

struct ParametersSpec
{
  double stiffness = 1.0;
  int samples = 1;
  bool enableSelfContact = true;
  bool enableExternalContact = true;
};

struct FrictionParametersSpec
{
  double frictionCoeff = 1.0;
  double velocityEps = 1.0;
};

struct SampledPenaltyActiveSet;

class SampledPenaltyContactEnergy : public StatefulContactEnergy
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

  void beginStep(const NonlinearOptimization::StepState &state) override;
  void refreshActiveSet(EigenSupport::ConstRefVecXd x) const override;
  void clearActiveSet() const override;
  void updateExternalSurface(int index, const Mesh::TriMeshGeo &surface);
  void beginLineSearch(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx) const override;
  void endLineSearch() const override;

protected:
  virtual bool hasFrictionStepState() const { return false; }
  virtual const EigenSupport::VXd *previousStepState() const { return nullptr; }
  virtual double stepTimestep() const { return 0.0; }
  virtual double frictionCoeff() const { return 0.0; }
  virtual double velocityEps() const { return 0.0; }

  virtual void configureExternalActiveEnergy(PointPenetrationEnergy &energy) const;
  virtual void configureSelfActiveEnergy(PointTrianglePairCouplingEnergyWithCollision &energy, EigenSupport::ConstRefVecXd x) const;

private:
  void validateStateVector(EigenSupport::ConstRefVecXd x) const;
  const SampledPenaltyActiveSet &evaluationActiveSet(EigenSupport::ConstRefVecXd x, const char *reason) const;
  std::unique_ptr<SampledPenaltyActiveSet> buildActiveSet(EigenSupport::ConstRefVecXd x) const;

protected:
  Mesh::TriMeshGeo surfaceMesh_;
  EigenSupport::VXd simulationRestPositions_;
  ParametersSpec params_;
  std::vector<Mesh::TriMeshGeo> externalSurfaces_;
  std::vector<int> vertexEmbeddingIndices_;
  std::vector<double> vertexEmbeddingWeights_;
  std::vector<int> dofs_;

  std::shared_ptr<TriangleMeshExternalContactHandler> externalHandler_;
  std::shared_ptr<TriangleMeshSelfContactHandler> selfHandler_;

  mutable std::unique_ptr<SampledPenaltyActiveSet> activeSet_;
  mutable std::unique_ptr<SampledPenaltyActiveSet> lineSearchActiveSet_;
};

class FrictionalSampledPenaltyContactEnergy final:
  public SampledPenaltyContactEnergy,
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

protected:
  bool hasFrictionStepState() const override { return hasStepState_; }
  const EigenSupport::VXd *previousStepState() const override { return &previousX_; }
  double stepTimestep() const override { return timestep_; }
  double frictionCoeff() const override { return frictionParams_.frictionCoeff; }
  double velocityEps() const override { return frictionParams_.velocityEps; }

  void configureExternalActiveEnergy(PointPenetrationEnergy &energy) const override;
  void configureSelfActiveEnergy(PointTrianglePairCouplingEnergyWithCollision &energy, EigenSupport::ConstRefVecXd x) const override;

private:
  FrictionParametersSpec frictionParams_;
  EigenSupport::VXd previousX_;
  double timestep_ = 0.0;
  bool hasStepState_ = false;
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
