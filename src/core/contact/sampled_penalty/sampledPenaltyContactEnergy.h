/*
  Long-lived sampled penalty contact energies.
*/

#pragma once

#include "sampled_penalty/sampledPenaltyContactBuilder.h"
#include "sampled_penalty/sampledPenaltyContactEvaluator.h"
#include "sampled_penalty/sampledPenaltyFrictionState.h"
#include "sampled_penalty/sampledPenaltySpecs.h"
#include "statefulContactEnergy.h"
#include "surfaceDofMap.h"
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

struct SampledPenaltyContactEnergyOptions
{
  ParametersSpec params;
  std::optional<FrictionParametersSpec> friction;
};

class SampledPenaltyContactEnergy final:
  public StatefulContactEnergy
{
public:
  SampledPenaltyContactEnergy(
    const EigenSupport::MXd &surfaceRestVertices,
    const EigenSupport::MXi &surfaceTriangles,
    const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
    const SampledPenaltyContactEnergyOptions &options,
    std::vector<Mesh::TriMeshGeo> externalSurfaces = {});
  ~SampledPenaltyContactEnergy() override;

  ContactModelKind contactModelKind() const override { return ContactModelKind::SampledPenalty; }
  bool isStepDependent() const override { return frictionState_.has_value(); }
  void beginStep(const NonlinearOptimization::StepState &state) override;

  double func(EigenSupport::ConstRefVecXd simulationDisplacements) const override;
  void gradient(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::RefVecXd simulationGradient) const override;
  void hessian(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::SpMatD &simulationHessian) const override;
  void hessianInPlace(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::SpMatD &simulationHessian) const override;
  void hessianAlloc(EigenSupport::SpMatD &simulationHessian) const override;
  double func_grad(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::RefVecXd simulationGradient) const override;
  double func_grad_hessian(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient,
    EigenSupport::SpMatD &simulationHessian) const override;
  void gradient_hessian(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient,
    EigenSupport::SpMatD &simulationHessian) const override;
  void getDOFs(std::vector<int> &dofs) const override;
  int getNumDOFs() const override;
  int isHessianTopologyFixed() const override { return 0; }

  void updateExternalSurface(int index, const Mesh::TriMeshGeo &surface);

private:
  SampledPenaltyEnergyConfigurator makeConfigurator() const;
  std::unique_ptr<SampledPenaltyEvaluationBundle> buildBundle(EigenSupport::ConstRefVecXd surfacePositions) const;
  const SampledPenaltyEvaluationBundle &evaluationBundle(
    EigenSupport::ConstRefVecXd surfacePositions,
    std::unique_ptr<SampledPenaltyEvaluationBundle> &fallbackBundle) const;
  void configureExternalEnergy(PointPenetrationEnergy &energy) const;
  void configureSelfEnergy(
    PointTrianglePairCouplingEnergyWithCollision &energy,
    EigenSupport::ConstRefVecXd surfacePositions) const;

  SurfaceDofMap dofMap_;
  SampledPenaltyContactEnergyOptions options_;
  SampledPenaltyContactBuilder builder_;
  SampledPenaltyContactEvaluator evaluator_;
  std::optional<SampledPenaltyFrictionState> frictionState_;
  std::unique_ptr<SampledPenaltyEvaluationBundle> stepBundle_;
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
