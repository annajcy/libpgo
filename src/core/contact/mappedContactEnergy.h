/*
  Generic mapped contact adapter.
*/

#pragma once

#include "statefulContactEnergy.h"
#include "stepAwareEnergy.h"
#include "stepDependentEnergy.h"
#include "EigenDef.h"

#include <memory>

namespace pgo
{
namespace NonlinearOptimization
{
struct StepState;
}  // namespace NonlinearOptimization

namespace Contact
{

class MappedContactEnergy : public StatefulContactEnergy
{
public:
  MappedContactEnergy(
    const EigenSupport::MXd &surfaceRestVertices,
    const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
    std::shared_ptr<StatefulContactEnergy> surfacePositionEnergy);

  ContactModelKind contactModelKind() const override;
  double func(EigenSupport::ConstRefVecXd simulationDisplacements) const override;
  void gradient(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient) const override;
  void hessianInPlace(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::SpMatD &simulationHessian) const override;
  void hessianAlloc(EigenSupport::SpMatD &simulationHessian) const override;
  void hessian(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::SpMatD &simulationHessian) const override;
  double func_grad(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient) const override;
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
  int isHessianTopologyFixed() const override;

protected:
  EigenSupport::VXd surfaceDisplacements(EigenSupport::ConstRefVecXd simulationDisplacements) const;
  EigenSupport::VXd surfacePositions(EigenSupport::ConstRefVecXd simulationDisplacements) const;
  void validateSimulationDisplacementSize(EigenSupport::ConstRefVecXd simulationDisplacements) const;
  EigenSupport::VXd pullbackGradient(EigenSupport::ConstRefVecXd surfaceGradient) const;
  void pullbackHessian(const EigenSupport::SpMatD &surfaceHessian, EigenSupport::SpMatD &simulationHessian) const;

  std::shared_ptr<StatefulContactEnergy> surfacePositionEnergy_;
  EigenSupport::VXd surfaceRestPositions_;
  EigenSupport::SpMatD surfaceFromSimulationDispMap_;
  std::vector<int> simulationDofs_;
};

class MappedEvaluationContactEnergy :
  public MappedContactEnergy,
  public NonlinearOptimization::EvaluationStateAwareEnergy,
  public NonlinearOptimization::LineSearchAwareEnergy
{
public:
  using MappedContactEnergy::MappedContactEnergy;

  void prepareEvaluationState(EigenSupport::ConstRefVecXd simulationDisplacements) const override;
  void beginLineSearch(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::ConstRefVecXd simulationStep) const override;
  void endLineSearch() const override;
  double maxValidLineSearchAlpha() const override;
};

class MappedStepAwareContactEnergy final :
  public MappedEvaluationContactEnergy,
  public NonlinearOptimization::StepAwareEnergy,
  public NonlinearOptimization::StepDependentEnergy
{
public:
  using MappedEvaluationContactEnergy::MappedEvaluationContactEnergy;

  void beginStep(const NonlinearOptimization::StepState &state) override;
};

std::shared_ptr<StatefulContactEnergy> makeMappedContactEnergy(
  const EigenSupport::MXd &surfaceRestVertices,
  const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
  std::shared_ptr<StatefulContactEnergy> surfacePositionEnergy);

}  // namespace Contact
}  // namespace pgo
