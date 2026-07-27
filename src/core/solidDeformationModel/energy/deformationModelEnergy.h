/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "energy/potentialEnergy.h"
#include "material/runtime/optimizableParameters.h"

#include <tbb/enumerable_thread_specific.h>

#include <memory>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{
class DeformationModelAssembler;
class Formulation;
class MaterialAssignment;

struct DeformationModelOptions
{
  bool projectHessianPSD = true;
  bool enableMaterialMaxStep = true;
  int dofOffset = 0;
  EigenSupport::VXd elementWeights;
};

class DeformationModelEnergy : public NonlinearOptimization::PotentialEnergy
{
public:
  DeformationModelEnergy(
    std::shared_ptr<const MaterialAssignment> assignment,
    const Formulation &formulation,
    const DeformationModelOptions &options = {});
  virtual ~DeformationModelEnergy();

  const DeformationModelAssembler &assembler() const { return *forceModelAssembler; }
  DeformationModelAssembler &assembler() { return *forceModelAssembler; }
  std::shared_ptr<OptimizableParameters> optimizableParameters() { return optimizableParameters_; }
  std::shared_ptr<const OptimizableParameters> optimizableParameters() const { return optimizableParameters_; }

  // Full generalized rest state. This is not necessarily one position per
  // mesh vertex (e.g. Hermite also stores derivative DOFs).
  const EigenSupport::VXd &getRestDofs() const { return *restDofs; }
  // Interleaved xyz positions of the actual mesh vertices, independent of
  // the formulation's generalized DOF layout.
  const EigenSupport::VXd &getVertexRestPositions() const { return vertexRestPositions; }

  virtual double func(EigenSupport::ConstRefVecXd x) const override;
  virtual void gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const override;
  virtual void hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const override;
  virtual void hessianAlloc(EigenSupport::SpMatD &hess) const override;

  double func(EigenSupport::ConstRefVecXd x, OptimizableParameterEvaluationView state) const;
  void gradient(
    EigenSupport::ConstRefVecXd x,
    OptimizableParameterEvaluationView state,
    EigenSupport::RefVecXd grad) const;
  void hessianInPlace(
    EigenSupport::ConstRefVecXd x,
    OptimizableParameterEvaluationView state,
    EigenSupport::SpMatD &hess) const;

  // Material derivatives of E(u, p, e), where u is displacement, p is the
  // plastic field, and e is the elastic field. Overloads without an explicit
  // OptimizableParameterEvaluationView use the currently committed parameter fields.
  void compute_dE_dp(EigenSupport::ConstRefVecXd displacement, EigenSupport::RefVecXd grad) const;
  void compute_dE_dp(EigenSupport::ConstRefVecXd displacement, OptimizableParameterEvaluationView state, EigenSupport::RefVecXd grad) const;
  void compute_dE_de(EigenSupport::ConstRefVecXd displacement, EigenSupport::RefVecXd grad) const;
  void compute_dE_de(EigenSupport::ConstRefVecXd displacement, OptimizableParameterEvaluationView state, EigenSupport::RefVecXd grad) const;
  void compute_d2E_dp2(EigenSupport::ConstRefVecXd displacement, EigenSupport::SpMatD &hess) const;
  void compute_d2E_dp2(EigenSupport::ConstRefVecXd displacement, OptimizableParameterEvaluationView state, EigenSupport::SpMatD &hess) const;
  void compute_d2E_de2(EigenSupport::ConstRefVecXd displacement, EigenSupport::SpMatD &hess) const;
  void compute_d2E_de2(EigenSupport::ConstRefVecXd displacement, OptimizableParameterEvaluationView state, EigenSupport::SpMatD &hess) const;
  void compute_d2E_dpde(EigenSupport::ConstRefVecXd displacement, EigenSupport::SpMatD &hess) const;
  void compute_d2E_dpde(EigenSupport::ConstRefVecXd displacement, OptimizableParameterEvaluationView state, EigenSupport::SpMatD &hess) const;
  void compute_d2E_dudp(EigenSupport::ConstRefVecXd displacement, EigenSupport::SpMatD &mixedHessian) const;
  void compute_d2E_dudp(EigenSupport::ConstRefVecXd displacement, OptimizableParameterEvaluationView state, EigenSupport::SpMatD &mixedHessian) const;
  void compute_d2E_dude(EigenSupport::ConstRefVecXd displacement, EigenSupport::SpMatD &mixedHessian) const;
  void compute_d2E_dude(EigenSupport::ConstRefVecXd displacement, OptimizableParameterEvaluationView state, EigenSupport::SpMatD &mixedHessian) const;
  void computeVonMisesStresses(EigenSupport::ConstRefVecXd displacement, EigenSupport::RefVecXd elementStresses) const;
  void computeMaxStrains(EigenSupport::ConstRefVecXd displacement, EigenSupport::RefVecXd elementStrains) const;
  virtual void getDOFs(std::vector<int> &dofs) const override { dofs = this->allDOFs; }
  virtual int getNumDOFs() const override { return (int)allDOFs.size(); }

  virtual NonlinearOptimization::EnergyStateKind stateKind() const override { return NonlinearOptimization::EnergyStateKind::Displacement; }

  virtual NonlinearOptimization::StepConstraint computeMaxStepLimit(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx, StepConstraintSink *sink = nullptr) const override;

  void setEnableMaterialMaxStep(bool enable) { enableMaterialMaxStep_ = enable; }
  bool isMaterialMaxStepEnabled() const { return enableMaterialMaxStep_; }

protected:
  std::unique_ptr<DeformationModelAssembler> forceModelAssembler;
  std::shared_ptr<OptimizableParameters> optimizableParameters_;
  std::vector<int> allDOFs;
  std::unique_ptr<EigenSupport::VXd> restDofs;
  EigenSupport::VXd vertexRestPositions;
  mutable tbb::enumerable_thread_specific<EigenSupport::VXd> absolutePositionScratch_;
  bool enableMaterialMaxStep_ = true;

private:
  struct BuildComponents
  {
    std::unique_ptr<DeformationModelAssembler> assembler;
    std::shared_ptr<OptimizableParameters> optimizableParameters;
    bool enableMaterialMaxStep = true;
    int dofOffset = 0;
  };

  static BuildComponents build(
    std::shared_ptr<const MaterialAssignment> assignment,
    const Formulation &formulation,
    const DeformationModelOptions &options);

  explicit DeformationModelEnergy(BuildComponents components);

  EigenSupport::VXd &absolutePositionScratch() const;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
