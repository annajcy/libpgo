/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "energy/potentialEnergy.h"
#include "material/fields/materialParameters.h"

#include <tbb/enumerable_thread_specific.h>

#include <memory>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{
class DeformationModelAssembler;

class DeformationModelEnergy : public NonlinearOptimization::PotentialEnergy
{
public:
  // offset only controls the local-to-global indices returned by getDOFs().
  // Evaluation methods always consume local vectors of size getNumDOFs().
  DeformationModelEnergy(
    std::unique_ptr<DeformationModelAssembler> fma,
    std::shared_ptr<MaterialParameters> materialParameters,
    int offset = 0,
    bool enableMaterialMaxStep = true);
  virtual ~DeformationModelEnergy();

  const DeformationModelAssembler &assembler() const { return *forceModelAssembler; }
  DeformationModelAssembler &assembler() { return *forceModelAssembler; }
  std::shared_ptr<MaterialParameters> materialParameters() { return materialParameters_; }
  std::shared_ptr<const MaterialParameters> materialParameters() const { return materialParameters_; }

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

  double func(EigenSupport::ConstRefVecXd x, MaterialStateView state) const;
  void gradient(
    EigenSupport::ConstRefVecXd x,
    MaterialStateView state,
    EigenSupport::RefVecXd grad) const;
  void hessianInPlace(
    EigenSupport::ConstRefVecXd x,
    MaterialStateView state,
    EigenSupport::SpMatD &hess) const;

  // Material derivatives of E(u, p, e), where u is displacement, p is the
  // plastic field, and e is the elastic field. Overloads without an explicit
  // MaterialStateView use the currently committed parameter fields.
  void compute_dE_dp(EigenSupport::ConstRefVecXd displacement, EigenSupport::RefVecXd grad) const;
  void compute_dE_dp(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::RefVecXd grad) const;
  void compute_dE_de(EigenSupport::ConstRefVecXd displacement, EigenSupport::RefVecXd grad) const;
  void compute_dE_de(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::RefVecXd grad) const;
  void compute_d2E_dp2(EigenSupport::ConstRefVecXd displacement, EigenSupport::SpMatD &hess) const;
  void compute_d2E_dp2(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::SpMatD &hess) const;
  void compute_d2E_de2(EigenSupport::ConstRefVecXd displacement, EigenSupport::SpMatD &hess) const;
  void compute_d2E_de2(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::SpMatD &hess) const;
  void compute_d2E_dpde(EigenSupport::ConstRefVecXd displacement, EigenSupport::SpMatD &hess) const;
  void compute_d2E_dpde(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::SpMatD &hess) const;
  void compute_d2E_dudp(EigenSupport::ConstRefVecXd displacement, EigenSupport::SpMatD &mixedHessian) const;
  void compute_d2E_dudp(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::SpMatD &mixedHessian) const;
  void compute_d2E_dude(EigenSupport::ConstRefVecXd displacement, EigenSupport::SpMatD &mixedHessian) const;
  void compute_d2E_dude(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::SpMatD &mixedHessian) const;
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
  std::shared_ptr<MaterialParameters> materialParameters_;
  std::vector<int> allDOFs;
  std::unique_ptr<EigenSupport::VXd> restDofs;
  EigenSupport::VXd vertexRestPositions;
  mutable tbb::enumerable_thread_specific<EigenSupport::VXd> absolutePositionScratch_;
  bool enableMaterialMaxStep_ = true;

private:
  EigenSupport::VXd &absolutePositionScratch() const;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
