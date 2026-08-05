/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "energy/potentialEnergy.h"
#include "material/runtime/materialState.h"

#include <tbb/enumerable_thread_specific.h>

#include <memory>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{
class DeformationModelAssembler;
class Formulation;
class MaterialBinding;
class MaterialAssignment;
class SimulationMesh;

struct DeformationModelOptions
{
  bool projectHessianPSD = true;
  bool enableMaterialMaxStep = true;
  int dofOffset = 0;
  EigenSupport::VXd elementWeights;
};

class DeformationEnergyOperator
{
public:
  DeformationEnergyOperator(
    std::shared_ptr<const SimulationMesh> mesh,
    std::shared_ptr<const MaterialBinding> materialBinding,
    const Formulation &formulation,
    const DeformationModelOptions &options = {});

  DeformationEnergyOperator(
    std::shared_ptr<const MaterialAssignment> assignment,
    const Formulation &formulation,
    const DeformationModelOptions &options = {});
  ~DeformationEnergyOperator();

  const DeformationModelAssembler &assembler() const { return *forceModelAssembler; }
  DeformationModelAssembler &assembler() { return *forceModelAssembler; }

  // Full generalized rest state. This is not necessarily one position per
  // mesh vertex (e.g. Hermite also stores derivative DOFs).
  const EigenSupport::VXd &getRestDofs() const { return *restDofs; }
  // Interleaved xyz positions of the actual mesh vertices, independent of
  // the formulation's generalized DOF layout.
  const EigenSupport::VXd &getVertexRestPositions() const { return vertexRestPositions; }

  double func(EigenSupport::ConstRefVecXd x, MaterialStateView state) const;
  void gradient(
    EigenSupport::ConstRefVecXd x,
    MaterialStateView state,
    EigenSupport::RefVecXd grad) const;
  void hessianInPlace(
    EigenSupport::ConstRefVecXd x,
    MaterialStateView state,
    EigenSupport::SpMatD &hess) const;

  void hessianAlloc(EigenSupport::SpMatD &hess) const;

  // Material derivatives of E(u, p, e), where u is displacement, p is the
  // plastic field, and e is the elastic field. Material state is always
  // explicit; the operator owns no committed material values.
  void compute_dE_dp(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::RefVecXd grad) const;
  void compute_dE_de(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::RefVecXd grad) const;
  void compute_d2E_dp2(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::SpMatD &hess) const;
  void compute_d2E_de2(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::SpMatD &hess) const;
  void compute_d2E_dpde(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::SpMatD &hess) const;
  void compute_d2E_dudp(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::SpMatD &mixedHessian) const;
  void compute_d2E_dude(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::SpMatD &mixedHessian) const;
  void computePlasticMaterialVJP(
    EigenSupport::ConstRefVecXd displacement,
    MaterialStateView state,
    EigenSupport::ConstRefVecXd adjoint,
    EigenSupport::RefVecXd output) const;
  void computeElasticMaterialVJP(
    EigenSupport::ConstRefVecXd displacement,
    MaterialStateView state,
    EigenSupport::ConstRefVecXd adjoint,
    EigenSupport::RefVecXd output) const;
  void computeVonMisesStresses(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::RefVecXd elementStresses) const;
  void computeMaxStrains(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::RefVecXd elementStrains) const;
  void getDOFs(std::vector<int> &dofs) const { dofs = this->allDOFs; }
  int getNumDOFs() const { return (int)allDOFs.size(); }

  NonlinearOptimization::StepConstraint computeMaxStepLimit(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx, NonlinearOptimization::StepConstraintSink *sink = nullptr) const;

  void setEnableMaterialMaxStep(bool enable) { enableMaterialMaxStep_ = enable; }
  bool isMaterialMaxStepEnabled() const { return enableMaterialMaxStep_; }

protected:
  std::unique_ptr<DeformationModelAssembler> forceModelAssembler;
  std::vector<int> allDOFs;
  std::unique_ptr<EigenSupport::VXd> restDofs;
  EigenSupport::VXd vertexRestPositions;
  mutable tbb::enumerable_thread_specific<EigenSupport::VXd> absolutePositionScratch_;
  bool enableMaterialMaxStep_ = true;

private:
  struct BuildComponents
  {
    std::unique_ptr<DeformationModelAssembler> assembler;
    bool enableMaterialMaxStep = true;
    int dofOffset = 0;
  };

  static BuildComponents build(
    std::shared_ptr<const SimulationMesh> mesh,
    std::shared_ptr<const MaterialBinding> materialBinding,
    const Formulation &formulation,
    const DeformationModelOptions &options);
  static BuildComponents build(
    std::shared_ptr<const MaterialAssignment> assignment,
    const Formulation &formulation,
    const DeformationModelOptions &options);

  explicit DeformationEnergyOperator(BuildComponents components);

  EigenSupport::VXd &absolutePositionScratch() const;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
