/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "energy/potentialEnergy.h"
#include "material/runtime/materialState.h"

#include <memory>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{
class DeformationModelAssembler;
class Formulation;
class MaterialBinding;
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
    const SimulationMesh &mesh,
    const MaterialBinding &materialBinding,
    const Formulation &formulation,
    const DeformationModelOptions &options = {});

  ~DeformationEnergyOperator();

  const DeformationModelAssembler &assembler() const { return *forceModelAssembler; }

  // Full generalized rest state. This is not necessarily one position per
  // mesh vertex (e.g. Hermite also stores derivative DOFs).
  const EigenSupport::VXd &getRestDofs() const;
  // Interleaved xyz positions of the actual mesh vertices, independent of
  // the formulation's generalized DOF layout.
  const EigenSupport::VXd &getVertexRestPositions() const { return vertexRestPositions; }
  int getNumVertices() const;
  int getNumElements() const;

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
  EigenSupport::VXd vertexRestPositions;
  // Mutable evaluation storage. One operator instance does not support
  // overlapping/concurrent evaluations.
  mutable EigenSupport::VXd absolutePositionScratch_;
  bool enableMaterialMaxStep_ = true;

private:
  struct BuildComponents
  {
    std::unique_ptr<DeformationModelAssembler> assembler;
    EigenSupport::VXd vertexRestPositions;
    bool enableMaterialMaxStep = true;
    int dofOffset = 0;
  };

  static BuildComponents build(
    const SimulationMesh &mesh,
    const MaterialBinding &materialBinding,
    const Formulation &formulation,
    const DeformationModelOptions &options);
  explicit DeformationEnergyOperator(BuildComponents components);

};
}  // namespace SolidDeformationModel
}  // namespace pgo
