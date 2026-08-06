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
  // Combined evaluations: the position buffer is filled once and every
  // element prepares its geometry/material state once before all requested
  // quantities are assembled. These are the recommended hot-path entry
  // points (e.g. Newton funcGradientHessian).
  double funcGradient(
    EigenSupport::ConstRefVecXd displacement,
    MaterialStateView state,
    EigenSupport::RefVecXd grad) const;
  double funcGradientHessian(
    EigenSupport::ConstRefVecXd displacement,
    MaterialStateView state,
    EigenSupport::RefVecXd grad,
    EigenSupport::SpMatD &hess) const;
  void gradientHessian(
    EigenSupport::ConstRefVecXd displacement,
    MaterialStateView state,
    EigenSupport::RefVecXd grad,
    EigenSupport::SpMatD &hess) const;

  // Material derivatives of E(u, p, e), where u is displacement, p is the
  // plastic field, and e is the elastic field. Material state is always
  // explicit; the operator owns no committed material values.
  void computePlasticGradient(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::RefVecXd grad) const;
  void computeElasticGradient(EigenSupport::ConstRefVecXd displacement, MaterialStateView state, EigenSupport::RefVecXd grad) const;
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

protected:
  std::unique_ptr<DeformationModelAssembler> forceModelAssembler;
  std::vector<int> allDOFs;
  EigenSupport::VXd vertexRestPositions;
  // Mutable evaluation storage. One operator instance does not support
  // overlapping/concurrent evaluations.
  mutable EigenSupport::VXd absolutePositionScratch_;

private:
  struct BuildComponents
  {
    std::unique_ptr<DeformationModelAssembler> assembler;
    EigenSupport::VXd vertexRestPositions;
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
