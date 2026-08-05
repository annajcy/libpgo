/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "deformation/deformationModelManager.h"
#include "deformation/deformationModelAssemblerElementWorkspace.h"
#include "formulations/dof/dofLayout.h"
#include "formulations/formulation/formulation.h"
#include "material/runtime/materialState.h"
#include "EigenDef.h"
#include <cstddef>
#include <functional>
#include <memory>
#include <span>

namespace pgo
{
namespace SolidDeformationModel
{

class DeformationModelAssembler
{
public:
  struct MaterialMaxStepObservation
  {
    double alpha = 1.0;
    bool hasIllegalInitialState = false;
    int limitingElementId = -1;
    int limitingLocationId = -1;
  };

  DeformationModelAssembler(std::shared_ptr<const DeformationModelManager> dm,
    const Formulation &formulation,
    std::shared_ptr<const OptimizableParameterField> elasticField,
    std::shared_ptr<const OptimizableParameterField> plasticField,
    std::span<const double> elementWeights = {});
  virtual ~DeformationModelAssembler();

  MaterialMaxStepObservation computeMaxStepObservation(std::span<const double> x, std::span<const double> dx) const;
  double computeMaxStepSize(std::span<const double> x, std::span<const double> dx) const;

  double compute_E(std::span<const double> x, MaterialStateView state) const;
  void compute_dE_dx(
    std::span<const double> x, MaterialStateView state,
    EigenSupport::RefVecXd grad) const;
  void compute_d2E_dx2(std::span<const double> x, MaterialStateView state, EigenSupport::SpMatD &hess) const;

  // E(u, p, e): u = displacement, p = plastic DOFs, e = elastic DOFs.
  // absolutePositions is rest + u, whose derivative with respect to u is I.
  void compute_d2E_dudp(
    std::span<const double> absolutePositions, MaterialStateView state,
    EigenSupport::SpMatD &mixedHessian) const;
  void compute_d2E_dude(
    std::span<const double> absolutePositions, MaterialStateView state,
    EigenSupport::SpMatD &mixedHessian) const;
  void computePlasticMaterialVJP(
    std::span<const double> absolutePositions,
    std::span<const double> adjoint,
    MaterialStateView state,
    std::span<double> output) const;
  void computeElasticMaterialVJP(
    std::span<const double> absolutePositions,
    std::span<const double> adjoint,
    MaterialStateView state,
    std::span<double> output) const;
  int getNumElasticGlobalParams() const;
  int getNumPlasticGlobalParams() const;
  const std::shared_ptr<const OptimizableParameterField> &elasticField() const
  {
    return elasticField_;
  }
  const std::shared_ptr<const OptimizableParameterField> &plasticField() const
  {
    return plasticField_;
  }
  const EigenSupport::SpMatD &d2E_dp2_template() const { return d2E_dp2Cache_.matrixTemplate; }
  const EigenSupport::SpMatD &d2E_de2_template() const { return d2E_de2Cache_.matrixTemplate; }
  const EigenSupport::SpMatD &d2E_dpde_template() const { return d2E_dpdeCache_.matrixTemplate; }
  const EigenSupport::SpMatD &d2E_dudp_template() const { return d2E_dudpCache_.matrixTemplate; }
  const EigenSupport::SpMatD &d2E_dude_template() const { return d2E_dudeCache_.matrixTemplate; }
  void compute_dE_dp(
    std::span<const double> x, MaterialStateView state,
    EigenSupport::RefVecXd grad) const;
  void compute_d2E_dp2(std::span<const double> x, MaterialStateView state, EigenSupport::SpMatD &hess) const;
  void compute_dE_de(
    std::span<const double> x, MaterialStateView state,
    EigenSupport::RefVecXd grad) const;
  void compute_d2E_de2(std::span<const double> x, MaterialStateView state, EigenSupport::SpMatD &hess) const;
  void compute_d2E_dpde(std::span<const double> x, MaterialStateView state, EigenSupport::SpMatD &hess) const;

  void computeVonMisesStresses(std::span<const double> x, MaterialStateView state, std::span<double> elementStresses) const;
  void computeMaxStrains(std::span<const double> x, MaterialStateView state, std::span<double> elementStrain) const;

  int getNumDOFs() const { return numDOFs; }

  const DeformationModelManager &getDeformationModelManager() const { return *deformationModelManager; }
  const DofLayout &getDofLayout() const { return *dofLayout; }
  // The formulation rest state contains every global DOF.  For Hermite
  // formulations this includes derivative DOFs in addition to vertex
  // positions, so it must not be interpreted as a (numVertices x 3) array.
  const EigenSupport::VXd &getRestDofs() const { return restDofs_; }
  const EigenSupport::SpMatD &getHessianTemplate() const { return KTemplate; }
  int getNumElasticParams() const { return numElasticParams_; }
  int getNumPlasticParams() const { return numPlasticParams_; }

protected:
  struct SparseAssemblyCache
  {
    EigenSupport::SpMatD matrixTemplate;
    std::vector<DynamicIndexMatrix> elementInverseIndices;
  };

  std::shared_ptr<const DeformationModelManager> deformationModelManager;
  std::shared_ptr<const DofLayout> dofLayout;
  EigenSupport::VXd restDofs_;
  mutable std::vector<DeformationModelAssemblerElementWorkspace> elementWorkspaces_;

  std::shared_ptr<const OptimizableParameterField> elasticField_;
  std::shared_ptr<const OptimizableParameterField> plasticField_;

  int numDOFs, nele, neleVtx, localDOFs;
  int numElasticParams_ = 0;
  int numPlasticParams_ = 0;
  int numElasticLocalParams_ = 0;
  int numPlasticLocalParams_ = 0;

  EigenSupport::SpMatD KTemplate;
  SparseAssemblyCache d2E_dudpCache_;
  SparseAssemblyCache d2E_dudeCache_;
  SparseAssemblyCache d2E_dp2Cache_;
  SparseAssemblyCache d2E_de2Cache_;
  SparseAssemblyCache d2E_dpdeCache_;

  std::vector<std::vector<HessianBlockOffset>> elementKBlockOffsets;

  std::vector<double> elementWeights;
  std::vector<std::reference_wrapper<const DeformationModel>> femModels;

  const int enableSanityCheck = 1;

private:
  // Build a mixed sparsity template + inverse-index map for d²E/dx dp.
  // Shared by the displacement-elastic and displacement-plastic templates.
  void buildMixedSparsityTemplate(
    int numLocalParams,
    int numGlobalParams,
    const std::function<int(int, int)> &paramGlobalCol,
    SparseAssemblyCache &cache,
    std::vector<EigenSupport::TripletD> &entries);

  // Generic d²E/(du dq) assembly loop, where q is p or e.
  void assemble_d2E_dudq(
    std::span<const double> absolutePositions,
    MaterialStateView state,
    int numMaterialParams,
    int numLocalParams,
    const OptimizableParameterField &paramBlock,
    const std::vector<DynamicIndexMatrix> &inverseIndices,
    void (DeformationModelEvaluator::*computeLocal)(
      EigenSupport::RefMatXd, int) const,
    EigenSupport::SpMatD &mixedHessian,
    const char *label) const;

  // Directly accumulates (d²E / du dq)^T adjoint without assembling the
  // global displacement-material mixed Hessian.
  void assembleMaterialVJP(
    std::span<const double> absolutePositions,
    std::span<const double> adjoint,
    MaterialStateView state,
    int numMaterialParams,
    int numLocalParams,
    const OptimizableParameterField &paramBlock,
    void (DeformationModelEvaluator::*computeLocal)(
      EigenSupport::RefMatXd, int) const,
    std::span<double> output,
    const char *label) const;

  // Gather local displacement DOFs and externally computed optimizable parameter values,
  // then prepare the element evaluator.
  DeformationModelEvaluator &gatherAndPrepare(
    int ele, std::span<const double> x, const MaterialStateView &state,
    DeformationModelAssemblerElementWorkspace &scratch) const;

  void validateMaterialState(
    const MaterialStateView &state) const;
  void validatePositionSpan(std::span<const double> x, const char *label) const;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
