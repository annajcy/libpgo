/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "deformation/deformationModelManager.h"
#include "deformation/deformationModelAssemblerCacheData.h"
#include "formulations/dof/dofLayout.h"
#include "formulations/formulation/formulation.h"
#include "material/core/materialParameters.h"
#include "EigenDef.h"
#include <cstddef>
#include <functional>
#include <memory>

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

  DeformationModelAssembler(std::shared_ptr<DeformationModelManager> dm,
    const Formulation &formulation,
    std::shared_ptr<const MaterialParameterSpace> materialParameterSpace,
    const double *elementWeights = nullptr);
  virtual ~DeformationModelAssembler();

  MaterialMaxStepObservation computeMaxStepObservation(const double *x, const double *dx) const;
  double computeMaxStepSize(const double *x, const double *dx) const;

  double computeEnergy(const double *x, MaterialParameterEvaluationView state) const;
  void computeGradient(const double *x, MaterialParameterEvaluationView state, double *grad) const;
  void computeHessian(const double *x, MaterialParameterEvaluationView state, EigenSupport::SpMatD &hess) const;

  // E(u, p, e): u = displacement, p = plastic DOFs, e = elastic DOFs.
  // absolutePositions is rest + u, whose derivative with respect to u is I.
  void compute_d2E_dudp(
    const double *absolutePositions, MaterialParameterEvaluationView state,
    EigenSupport::SpMatD &mixedHessian) const;
  void compute_d2E_dude(
    const double *absolutePositions, MaterialParameterEvaluationView state,
    EigenSupport::SpMatD &mixedHessian) const;
  int getNumElasticGlobalParams() const;
  int getNumPlasticGlobalParams() const;
  std::shared_ptr<const MaterialParameterSpace> materialParameterSpace() const
  {
    return materialParameterSpace_;
  }
  const EigenSupport::SpMatD &d2E_dp2_template() const { return d2E_dp2Template; }
  const EigenSupport::SpMatD &d2E_de2_template() const { return d2E_de2Template; }
  const EigenSupport::SpMatD &d2E_dpde_template() const { return d2E_dpdeTemplate; }
  const EigenSupport::SpMatD &d2E_dudp_template() const
  {
    return d2E_dudpTemplate;
  }
  const EigenSupport::SpMatD &d2E_dude_template() const
  {
    return d2E_dudeTemplate;
  }
  void compute_dE_dp(const double *x, MaterialParameterEvaluationView state, double *grad) const;
  void compute_d2E_dp2(const double *x, MaterialParameterEvaluationView state, EigenSupport::SpMatD &hess) const;
  void compute_dE_de(const double *x, MaterialParameterEvaluationView state, double *grad) const;
  void compute_d2E_de2(const double *x, MaterialParameterEvaluationView state, EigenSupport::SpMatD &hess) const;
  void compute_d2E_dpde(const double *x, MaterialParameterEvaluationView state, EigenSupport::SpMatD &hess) const;

  void computeVonMisesStresses(const double *x, MaterialParameterEvaluationView state, double *elementStresses) const;
  void computeMaxStrains(const double *x, MaterialParameterEvaluationView state, double *elementStrain) const;

  int getNumDOFs() const { return numDOFs; }

  const DeformationModelManager &getDeformationModelManager() const { return *deformationModelManager; }
  DeformationModelManager &getDeformationModelManager() { return *deformationModelManager; }
  const DofLayout &getDofLayout() const { return *dofLayout; }
  // The formulation rest state contains every global DOF.  For Hermite
  // formulations this includes derivative DOFs in addition to vertex
  // positions, so it must not be interpreted as a (numVertices x 3) array.
  const EigenSupport::VXd &getRestDofs() const { return restDofs_; }
  const EigenSupport::SpMatD &getHessianTemplate() const { return KTemplate; }
  int getNumElasticParams() const { return numElasticParams_; }
  int getNumPlasticParams() const { return numPlasticParams_; }

protected:
  std::shared_ptr<DeformationModelManager> deformationModelManager;
  std::shared_ptr<const DofLayout> dofLayout;
  EigenSupport::VXd restDofs_;
  std::unique_ptr<DeformationModelAssemblerCacheData> data;

  std::shared_ptr<const MaterialParameterSpace> materialParameterSpace_;

  int numDOFs, nele, neleVtx, localDOFs;
  int numElasticParams_ = 0;
  int numPlasticParams_ = 0;
  int numElasticLocalParams_ = 0;
  int numPlasticLocalParams_ = 0;

  EigenSupport::SpMatD KTemplate;
  EigenSupport::SpMatD d2E_dudpTemplate;
  EigenSupport::SpMatD d2E_dudeTemplate;
  EigenSupport::SpMatD d2E_dp2Template;
  EigenSupport::SpMatD d2E_de2Template;
  EigenSupport::SpMatD d2E_dpdeTemplate;

  std::vector<std::vector<HessianBlockOffset>> elementKBlockOffsets;
  std::vector<DynamicIndexMatrix> element_d2E_dudp_InverseIndices;
  std::vector<DynamicIndexMatrix> element_d2E_dude_InverseIndices;
  std::vector<DynamicIndexMatrix> element_d2E_dp2_InverseIndices;
  std::vector<DynamicIndexMatrix> element_d2E_de2_InverseIndices;
  std::vector<DynamicIndexMatrix> element_d2E_dpde_InverseIndices;

  std::vector<double> elementWeights;
  std::vector<const DeformationModel *> femModels;

  const int enableSanityCheck = 1;

private:
  struct PreparedElement
  {
    const DeformationModel *model = nullptr;
    DeformationModel::CacheData *cache = nullptr;
  };

  // Build a mixed sparsity template + inverse-index map for d²E/dx dp.
  // Shared by the displacement-elastic and displacement-plastic templates.
  void buildMixedSparsityTemplate(
    int numLocalParams,
    int numGlobalParams,
    const std::function<int(int, int)> &paramGlobalCol,
    EigenSupport::SpMatD &tmpl,
    std::vector<DynamicIndexMatrix> &inverseIndices,
    std::vector<EigenSupport::TripletD> &entries);

  // Generic d²E/(du dq) assembly loop, where q is p or e.
  void assemble_d2E_dudq(
    const double *absolutePositions,
    MaterialParameterEvaluationView state,
    int numMaterialParams,
    int numLocalParams,
    const MaterialParameterField &paramBlock,
    const std::vector<DynamicIndexMatrix> &inverseIndices,
    void (DeformationModel::*computeLocal)(
      const DeformationModel::CacheData *, double *, int) const,
    EigenSupport::SpMatD &mixedHessian,
    const char *label) const;

  // Gather local displacement DOFs and externally computed material parameter values,
  // then prepare the element cache.
  PreparedElement gatherAndPrepare(
    int ele, const double *x, MaterialParameterEvaluationView state,
    DeformationModelAssemblerCacheData::ElementScratch &scratch) const;

  void validateMaterialParameterSnapshot(MaterialParameterEvaluationView state) const;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
