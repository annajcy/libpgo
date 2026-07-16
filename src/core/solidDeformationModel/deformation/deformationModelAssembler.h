/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "deformation/deformationModelManager.h"
#include "deformation/deformationModelAssemblerCacheData.h"
#include "formulations/dof/dofLayout.h"
#include "formulations/formulation/formulation.h"
#include "material/fields/parameterField.h"
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
    std::shared_ptr<OptimizableField> elasticParamField,
    std::shared_ptr<OptimizableField> plasticParamField,
    const double *elementWeights = nullptr);
  virtual ~DeformationModelAssembler();

  double computeEnergy(const double *x) const;
  MaterialMaxStepObservation computeMaxStepObservation(const double *x, const double *dx) const;
  double computeMaxStepSize(const double *x, const double *dx) const;
  void computeGradient(const double *x, double *grad) const;
  void computeHessian(const double *x, EigenSupport::SpMatD &hess) const;

  void compute_df_da(const double *x, EigenSupport::SpMatD &hess) const;
  void compute_df_db(const double *x, EigenSupport::SpMatD &hess) const;
  int getNumElasticGlobalParams() const;
  int getNumPlasticGlobalParams() const;
  EigenSupport::VXd getElasticParameterSnapshot() const;
  EigenSupport::VXd getPlasticParameterSnapshot() const;
  void setElasticValues(EigenSupport::ConstRefVecXd values);
  void setPlasticValues(EigenSupport::ConstRefVecXd values);
  std::shared_ptr<OptimizableField> elasticParameterFieldPtr() const { return elasticParamField_; }
  std::shared_ptr<OptimizableField> plasticParameterFieldPtr() const { return plasticParamField_; }
  const OptimizableField &elasticParameterField() const { return *elasticParamField_; }
  const OptimizableField &plasticParameterField() const { return *plasticParamField_; }
  const EigenSupport::SpMatD &getPlasticHessianTemplate() const { return d2Eda2Template; }
  const EigenSupport::SpMatD &getElasticHessianTemplate() const { return d2Edb2Template; }
  const EigenSupport::SpMatD &getPlasticElasticHessianTemplate() const { return d2EdadbTemplate; }
  void computePlasticGradient(const double *x, double *grad) const;
  void computePlasticHessian(const double *x, EigenSupport::SpMatD &hess) const;
  void computeElasticGradient(const double *x, double *grad) const;
  void computeElasticHessian(const double *x, EigenSupport::SpMatD &hess) const;
  void computePlasticElasticHessian(const double *x, EigenSupport::SpMatD &hess) const;

  void computeVonMisesStresses(const double *x, double *elementStresses) const;
  void computeMaxStrains(const double *x, double *elementStrain) const;

  int getNumDOFs() const { return numDOFs; }

  const DeformationModelManager &getDeformationModelManager() const { return *deformationModelManager; }
  DeformationModelManager &getDeformationModelManager() { return *deformationModelManager; }
  const DofLayout &getDofLayout() const { return *dofLayout; }
  const EigenSupport::VXd &getRestPosition() const { return restDofs_; }
  const EigenSupport::SpMatD &getHessianTemplate() const { return KTemplate; }
  const EigenSupport::SpMatD &get_dfda_Template() const { return dfdaTemplate; }
  const EigenSupport::SpMatD &get_dfdb_Template() const { return dfdbTemplate; }

  int getNumElasticParams() const { return numElasticParams_; }
  int getNumPlasticParams() const { return numPlasticParams_; }

protected:
  std::shared_ptr<DeformationModelManager> deformationModelManager;
  std::shared_ptr<const DofLayout> dofLayout;
  EigenSupport::VXd restDofs_;
  std::unique_ptr<DeformationModelAssemblerCacheData> data;

  std::shared_ptr<OptimizableField> elasticParamField_;
  std::shared_ptr<OptimizableField> plasticParamField_;

  int numDOFs, nele, neleVtx, localDOFs;
  int numElasticParams_ = 0;
  int numPlasticParams_ = 0;
  int numElasticLocalParams_ = 0;
  int numPlasticLocalParams_ = 0;

  EigenSupport::SpMatD KTemplate, dfdaTemplate, dfdbTemplate, d2Eda2Template, d2Edb2Template, d2EdadbTemplate;

  std::vector<std::vector<HessianBlockOffset>> elementKBlockOffsets;
  std::vector<DynamicIndexMatrix> element_dfda_InverseIndices, element_dfdb_InverseIndices;
  std::vector<DynamicIndexMatrix> element_d2Eda2_InverseIndices, element_d2Edb2_InverseIndices, element_d2Edadb_InverseIndices;

  std::vector<double> elementWeights;
  std::vector<const DeformationModel *> femModels;

  const int enableSanityCheck = 1;

private:
  struct PreparedElement
  {
    const DeformationModel *model = nullptr;
    DeformationModel::CacheData *cache = nullptr;
  };

  // Build a mixed sparsity template + inverse-index map for d²E/dx dp (df/dp).
  // Shared by the dfdb and dfda template construction in the constructor.
  void buildMixedSparsityTemplate(
    int numLocalParams,
    int numGlobalParams,
    const std::function<int(int, int)> &paramGlobalCol,
    EigenSupport::SpMatD &tmpl,
    std::vector<DynamicIndexMatrix> &inverseIndices,
    std::vector<EigenSupport::TripletD> &entries);

  // Generic df/dparam assembly loop — shared by compute_df_da and compute_df_db.
  // computeLocal is a pointer to DeformationModel::compute_d2E_dxda (or _dxdb).
  void assembleDfDparam(
    const double *x,
    int numMaterialParams,
    int numLocalParams,
    const OptimizableField *paramField,
    const std::vector<DynamicIndexMatrix> &inverseIndices,
    void (DeformationModel::*computeLocal)(const DeformationModel::CacheData *, double *) const,
    EigenSupport::SpMatD &hess,
    const char *label) const;

  // Gather local displacement DOFs and externally computed material parameter values,
  // then prepare the element cache.
  PreparedElement gatherAndPrepare(
    int ele, const double *x,
    DeformationModelAssemblerCacheData::ThreadScratch &scratch) const;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
