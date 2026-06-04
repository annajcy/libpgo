/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "deformationModelManager.h"
#include "deformationModelAssemblerCacheData.h"
#include "formulations/dof/dofLayout.h"
#include "formulations/parameters/parameterField.h"
#include "EigenDef.h"

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

  DeformationModelAssembler(std::unique_ptr<DeformationModelManager> dm,
    const double *elementWeights = nullptr);
  virtual ~DeformationModelAssembler();

  double computeEnergy(const double *x) const;
  MaterialMaxStepObservation computeMaxStepObservation(const double *x, const double *dx) const;
  double computeMaxStepSize(const double *x, const double *dx) const;
  void computeGradient(const double *x, double *grad) const;
  void computeHessian(const double *x, EigenSupport::SpMatD &hess) const;

  void compute_df_da(const double *x, EigenSupport::SpMatD &hess) const;
  void compute_df_db(const double *x, EigenSupport::SpMatD &hess) const;
  int getNumPlasticGlobalParams() const;
  const EigenSupport::SpMatD &getPlasticHessianTemplate() const { return d2Eda2Template; }
  void computePlasticGradient(const double *x, double *grad) const;
  void computePlasticHessian(const double *x, EigenSupport::SpMatD &hess) const;

  void computeVonMisesStresses(const double *x, double *elementStresses) const;
  void computeMaxStrains(const double *x, double *elementStrain) const;

  int getNumDOFs() const { return numDOFs; }

  const DeformationModelManager &getDeformationModelManager() const { return *deformationModelManager; }
  DeformationModelManager &getDeformationModelManager() { return *deformationModelManager; }
  const DofLayout &getDofLayout() const { return *dofLayout; }
  const EigenSupport::SpMatD &getHessianTemplate() const { return KTemplate; }
  const EigenSupport::SpMatD &get_dfda_Template() const { return dfdaTemplate; }
  const EigenSupport::SpMatD &get_dfdb_Template() const { return dfdbTemplate; }

  int getNumElasticParams() const { return numElasticParams_; }
  int getNumPlasticParams() const { return numPlasticParams_; }

protected:
  std::unique_ptr<DeformationModelManager> deformationModelManager;
  std::unique_ptr<const DofLayout> dofLayout;
  std::unique_ptr<DeformationModelAssemblerCacheData> data;

  const OptimizableField *elasticParamField_ = nullptr;
  const OptimizableField *plasticParamField_ = nullptr;

  int numDOFs, nele, neleVtx, localDOFs;
  int numElasticParams_ = 0;
  int numPlasticParams_ = 0;

  EigenSupport::SpMatD KTemplate, dfdaTemplate, dfdbTemplate, d2Eda2Template;
  std::vector<DynamicIndexMatrix> elementKInverseIndices, element_dfda_InverseIndices, element_dfdb_InverseIndices, element_d2Eda2_InverseIndices;

  std::vector<double> elementWeights;
  std::vector<const DeformationModel *> femModels;

  const int enableSanityCheck = 1;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
