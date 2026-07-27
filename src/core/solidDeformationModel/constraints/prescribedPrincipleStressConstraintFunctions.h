/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "constraints/constraintFunctions.h"
#include "deformation/volume/volumetricDeformationModel.h"
#include "deformation/volume/volumetricDeformationModelEvaluator.h"
#include "material/core/optimizableParameters.h"

#include <tbb/spin_mutex.h>

#include <memory>
#include <span>
#include <vector>
#include <functional>

namespace pgo
{
namespace SolidDeformationModel
{
class DeformationModelManager;

class PrescribedPrincipleStressConstraintFunctions : public NonlinearOptimization::ConstraintFunctions
{
public:
  PrescribedPrincipleStressConstraintFunctions(
    int nAll, int dofOffset, std::span<const int> elementIDs,
    const DeformationModelManager &tetMeshDMM,
    std::shared_ptr<const OptimizableParameters> optimizableParameters);
  virtual ~PrescribedPrincipleStressConstraintFunctions() {}

  using XToPosFunc = std::function<void(const EigenSupport::V3d &, int offset, EigenSupport::V3d &)>;
  void setXToPosFunc(XToPosFunc func) { xToPosFunc = func; }
  void setTargetPHat(std::span<const double> phat);

  void computeForceFromTargetPHat(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd fext) const;
  double computeSurfaceNormalTractionFromElement(EigenSupport::ConstRefVecXd x, const EigenSupport::V3d &n, int eleID) const;

  virtual void func(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd g) const override;
  virtual void jacobian(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &jac) const override;
  virtual void hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd lambda, EigenSupport::SpMatD &hess) const override;

  virtual bool isLinear() const override { return false; }
  virtual bool isQuadratic() const override { return false; }
  virtual bool hasHessianVector() const override { return false; }

protected:
  int dofStart;
  const DeformationModelManager &tetMeshDMM;
  std::shared_ptr<const OptimizableParameters> optimizableParameters_;
  std::vector<int> elements;
  XToPosFunc xToPosFunc;
  EigenSupport::VXd targetPrincipleStress;

  struct ElementData
  {
    const int numDOFs;
    const int numMaterialLocations;
    EigenSupport::V18d localp;
    OptimizableParameterEvaluationScratch elasticParameters;
    OptimizableParameterEvaluationScratch plasticParameters;
    std::unique_ptr<VolumetricDeformationModelEvaluator> evaluator;

    explicit ElementData(const VolumetricDeformationModel &model):
      numDOFs(model.getNumDOFs()),
      numMaterialLocations(model.getNumMaterialLocations()),
      evaluator(
        std::make_unique<VolumetricDeformationModelEvaluator>(model)) {}
  };

  mutable std::vector<ElementData> elementData_;

  VolumetricDeformationModelEvaluator &prepareElement(
    int elementID, OptimizableParameterEvaluationView state,
    ElementData &data) const;

  EigenSupport::EntryMap jacEntries, hessEntries;

  mutable std::vector<tbb::spin_mutex> hessLocks;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
