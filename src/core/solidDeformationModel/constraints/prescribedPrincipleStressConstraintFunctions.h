/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "constraints/constraintFunctions.h"
#include "deformation/volume/volumetricDeformationElement.h"
#include "material/runtime/materialState.h"

#include <tbb/spin_mutex.h>

#include <memory>
#include <span>
#include <vector>
#include <functional>

namespace pgo
{
namespace SolidDeformationModel
{
class DeformationModelAssembler;

class PrescribedPrincipleStressConstraintFunctions : public NonlinearOptimization::ConstraintFunctions
{
public:
  PrescribedPrincipleStressConstraintFunctions(
    int nAll, int dofOffset, std::span<const int> elementIDs,
    const DeformationModelAssembler &assembler,
    MaterialState materialState);
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
  const DeformationModelAssembler &assembler_;
  MaterialState materialState_;
  std::vector<int> elements;
  XToPosFunc xToPosFunc;
  EigenSupport::VXd targetPrincipleStress;

  struct ElementData
  {
    const int numDOFs;
    const int numMaterialLocations;
    EigenSupport::V18d localp;
    const VolumetricDeformationElement *deformation = nullptr;

    explicit ElementData(const VolumetricDeformationElement &model):
      numDOFs(model.getNumDOFs()),
      numMaterialLocations(model.getNumMaterialLocations()),
      deformation(&model) {}
  };

  mutable std::vector<ElementData> elementData_;

  std::span<const double> elasticValues(
    int elementID, MaterialStateView state) const;
  std::span<const double> plasticValues(
    int elementID, MaterialStateView state) const;

  EigenSupport::EntryMap jacEntries, hessEntries;

  mutable std::vector<tbb::spin_mutex> hessLocks;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
