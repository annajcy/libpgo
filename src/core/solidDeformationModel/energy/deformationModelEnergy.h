/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "energy/potentialEnergy.h"

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
  DeformationModelEnergy(std::unique_ptr<DeformationModelAssembler> fma, int offset = 0, bool enableMaterialMaxStep = true);
  virtual ~DeformationModelEnergy();

  const DeformationModelAssembler &assembler() const { return *forceModelAssembler; }
  DeformationModelAssembler &assembler() { return *forceModelAssembler; }

  const EigenSupport::VXd &getRestPosition() const { return *restPosition; }

  virtual double func(EigenSupport::ConstRefVecXd x) const override;
  virtual void gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const override;
  virtual void hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const override;
  virtual void hessianAlloc(EigenSupport::SpMatD &hess) const override;
  virtual void getDOFs(std::vector<int> &dofs) const override { dofs = this->allDOFs; }
  virtual int getNumDOFs() const override { return (int)allDOFs.size(); }

  virtual NonlinearOptimization::EnergyStateKind stateKind() const override { return NonlinearOptimization::EnergyStateKind::Displacement; }

  virtual NonlinearOptimization::StepConstraint computeMaxStepLimit(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx, StepConstraintSink *sink = nullptr) const override;

  void setEnableMaterialMaxStep(bool enable) { enableMaterialMaxStep_ = enable; }
  bool isMaterialMaxStepEnabled() const { return enableMaterialMaxStep_; }

protected:
  std::unique_ptr<DeformationModelAssembler> forceModelAssembler;
  std::vector<int> allDOFs;
  std::unique_ptr<EigenSupport::VXd> restPosition;
  mutable tbb::enumerable_thread_specific<EigenSupport::VXd> absolutePositionScratch_;
  mutable tbb::enumerable_thread_specific<EigenSupport::VXd> directionScratch_;
  bool enableMaterialMaxStep_ = true;

private:
  EigenSupport::VXd &absolutePositionScratch() const;
  EigenSupport::VXd &directionScratch() const;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
