/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "energy/deformationModelEnergy.h"
#include "energy/potentialEnergy.h"

#include <memory>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

class PlasticMaterialEnergy : public NonlinearOptimization::PotentialEnergy
{
public:
  PlasticMaterialEnergy(
    std::shared_ptr<const DeformationModelEnergy> deformationEnergy,
    EigenSupport::ConstRefVecXd fixedDisplacement);
  virtual ~PlasticMaterialEnergy();

  virtual double func(EigenSupport::ConstRefVecXd x) const override;
  virtual void gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const override;
  virtual void hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const override;
  virtual void hessianAlloc(EigenSupport::SpMatD &hess) const override;
  virtual void getDOFs(std::vector<int> &dofs) const override { dofs = allDOFs_; }
  virtual int getNumDOFs() const override { return static_cast<int>(allDOFs_.size()); }

  virtual NonlinearOptimization::EnergyStateKind stateKind() const override { return NonlinearOptimization::EnergyStateKind::Generic; }

  const EigenSupport::VXd &fixedDisplacement() const { return fixedDisplacement_; }
  std::shared_ptr<const DeformationModelEnergy> deformationEnergy() const { return deformationEnergy_; }

private:
  EigenSupport::VXd absolutePositions() const;

  std::shared_ptr<const DeformationModelEnergy> deformationEnergy_;
  EigenSupport::VXd fixedDisplacement_;
  MaterialParameterSnapshot fixedState_;
  std::vector<int> allDOFs_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
