#include "energy/deformationPotentialEnergy.h"

#include "deformation/deformationModelAssembler.h"

#include <stdexcept>

namespace pgo::SolidDeformationModel
{

DeformationPotentialEnergy::DeformationPotentialEnergy(
  std::shared_ptr<DeformationEnergyOperator> energyOperator,
  MaterialState materialState):
  energyOperator_(std::move(energyOperator)),
  materialState_(std::move(materialState))
{
  if (!energyOperator_ || materialState_.empty())
    throw std::invalid_argument(
      "DeformationPotentialEnergy requires an operator and material state.");
  if (!materialState_.elasticField().sharesStateWith(
      *energyOperator_->assembler().elasticField()) ||
    !materialState_.plasticField().sharesStateWith(
      *energyOperator_->assembler().plasticField()))
    throw std::invalid_argument(
      "DeformationPotentialEnergy material state belongs to different fields.");
}

double DeformationPotentialEnergy::func(EigenSupport::ConstRefVecXd x) const
{
  return energyOperator_->func(x, materialState_.view());
}

void DeformationPotentialEnergy::gradient(
  EigenSupport::ConstRefVecXd x,
  EigenSupport::RefVecXd grad) const
{
  energyOperator_->gradient(x, materialState_.view(), grad);
}

void DeformationPotentialEnergy::hessianInPlace(
  EigenSupport::ConstRefVecXd x,
  EigenSupport::SpMatD &hess) const
{
  energyOperator_->hessianInPlace(x, materialState_.view(), hess);
}

void DeformationPotentialEnergy::hessianAlloc(EigenSupport::SpMatD &hess) const
{
  energyOperator_->hessianAlloc(hess);
}

void DeformationPotentialEnergy::getDOFs(std::vector<int> &dofs) const
{
  energyOperator_->getDOFs(dofs);
}

int DeformationPotentialEnergy::getNumDOFs() const
{
  return energyOperator_->getNumDOFs();
}

NonlinearOptimization::StepConstraint
DeformationPotentialEnergy::computeMaxStepLimit(
  EigenSupport::ConstRefVecXd x,
  EigenSupport::ConstRefVecXd dx,
  NonlinearOptimization::StepConstraintSink *sink) const
{
  return energyOperator_->computeMaxStepLimit(x, dx, sink);
}

}  // namespace pgo::SolidDeformationModel
