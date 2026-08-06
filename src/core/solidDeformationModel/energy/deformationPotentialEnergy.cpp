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
  if (!energyOperator_)
    throw std::invalid_argument(
      "DeformationPotentialEnergy requires an operator and material state.");
  if (materialState_.elasticValues().size() !=
      energyOperator_->assembler().getNumElasticGlobalParams() ||
    materialState_.plasticValues().size() !=
      energyOperator_->assembler().getNumPlasticGlobalParams())
    throw std::invalid_argument(
      "DeformationPotentialEnergy material state size does not match the operator.");
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

double DeformationPotentialEnergy::funcGradient(
  EigenSupport::ConstRefVecXd x,
  EigenSupport::RefVecXd grad) const
{
  return energyOperator_->funcGradient(x, materialState_.view(), grad);
}

double DeformationPotentialEnergy::funcGradientHessian(
  EigenSupport::ConstRefVecXd x,
  EigenSupport::RefVecXd grad,
  EigenSupport::SpMatD &hess) const
{
  return energyOperator_->funcGradientHessian(
    x, materialState_.view(), grad, hess);
}

void DeformationPotentialEnergy::gradientHessian(
  EigenSupport::ConstRefVecXd x,
  EigenSupport::RefVecXd grad,
  EigenSupport::SpMatD &hess) const
{
  energyOperator_->gradientHessian(x, materialState_.view(), grad, hess);
}

void DeformationPotentialEnergy::getDOFs(std::vector<int> &dofs) const
{
  energyOperator_->getDOFs(dofs);
}

int DeformationPotentialEnergy::getNumDOFs() const
{
  return energyOperator_->getNumDOFs();
}

}  // namespace pgo::SolidDeformationModel
