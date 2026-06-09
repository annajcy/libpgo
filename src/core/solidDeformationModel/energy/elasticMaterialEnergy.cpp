/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "energy/elasticMaterialEnergy.h"

#include "deformation/deformationModelAssembler.h"

#include <numeric>
#include <stdexcept>

using namespace pgo;
using namespace pgo::NonlinearOptimization;
using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

ElasticMaterialEnergy::ElasticMaterialEnergy(
  std::shared_ptr<DeformationModelEnergy> deformationEnergy,
  EigenSupport::ConstRefVecXd fixedDisplacement):
  deformationEnergy_(std::move(deformationEnergy)),
  fixedDisplacement_(fixedDisplacement)
{
  if (!deformationEnergy_) {
    throw std::invalid_argument("ElasticMaterialEnergy requires a non-null deformation energy.");
  }
  if (fixedDisplacement_.size() != deformationEnergy_->getRestPosition().size()) {
    throw std::invalid_argument("ElasticMaterialEnergy fixed displacement size must match the deformation energy rest position size.");
  }

  allDOFs_.resize(deformationEnergy_->assembler().getNumElasticGlobalParams());
  std::iota(allDOFs_.begin(), allDOFs_.end(), 0);
}

ElasticMaterialEnergy::~ElasticMaterialEnergy() = default;

void ElasticMaterialEnergy::setElasticState(EigenSupport::ConstRefVecXd x) const
{
  if (x.size() != getNumDOFs()) {
    throw std::invalid_argument("ElasticMaterialEnergy state size does not match the number of elastic DOFs.");
  }
  deformationEnergy_->assembler().setElasticValues(x);
}

ES::VXd ElasticMaterialEnergy::absolutePositions() const
{
  return deformationEnergy_->getRestPosition() + fixedDisplacement_;
}

double ElasticMaterialEnergy::func(EigenSupport::ConstRefVecXd x) const
{
  setElasticState(x);
  const ES::VXd p = absolutePositions();
  return deformationEnergy_->assembler().computeEnergy(p.data());
}

void ElasticMaterialEnergy::gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const
{
  setElasticState(x);
  const ES::VXd p = absolutePositions();
  deformationEnergy_->assembler().computeElasticGradient(p.data(), grad.data());
}

void ElasticMaterialEnergy::hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const
{
  setElasticState(x);
  const ES::VXd p = absolutePositions();
  deformationEnergy_->assembler().computeElasticHessian(p.data(), hess);
}

void ElasticMaterialEnergy::hessianAlloc(EigenSupport::SpMatD &hess) const
{
  hess = deformationEnergy_->assembler().getElasticHessianTemplate();
}
