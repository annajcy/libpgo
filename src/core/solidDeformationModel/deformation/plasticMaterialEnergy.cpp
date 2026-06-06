/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "deformation/plasticMaterialEnergy.h"

#include "deformation/deformationModelAssembler.h"

#include <numeric>
#include <stdexcept>

using namespace pgo;
using namespace pgo::NonlinearOptimization;
using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

PlasticMaterialEnergy::PlasticMaterialEnergy(
  std::shared_ptr<DeformationModelState> state,
  std::shared_ptr<DeformationModelEnergy> deformationEnergy,
  EigenSupport::ConstRefVecXd fixedDisplacement):
  state_(std::move(state)),
  deformationEnergy_(std::move(deformationEnergy)),
  fixedDisplacement_(fixedDisplacement)
{
  if (!state_) {
    throw std::invalid_argument("PlasticMaterialEnergy requires a non-null state.");
  }
  if (!deformationEnergy_) {
    throw std::invalid_argument("PlasticMaterialEnergy requires a non-null deformation energy.");
  }
  if (fixedDisplacement_.size() != deformationEnergy_->getRestPosition().size()) {
    throw std::invalid_argument("PlasticMaterialEnergy fixed displacement size must match the deformation energy rest position size.");
  }

  allDOFs_.resize(deformationEnergy_->assembler().getNumPlasticGlobalParams());
  std::iota(allDOFs_.begin(), allDOFs_.end(), 0);
}

PlasticMaterialEnergy::~PlasticMaterialEnergy() = default;

void PlasticMaterialEnergy::setPlasticState(EigenSupport::ConstRefVecXd x) const
{
  if (x.size() != getNumDOFs()) {
    throw std::invalid_argument("PlasticMaterialEnergy state size does not match the number of plastic DOFs.");
  }
  state_->setPlasticValues(x);
}

ES::VXd PlasticMaterialEnergy::absolutePositions() const
{
  return deformationEnergy_->getRestPosition() + fixedDisplacement_;
}

double PlasticMaterialEnergy::func(EigenSupport::ConstRefVecXd x) const
{
  setPlasticState(x);
  const ES::VXd p = absolutePositions();
  return deformationEnergy_->assembler().computeEnergy(p.data());
}

void PlasticMaterialEnergy::gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const
{
  setPlasticState(x);
  const ES::VXd p = absolutePositions();
  deformationEnergy_->assembler().computePlasticGradient(p.data(), grad.data());
}

void PlasticMaterialEnergy::hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const
{
  setPlasticState(x);
  const ES::VXd p = absolutePositions();
  deformationEnergy_->assembler().computePlasticHessian(p.data(), hess);
}

void PlasticMaterialEnergy::hessianAlloc(EigenSupport::SpMatD &hess) const
{
  hess = deformationEnergy_->assembler().getPlasticHessianTemplate();
}
