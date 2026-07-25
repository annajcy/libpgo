/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "energy/plasticMaterialEnergy.h"

#include "deformation/deformationModelAssembler.h"

#include <numeric>
#include <span>
#include <stdexcept>

using namespace pgo;
using namespace pgo::NonlinearOptimization;
using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

PlasticMaterialEnergy::PlasticMaterialEnergy(
  std::shared_ptr<const DeformationModelEnergy> deformationEnergy,
  EigenSupport::ConstRefVecXd fixedDisplacement):
  deformationEnergy_(std::move(deformationEnergy)),
  fixedDisplacement_(fixedDisplacement)
{
  if (!deformationEnergy_) {
    throw std::invalid_argument("PlasticMaterialEnergy requires a non-null deformation energy.");
  }
  if (fixedDisplacement_.size() != deformationEnergy_->getRestDofs().size()) {
    throw std::invalid_argument("PlasticMaterialEnergy fixed displacement size must match the deformation energy rest position size.");
  }

  fixedState_ = deformationEnergy_->materialParameters()->snapshot();

  allDOFs_.resize(deformationEnergy_->assembler().getNumPlasticGlobalParams());
  std::iota(allDOFs_.begin(), allDOFs_.end(), 0);
}

PlasticMaterialEnergy::~PlasticMaterialEnergy() = default;

ES::VXd PlasticMaterialEnergy::absolutePositions() const
{
  return deformationEnergy_->getRestDofs() + fixedDisplacement_;
}

double PlasticMaterialEnergy::func(EigenSupport::ConstRefVecXd x) const
{
  const ES::VXd p = absolutePositions();
  const MaterialParameterEvaluationView state = fixedState_.withPlasticValues(
    std::span<const double>(x.data(), static_cast<std::size_t>(x.size())));
  return deformationEnergy_->assembler().computeEnergy(p.data(), state);
}

void PlasticMaterialEnergy::gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const
{
  const ES::VXd p = absolutePositions();
  const MaterialParameterEvaluationView state = fixedState_.withPlasticValues(
    std::span<const double>(x.data(), static_cast<std::size_t>(x.size())));
  deformationEnergy_->assembler().compute_dE_dp(
    p.data(), state, grad.data());
}

void PlasticMaterialEnergy::hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const
{
  const ES::VXd p = absolutePositions();
  const MaterialParameterEvaluationView state = fixedState_.withPlasticValues(
    std::span<const double>(x.data(), static_cast<std::size_t>(x.size())));
  deformationEnergy_->assembler().compute_d2E_dp2(
    p.data(), state, hess);
}

void PlasticMaterialEnergy::hessianAlloc(EigenSupport::SpMatD &hess) const
{
  hess = deformationEnergy_->assembler().d2E_dp2_template();
}
