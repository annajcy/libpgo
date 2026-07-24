/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "energy/elasticMaterialEnergy.h"

#include "deformation/deformationModelAssembler.h"

#include <numeric>
#include <span>
#include <stdexcept>

using namespace pgo;
using namespace pgo::NonlinearOptimization;
using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

ElasticMaterialEnergy::ElasticMaterialEnergy(
  std::shared_ptr<const DeformationModelEnergy> deformationEnergy,
  EigenSupport::ConstRefVecXd fixedDisplacement):
  deformationEnergy_(std::move(deformationEnergy)),
  fixedDisplacement_(fixedDisplacement)
{
  if (!deformationEnergy_) {
    throw std::invalid_argument("ElasticMaterialEnergy requires a non-null deformation energy.");
  }
  if (fixedDisplacement_.size() != deformationEnergy_->getRestDofs().size()) {
    throw std::invalid_argument("ElasticMaterialEnergy fixed displacement size must match the deformation energy rest position size.");
  }

  fixedPlasticParameters_ =
    deformationEnergy_->materialParameters()->plasticSnapshot();

  allDOFs_.resize(deformationEnergy_->assembler().getNumElasticGlobalParams());
  std::iota(allDOFs_.begin(), allDOFs_.end(), 0);
}

ElasticMaterialEnergy::~ElasticMaterialEnergy() = default;

ES::VXd ElasticMaterialEnergy::absolutePositions() const
{
  return deformationEnergy_->getRestDofs() + fixedDisplacement_;
}

double ElasticMaterialEnergy::func(EigenSupport::ConstRefVecXd x) const
{
  const ES::VXd p = absolutePositions();
  const MaterialStateView state =
    deformationEnergy_->materialParameters()->space()->makeStateView(
    std::span<const double>(x.data(), static_cast<std::size_t>(x.size())),
    std::span<const double>(fixedPlasticParameters_.data(), static_cast<std::size_t>(fixedPlasticParameters_.size())));
  return deformationEnergy_->assembler().computeEnergy(p.data(), state);
}

void ElasticMaterialEnergy::gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const
{
  const ES::VXd p = absolutePositions();
  const MaterialStateView state =
    deformationEnergy_->materialParameters()->space()->makeStateView(
    std::span<const double>(x.data(), static_cast<std::size_t>(x.size())),
    std::span<const double>(fixedPlasticParameters_.data(), static_cast<std::size_t>(fixedPlasticParameters_.size())));
  deformationEnergy_->assembler().compute_dE_de(
    p.data(), state, grad.data());
}

void ElasticMaterialEnergy::hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const
{
  const ES::VXd p = absolutePositions();
  const MaterialStateView state =
    deformationEnergy_->materialParameters()->space()->makeStateView(
    std::span<const double>(x.data(), static_cast<std::size_t>(x.size())),
    std::span<const double>(fixedPlasticParameters_.data(), static_cast<std::size_t>(fixedPlasticParameters_.size())));
  deformationEnergy_->assembler().compute_d2E_de2(
    p.data(), state, hess);
}

void ElasticMaterialEnergy::hessianAlloc(EigenSupport::SpMatD &hess) const
{
  hess = deformationEnergy_->assembler().d2E_de2_template();
}
