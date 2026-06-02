#include "energySet.h"
#include "EigenSupport.h"

#include <cstring>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace pgo::NonlinearOptimization
{
namespace ES = pgo::EigenSupport;

class EnergySetBuffer
{
public:
  std::vector<double> energyBuffers;

  std::vector<ES::VXd> xlocals;
  std::vector<ES::VXd> vecs;

  std::vector<ES::VXd> gradients;
  std::vector<ES::VXd> hessianVectors;
  std::vector<ES::SpMatD> hessianMatrices;
};

EnergySet::EnergySet(int numDofs, std::vector<Term> terms)
  : terms_(std::move(terms)), nAll(numDofs)
{
  if (terms_.empty())
    throw std::invalid_argument("EnergySet requires at least one term");

  buffer_ = std::make_shared<EnergySetBuffer>();

  potentialEnergies.reserve(terms_.size());
  energyCoeffs.reserve(terms_.size());
  for (const auto &term : terms_) {
    potentialEnergies.push_back(term.energy);
    energyCoeffs.push_back(term.weight);
  }

  init_();
}

EnergySet::EnergySet(int numDofs, std::vector<Term> terms, std::shared_ptr<EnergySetBuffer> buffer)
  : terms_(std::move(terms)), nAll(numDofs), buffer_(std::move(buffer))
{
  if (terms_.empty())
    throw std::invalid_argument("EnergySet requires at least one term");

  potentialEnergies.reserve(terms_.size());
  energyCoeffs.reserve(terms_.size());
  for (const auto &term : terms_) {
    potentialEnergies.push_back(term.energy);
    energyCoeffs.push_back(term.weight);
  }

  init_();
}

EnergySet::~EnergySet()
{
}

void EnergySet::setWeight(int i, double w)
{
  energyCoeffs.at(i) = w;
  terms_[i].weight = w;
}

void EnergySet::init_()
{
  std::vector<ES::TripletD> entries;
  for (auto energy : potentialEnergies) {
    ES::SpMatD h;

    std::vector<int> dofs;
    energy->getDOFs(dofs);

    if (energy->isHessianTopologyFixed()) {
      energy->hessianAlloc(h);
      entries.reserve(entries.size() + static_cast<std::size_t>(h.nonZeros()));
      for (ES::IDX outeri = 0; outeri < h.outerSize(); ++outeri) {
        for (ES::SpMatD::InnerIterator it(h, outeri); it; ++it) {
          entries.emplace_back(
            static_cast<ES::SpMatD::StorageIndex>(dofs[it.row()]),
            static_cast<ES::SpMatD::StorageIndex>(dofs[it.col()]),
            1.0);
        }
      }
    }

    buffer_->hessianMatrices.push_back(h);
  }

  hessianAll.resize(nAll, nAll);
  hessianAll.setFromTriplets(entries.begin(), entries.end());

  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    std::vector<int> dofs;
    potentialEnergies[i]->getDOFs(dofs);

    ES::SpMatI mapping;
    if (potentialEnergies[i]->isHessianTopologyFixed() && buffer_->hessianMatrices[i].nonZeros()) {
      ES::small2Big(buffer_->hessianMatrices[i], hessianAll, dofs, mapping);
    }

    hessianMatrixMappings.push_back(mapping);

    buffer_->xlocals.push_back(ES::VXd::Zero(dofs.size()));
    buffer_->vecs.push_back(ES::VXd::Zero(dofs.size()));
    buffer_->gradients.push_back(ES::VXd::Zero(dofs.size()));
    buffer_->hessianVectors.push_back(ES::VXd::Zero(dofs.size()));
    energyDOFs.emplace_back(std::move(dofs));
  }

  allDOFs.resize(nAll);
  std::iota(allDOFs.begin(), allDOFs.end(), 0);

  isQuadraticEnergy = 1;
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (potentialEnergies[i]->isQuadratic() == 0) {
      isQuadraticEnergy = 0;
      break;
    }
  }

  hasHessianVectorProduct = 1;
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (potentialEnergies[i]->hasHessianVector() == 0) {
      hasHessianVectorProduct = 0;
      break;
    }
  }
}

void EnergySet::mapx(ES::ConstRefVecXd x, const std::vector<int> &dofs, ES::RefVecXd xlocal) const
{
  for (int i = 0; i < static_cast<int>(dofs.size()); i++) {
    xlocal(i) = x(dofs[i]);
  }
}

double EnergySet::func(ES::ConstRefVecXd x) const
{
  double energyAll = 0;
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (energyCoeffs[i] == 0) {
      continue;
    }

    mapx(x, energyDOFs[i], buffer_->xlocals[i]);
    double eng = potentialEnergies[i]->func(buffer_->xlocals[i]) * energyCoeffs[i];
    energyAll += eng;
  }

  return energyAll;
}

void EnergySet::gradient(ES::ConstRefVecXd x, ES::RefVecXd grad) const
{
  grad.setZero();

  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (energyCoeffs[i] == 0) {
      continue;
    }

    mapx(x, energyDOFs[i], buffer_->xlocals[i]);
    potentialEnergies[i]->gradient(buffer_->xlocals[i], buffer_->gradients[i]);

    for (Eigen::Index j = 0; j < buffer_->gradients[i].size(); j++)
      grad[energyDOFs[i][j]] += buffer_->gradients[i][j] * energyCoeffs[i];
  }
}

void EnergySet::hessianInPlace(ES::ConstRefVecXd x, ES::SpMatD &hess) const
{
  std::memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (energyCoeffs[i] == 0) {
      continue;
    }

    if (buffer_->hessianMatrices[i].nonZeros() == 0)
      continue;

    mapx(x, energyDOFs[i], buffer_->xlocals[i]);
    potentialEnergies[i]->hessianInPlace(buffer_->xlocals[i], buffer_->hessianMatrices[i]);

    ES::addSmallToBig(energyCoeffs[i], buffer_->hessianMatrices[i], hess, 1.0, hessianMatrixMappings[i]);
  }
}

void EnergySet::printEnergy(EigenSupport::ConstRefVecXd x) const
{
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    mapx(x, energyDOFs[i], buffer_->xlocals[i]);
    double energy = potentialEnergies[i]->func(buffer_->xlocals[i]);
    std::cout << "Energy " << i << ": " << energy << ',' << energy * energyCoeffs[i] << std::endl;
  }
}

void EnergySet::hessianVector(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd vec, EigenSupport::RefVecXd hessVec) const
{
  hessVec.setZero();

  if (hasHessianVectorProduct) {
    for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
      if (energyCoeffs[i] == 0) {
        continue;
      }

      mapx(x, energyDOFs[i], buffer_->xlocals[i]);
      mapx(vec, energyDOFs[i], buffer_->vecs[i]);
      potentialEnergies[i]->hessianVector(buffer_->xlocals[i], buffer_->vecs[i], buffer_->hessianVectors[i]);

      for (Eigen::Index j = 0; j < buffer_->hessianVectors[i].size(); j++)
        hessVec[energyDOFs[i][j]] += buffer_->hessianVectors[i][j] * energyCoeffs[i];
    }
  }
}

int EnergySet::isHessianTopologyFixed() const
{
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (!potentialEnergies[i]->isHessianTopologyFixed())
      return 0;
  }
  return 1;
}

void EnergySet::hessian(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const
{
  hess = hessianAll;
  std::memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (energyCoeffs[i] == 0)
      continue;

    if (buffer_->hessianMatrices[i].nonZeros() == 0)
      continue;

    if (potentialEnergies[i]->isHessianTopologyFixed()) {
      mapx(x, energyDOFs[i], buffer_->xlocals[i]);
      potentialEnergies[i]->hessianInPlace(buffer_->xlocals[i], buffer_->hessianMatrices[i]);
      ES::addSmallToBig(energyCoeffs[i], buffer_->hessianMatrices[i], hess, 1.0, hessianMatrixMappings[i]);
    }
  }

  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (energyCoeffs[i] == 0)
      continue;

    if (!potentialEnergies[i]->isHessianTopologyFixed()) {
      mapx(x, energyDOFs[i], buffer_->xlocals[i]);
      ES::SpMatD Ki;
      potentialEnergies[i]->hessian(buffer_->xlocals[i], Ki);
      if (Ki.nonZeros() == 0)
        continue;

      ES::SpMatD KiGlobal(nAll, nAll);
      std::vector<ES::TripletD> entries;
      entries.reserve(Ki.nonZeros());
      for (Eigen::Index outeri = 0; outeri < Ki.outerSize(); outeri++) {
        for (ES::SpMatD::InnerIterator it(Ki, outeri); it; ++it) {
          entries.emplace_back(
            static_cast<ES::SpMatD::StorageIndex>(energyDOFs[i][it.row()]),
            static_cast<ES::SpMatD::StorageIndex>(energyDOFs[i][it.col()]),
            it.value() * energyCoeffs[i]);
        }
      }
      KiGlobal.setFromTriplets(entries.begin(), entries.end());
      hess = hess + KiGlobal;
    }
  }
}

void EnergySet::gradient_hessian(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad, EigenSupport::SpMatD &hess) const
{
  grad.setZero();

  hess = hessianAll;
  std::memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    if (energyCoeffs[i] == 0)
      continue;

    mapx(x, energyDOFs[i], buffer_->xlocals[i]);
    buffer_->gradients[i].setZero();

    if (potentialEnergies[i]->isHessianTopologyFixed()) {
      potentialEnergies[i]->gradient(buffer_->xlocals[i], buffer_->gradients[i]);

      if (buffer_->hessianMatrices[i].nonZeros()) {
        potentialEnergies[i]->hessianInPlace(buffer_->xlocals[i], buffer_->hessianMatrices[i]);
        ES::addSmallToBig(energyCoeffs[i], buffer_->hessianMatrices[i], hess, 1.0, hessianMatrixMappings[i]);
      }
    }
    else {
      ES::SpMatD Ki;
      potentialEnergies[i]->gradient_hessian(buffer_->xlocals[i], buffer_->gradients[i], Ki);
      if (Ki.nonZeros()) {
        ES::SpMatD KiGlobal(nAll, nAll);
        std::vector<ES::TripletD> entries;
        entries.reserve(Ki.nonZeros());
        for (Eigen::Index outeri = 0; outeri < Ki.outerSize(); outeri++) {
          for (ES::SpMatD::InnerIterator it(Ki, outeri); it; ++it) {
            entries.emplace_back(
              static_cast<ES::SpMatD::StorageIndex>(energyDOFs[i][it.row()]),
              static_cast<ES::SpMatD::StorageIndex>(energyDOFs[i][it.col()]),
              it.value() * energyCoeffs[i]);
          }
        }
        KiGlobal.setFromTriplets(entries.begin(), entries.end());
        hess = hess + KiGlobal;
      }
    }

    for (Eigen::Index j = 0; j < buffer_->gradients[i].size(); j++)
      grad[energyDOFs[i][j]] += buffer_->gradients[i][j] * energyCoeffs[i];
  }
}

double EnergySet::func_grad_hessian(
  EigenSupport::ConstRefVecXd x,
  EigenSupport::RefVecXd grad,
  EigenSupport::SpMatD &hess) const
{
  gradient_hessian(x, grad, hess);
  return func(x);
}

StepConstraint EnergySet::computeMaxStepLimit(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx, StepConstraintSink *sink) const
{
  // Across energies of possibly different sources, keep the binding (min-alpha) one.
  StepConstraint binding = {};
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    mapx(x, energyDOFs[i], buffer_->xlocals[i]);
    mapx(dx, energyDOFs[i], buffer_->vecs[i]);
    const StepConstraint c = potentialEnergies[i]->computeMaxStepLimit(buffer_->xlocals[i], buffer_->vecs[i], sink);
    if (c.alpha < binding.alpha)
      binding = c;
  }
  return binding;
}

void EnergySet::beginLineSearch(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd dx) const
{
  for (std::size_t i = 0; i < potentialEnergies.size(); i++) {
    const auto *aware = dynamic_cast<const LineSearchAwareEnergy *>(potentialEnergies[i].get());
    if (!aware)
      continue;
    mapx(x, energyDOFs[i], buffer_->xlocals[i]);
    mapx(dx, energyDOFs[i], buffer_->vecs[i]);
    aware->beginLineSearch(buffer_->xlocals[i], buffer_->vecs[i]);
  }
}

void EnergySet::endLineSearch() const
{
  for (const auto &energy : potentialEnergies) {
    if (const auto *aware = dynamic_cast<const LineSearchAwareEnergy *>(energy.get()))
      aware->endLineSearch();
  }
}

EnergyStateKind EnergySet::stateKind() const
{
  EnergyStateKind kind = terms_[0].energy->stateKind();
  for (std::size_t i = 1; i < terms_.size(); i++) {
    if (terms_[i].energy->stateKind() != kind)
      return EnergyStateKind::Generic;
  }
  return kind;
}
}  // namespace pgo::NonlinearOptimization
