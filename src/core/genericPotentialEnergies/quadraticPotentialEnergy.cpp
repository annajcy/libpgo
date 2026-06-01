/*
author: Bohan Wang
copyright to USC, MIT
*/

#include "quadraticPotentialEnergy.h"
#include "pgoLogging.h"
#include "EigenSupport.h"

#include <tbb/enumerable_thread_specific.h>

#include <numeric>
#include <vector>

using namespace pgo;
using namespace pgo::PredefinedPotentialEnergies;

namespace ES = pgo::EigenSupport;

namespace pgo::PredefinedPotentialEnergies
{
class QuadraticPotentialEnergyCache
{
public:
  QuadraticPotentialEnergyCache(int n)
  {
    temp.setZero(n);
  }

  ES::VXd temp;
};

}  // namespace pgo::PredefinedPotentialEnergies

// ── Direct quadratic-form ctors ─────────────────────────────────

QuadraticPotentialEnergy::QuadraticPotentialEnergy(ES::SpMatD A):
  A_(std::move(A)), b_(std::nullopt)
{
  cache = std::make_shared<QuadraticPotentialEnergyCache>(A_.rows());

  allDOFs.resize(A_.rows());
  std::iota(allDOFs.begin(), allDOFs.end(), 0);
}

QuadraticPotentialEnergy::QuadraticPotentialEnergy(ES::SpMatD A, ES::VXd b):
  A_(std::move(A)), b_(std::move(b))
{
  cache = std::make_shared<QuadraticPotentialEnergyCache>(A_.rows());

  allDOFs.resize(A_.rows());
  std::iota(allDOFs.begin(), allDOFs.end(), 0);
}

// ── Least-squares factories ─────────────────────────────────────

namespace pgo::PredefinedPotentialEnergies
{

std::shared_ptr<QuadraticPotentialEnergy>
  makeLeastSquaresEnergy(ES::SpMatD A)
{
  ES::SpMatD ATA;
  ES::mm(A, A, ATA, 1);
  return std::make_shared<QuadraticPotentialEnergy>(std::move(ATA));
}

std::shared_ptr<QuadraticPotentialEnergy>
  makeLeastSquaresEnergy(ES::SpMatD A, ES::VXd b)
{
  ES::SpMatD ATA;
  ES::mm(A, A, ATA, 1);

  ES::VXd ATb(ATA.rows());
  ES::mv(A, b, ATb, 1);

  auto energy = std::make_shared<QuadraticPotentialEnergy>(std::move(ATA), std::move(ATb));
  energy->c = b.dot(b) * 0.5;
  return energy;
}

std::shared_ptr<QuadraticPotentialEnergy>
  makeLeastSquaresEnergy(ES::SpMatD A, const double *W)
{
  ES::SpMatD ATWA;
  ES::aba(A, W, ATWA, 1);
  return std::make_shared<QuadraticPotentialEnergy>(std::move(ATWA));
}

std::shared_ptr<QuadraticPotentialEnergy>
  makeLeastSquaresEnergy(ES::SpMatD A, ES::VXd b, const double *W)
{
  ES::SpMatD ATWA;
  ES::aba(A, W, ATWA, 1);

  ES::VXd Wb = b;
  for (ES::IDX i = 0; i < A.rows(); i++)
    Wb[i] = b[i] * W[i];

  ES::VXd ATWb(ATWA.rows());
  ES::mv(A, Wb, ATWb, 1);

  auto energy = std::make_shared<QuadraticPotentialEnergy>(std::move(ATWA), std::move(ATWb));
  energy->c = b.dot(Wb) * 0.5;
  return energy;
}

}  // namespace pgo::PredefinedPotentialEnergies

// ── Methods ─────────────────────────────────────────────────────

void QuadraticPotentialEnergy::setDOFs(const std::vector<int> &dofs)
{
  PGO_ALOG((int)dofs.size() == (int)A_.rows());
  allDOFs = dofs;
}

double QuadraticPotentialEnergy::func(ES::ConstRefVecXd x) const
{
  ES::VXd &temp = cache->temp;
  double energy = ES::vTMv(A_, x, temp) * 0.5;

  energy += c;
  if (b_) {
    energy += (*b_).dot(x);
  }
  return energy;
}

void QuadraticPotentialEnergy::gradient(ES::ConstRefVecXd x, ES::RefVecXd grad) const
{
  ES::mv(A_, x, grad);

  if (b_) {
    grad += (*b_);
  }
}

void QuadraticPotentialEnergy::hessianInPlace(ES::ConstRefVecXd, ES::SpMatD &hess) const
{
  memcpy(hess.valuePtr(), A_.valuePtr(), sizeof(double) * A_.nonZeros());
}

void QuadraticPotentialEnergy::hessianVector(EigenSupport::ConstRefVecXd, EigenSupport::ConstRefVecXd vec, EigenSupport::RefVecXd hVec) const
{
  ES::mv(A_, vec, hVec);
}

void QuadraticPotentialEnergy::gradientComponent(ES::SpMatD *A, ES::VXd *b) const
{
  if (A)
    *A = A_;

  if (b) {
    if (b_) {
      *b = *b_;
    }
    else {
      b->setZero(getNumDOFs());
    }
  }
}
