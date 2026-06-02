/*
author: Bohan Wang
copyright to USC
*/

#include "potentialEnergyFromConstraintFunctions.h"
#include "EigenSupport.h"

#include <cstring>
#include <stdexcept>
#include <numeric>
#include <iostream>

using namespace pgo;
using namespace pgo::NonlinearOptimization;

PotentialEnergyConstraintFunctions::PotentialEnergyConstraintFunctions(int nAll, std::shared_ptr<const ConstraintFunctions> c):
  cnstt(c)
{
  g.resize(cnstt->getNumConstraints());
  cnstt->createJacobian(jac);
  cnstt->hessianAlloc(lambdaHessian);

  allDOFs.resize(nAll);
  std::iota(allDOFs.begin(), allDOFs.end(), 0);

  if (lambdaHessian.nonZeros()) {
    ES::symbolicMm(jac, jac, JTJ, &mmData, 1);
    ES::mergeSparseMatrix(hessAll, JTJ, lambdaHessian);
    ES::small2Big(JTJ, hessAll, 0, 0, JTJMapping);
    ES::small2Big(lambdaHessian, hessAll, 0, 0, lambdaHessMapping);
  }
  else {
    ES::VXd zero(nAll);
    zero.setZero();

    cnstt->jacobian(zero, jac);
    ES::mm(jac, jac, JTJ, 1);
    hessAll = JTJ;
  }
}

PotentialEnergyConstraintFunctions::~PotentialEnergyConstraintFunctions()
{
  if (lambdaHessian.nonZeros()) {
    ES::destroySymbolicMmData(mmData);
  }
}

double PotentialEnergyConstraintFunctions::func(ES::ConstRefVecXd x) const
{
  g.setZero();
  cnstt->func(x, g);
  return g.dot(g) * 0.5;
}

void PotentialEnergyConstraintFunctions::gradient(ES::ConstRefVecXd x, ES::RefVecXd grad) const
{
  cnstt->jacobian(x, jac);
  cnstt->func(x, g);
  ES::mv(jac, g, grad, 1);
}

void PotentialEnergyConstraintFunctions::hessianInPlace(ES::ConstRefVecXd x, ES::SpMatD &hess) const
{
  memset(hess.valuePtr(), 0, hess.nonZeros() * sizeof(double));

  if (lambdaHessian.nonZeros()) {
    // g^2
    // dE = gT dgdx
    // d2E = dgdx^T dgdx + gT d2gdx
    cnstt->jacobian(x, jac);
    ES::mm(jac, jac, mmData, JTJ, 1);
    ES::addSmallToBig(1.0, JTJ, hess, 1.0, JTJMapping, 1);

    // std::cout << "Jac:\n";
    // std::cout << ES::MXd(jac) << std::endl;

    cnstt->func(x, g);
    cnstt->hessianInPlace(x, g, lambdaHessian);

    // std::cout << "g:\n"
    //           << g << std::endl;

    // std::cout << "x:\n"
    //           << x << std::endl;

    // std::cout << "H:\n";
    // std::cout << ES::MXd(lambdaHessian) << std::endl;

    ES::addSmallToBig(1.0, lambdaHessian, hess, 1.0, lambdaHessMapping, 1);
  }
  else {
    memcpy(hess.valuePtr(), JTJ.valuePtr(), sizeof(double) * JTJ.nonZeros());
  }
}

PotentialEnergyBoundedConstraintFunctions::PotentialEnergyBoundedConstraintFunctions(
  int nAll,
  std::shared_ptr<const ConstraintFunctions> c,
  ES::VXd lower_,
  ES::VXd upper_):
  cnstt(std::move(c)), lower(std::move(lower_)), upper(std::move(upper_))
{
  if (!cnstt) {
    throw std::invalid_argument("PotentialEnergyBoundedConstraintFunctions requires non-null constraints");
  }
  if (lower.size() != cnstt->getNumConstraints() || upper.size() != cnstt->getNumConstraints()) {
    throw std::invalid_argument("Bound vectors must match constraint count");
  }
  for (int i = 0; i < lower.size(); i++) {
    if (lower[i] > upper[i]) {
      throw std::invalid_argument("Constraint lower bound must be <= upper bound");
    }
  }

  values.resize(cnstt->getNumConstraints());
  residual.resize(cnstt->getNumConstraints());
  activeMask.resize(cnstt->getNumConstraints());
  cnstt->createJacobian(jac);
  cnstt->hessianAlloc(lambdaHessian);

  allDOFs.resize(nAll);
  std::iota(allDOFs.begin(), allDOFs.end(), 0);

  ES::VXd zero(nAll);
  zero.setZero();
  cnstt->jacobian(zero, jac);
  ES::mm(jac, jac, weightedJTJ, 1);

  if (lambdaHessian.nonZeros()) {
    ES::symbolicMm(jac, jac, weightedJTJ, &mmData, 1);
    ES::mergeSparseMatrix(hessAll, weightedJTJ, lambdaHessian);
    ES::small2Big(weightedJTJ, hessAll, 0, 0, weightedJTJMapping);
    ES::small2Big(lambdaHessian, hessAll, 0, 0, lambdaHessMapping);
  }
  else {
    hessAll = weightedJTJ;
  }
}

PotentialEnergyBoundedConstraintFunctions::~PotentialEnergyBoundedConstraintFunctions()
{
  if (mmData) {
    ES::destroySymbolicMmData(mmData);
  }
}

void PotentialEnergyBoundedConstraintFunctions::computeViolation(ES::ConstRefVecXd x) const
{
  cnstt->func(x, values);
  residual.setZero();
  activeMask.setZero();

  for (int i = 0; i < values.size(); i++) {
    if (values[i] < lower[i]) {
      residual[i] = values[i] - lower[i];
      activeMask[i] = 1.0;
    }
    else if (values[i] > upper[i]) {
      residual[i] = values[i] - upper[i];
      activeMask[i] = 1.0;
    }
  }
}

double PotentialEnergyBoundedConstraintFunctions::func(ES::ConstRefVecXd x) const
{
  computeViolation(x);
  return residual.dot(residual) * 0.5;
}

void PotentialEnergyBoundedConstraintFunctions::gradient(ES::ConstRefVecXd x, ES::RefVecXd grad) const
{
  computeViolation(x);
  cnstt->jacobian(x, jac);
  ES::mv(jac, residual, grad, 1);
}

void PotentialEnergyBoundedConstraintFunctions::hessianInPlace(ES::ConstRefVecXd x, ES::SpMatD &hess) const
{
  if (hess.valuePtr()) {
    memset(hess.valuePtr(), 0, hess.nonZeros() * sizeof(double));
  }

  computeViolation(x);
  cnstt->jacobian(x, jac);

  for (int outer = 0; outer < jac.outerSize(); outer++) {
    for (ES::SpMatD::InnerIterator it(jac, outer); it; ++it) {
      it.valueRef() *= activeMask[it.row()];
    }
  }

  if (lambdaHessian.nonZeros()) {
    ES::mm(jac, jac, mmData, weightedJTJ, 1);
    ES::addSmallToBig(1.0, weightedJTJ, hess, 1.0, weightedJTJMapping, 1);

    cnstt->hessianInPlace(x, residual, lambdaHessian);
    ES::addSmallToBig(1.0, lambdaHessian, hess, 1.0, lambdaHessMapping, 1);
  }
  else {
    ES::mm(jac, jac, weightedJTJ, 1);
    memcpy(hess.valuePtr(), weightedJTJ.valuePtr(), sizeof(double) * weightedJTJ.nonZeros());
  }
}
