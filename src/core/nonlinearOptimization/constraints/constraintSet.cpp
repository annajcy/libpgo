/*
author: Bohan Wang
copyright to USC
*/

#include "constraintSet.h"

#include "EigenSupport.h"

#include <cstring>
#include <numeric>
#include <stdexcept>

using namespace pgo;
using namespace pgo::NonlinearOptimization;
namespace ES = pgo::EigenSupport;

ConstraintSet::ConstraintSet(int numDofs, std::vector<Term> terms):
  ConstraintFunctions(numDofs), terms_(std::move(terms))
{
  if (terms_.empty()) {
    throw std::invalid_argument("ConstraintSet requires at least one term");
  }

  std::vector<ES::TripletD> entriesj, entriesh;
  rowOffsets_.reserve(terms_.size() + 1);
  childJacobianBuffers_.reserve(terms_.size());
  childHessianBuffers_.reserve(terms_.size());
  rowOffsets_.push_back(0);

  for (const Term &term : terms_) {
    if (!term.functions) {
      throw std::invalid_argument("ConstraintSet term functions must be non-null");
    }

    const ES::SpMatD &jacTemplate = term.functions->getJacobianTemplate();
    if (jacTemplate.cols() != numDofs) {
      throw std::invalid_argument("ConstraintSet term num_dofs mismatch");
    }

    ES::SpMatD jac, hess;
    term.functions->createJacobian(jac);
    term.functions->hessianAlloc(hess);
    if (hess.rows() != numDofs || hess.cols() != numDofs) {
      throw std::invalid_argument("ConstraintSet term Hessian shape mismatch");
    }

    const ES::IDX rowOffset = rowOffsets_.back();
    for (Eigen::Index outeri = 0; outeri < jac.outerSize(); outeri++) {
      for (ES::SpMatD::InnerIterator it(jac, outeri); it; ++it) {
        entriesj.emplace_back(rowOffset + it.row(), it.col(), 1.0);
      }
    }

    for (Eigen::Index outeri = 0; outeri < hess.outerSize(); outeri++) {
      for (ES::SpMatD::InnerIterator it(hess, outeri); it; ++it) {
        entriesh.emplace_back(it.row(), it.col(), 1.0);
      }
    }

    childJacobianBuffers_.push_back(jac);
    childHessianBuffers_.push_back(hess);
    rowOffsets_.push_back(rowOffset + jac.rows());

    isLinear_ = isLinear_ && term.functions->isLinear();
    hasHessianVectorRoutine_ = hasHessianVectorRoutine_ && term.functions->hasHessianVector();
  }

  jacobianTemplate.resize(rowOffsets_.back(), numDofs);
  jacobianTemplate.setFromTriplets(entriesj.begin(), entriesj.end());

  lambdahTemplate.resize(numDofs, numDofs);
  lambdahTemplate.setFromTriplets(entriesh.begin(), entriesh.end());

  std::vector<int> dofsCol(numDofs);
  std::iota(dofsCol.begin(), dofsCol.end(), 0);

  for (size_t i = 0; i < terms_.size(); i++) {
    ES::SpMatI mapping;
    std::vector<int> dofsRow;
    dofsRow.reserve(static_cast<size_t>(rowOffsets_[i + 1] - rowOffsets_[i]));
    for (ES::IDX row = rowOffsets_[i]; row < rowOffsets_[i + 1]; row++) {
      dofsRow.push_back((int)row);
    }

    ES::small2Big(childJacobianBuffers_[i], jacobianTemplate, dofsRow, dofsCol, mapping);
    jacobianMappings_.push_back(mapping);

    ES::SpMatI mappingh;
    ES::small2Big(childHessianBuffers_[i], lambdahTemplate, dofsCol, mappingh);
    hessianMappings_.push_back(mappingh);
  }
}

ConstraintSet::~ConstraintSet()
{
}

void ConstraintSet::func(ES::ConstRefVecXd x, ES::RefVecXd g) const
{
  for (size_t i = 0; i < terms_.size(); i++) {
    ES::IDX dim = rowOffsets_[i + 1] - rowOffsets_[i];
    terms_[i].functions->func(x, g.segment(rowOffsets_[i], dim));
  }
}

void ConstraintSet::jacobian(ES::ConstRefVecXd x, ES::SpMatD &jac) const
{
  for (size_t i = 0; i < terms_.size(); i++) {
    terms_[i].functions->jacobian(x, childJacobianBuffers_[i]);
    ES::addSmallToBig(1.0, childJacobianBuffers_[i], jac, 0.0, jacobianMappings_[i]);
  }
}

void ConstraintSet::hessianInPlace(ES::ConstRefVecXd x, ES::ConstRefVecXd lambda, ES::SpMatD &hess) const
{
  if (hess.valuePtr()) {
    memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());
  }

  for (size_t i = 0; i < terms_.size(); i++) {
    if (childHessianBuffers_[i].nonZeros() == 0) {
      continue;
    }

    ES::IDX dim = rowOffsets_[i + 1] - rowOffsets_[i];
    terms_[i].functions->hessianInPlace(x, lambda.segment(rowOffsets_[i], dim), childHessianBuffers_[i]);
    ES::addSmallToBig(1.0, childHessianBuffers_[i], hess, 1.0, hessianMappings_[i]);
  }
}

void ConstraintSet::hessianVector(ES::ConstRefVecXd x, ES::ConstRefVecXd lambda, ES::ConstRefVecXd vec, ES::RefVecXd hessVec) const
{
  hessVec.setZero();

  if (!hasHessianVectorRoutine_) {
    ES::SpMatD hess;
    hessianAlloc(hess);
    hessianInPlace(x, lambda, hess);
    ES::mv(hess, vec, hessVec);
    return;
  }

  ES::VXd termVec(nAll);
  for (size_t i = 0; i < terms_.size(); i++) {
    ES::IDX dim = rowOffsets_[i + 1] - rowOffsets_[i];
    termVec.setZero();
    terms_[i].functions->hessianVector(x, lambda.segment(rowOffsets_[i], dim), vec, termVec);
    hessVec += termVec;
  }
}
