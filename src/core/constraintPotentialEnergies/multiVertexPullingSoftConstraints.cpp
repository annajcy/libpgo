/*
author: Bohan Wang
copyright to USC, MIT
*/

#include "multiVertexPullingSoftConstraints.h"

#include "pgoLogging.h"
#include "parallel/parallelFor.h"
#include "parallel/parallelReduce.h"


#include <iostream>

using namespace pgo::ConstraintPotentialEnergies;

namespace ES = pgo::EigenSupport;

MultipleVertexPulling::MultipleVertexPulling(
  EigenSupport::SpMatD Koff,
  EigenSupport::VXd restPositionsAll,
  std::vector<int> vertexIndices,
  EigenSupport::VXd targetPositions,
  double coeff,
  bool isDisplacement):
  PotentialEnergyAligningMeshConnectivity(Koff),
  tgtp_(std::move(targetPositions)),
  restpAll_(std::move(restPositionsAll)),
  vertexIndices_(std::move(vertexIndices)),
  coeffAll_(coeff),
  isDisplacement_(isDisplacement)
{
  int numPts = static_cast<int>(vertexIndices_.size());
  KIndices.assign(numPts, M3i::Constant(-1));

  for (size_t vi = 0; vi < vertexIndices_.size(); vi++) {
    int vid = vertexIndices_[vi];

    KIndices[vi] = Eigen::Matrix<ES::IDX, 3, 3>::Constant(-1);
    for (int i = 0; i < 3; i++) {
      KIndices[vi](i, i) = ES::findEntryOffset(Koff, vid * 3 + i, vid * 3 + i);
      PGO_ALOG(KIndices[vi](i, i) >= 0);
    }
  }

  coeffs_.setConstant(vertexIndices_.size(), 1.0);
  masks_.setOnes(vertexIndices_.size() * 3);
}

void MultipleVertexPulling::setTargetPositions(ES::VXd tgt)
{
  tgtp_ = std::move(tgt);
}

double MultipleVertexPulling::func(ES::ConstRefVecXd u) const
{
  auto computeEnergyForVertex = [&](size_t i) -> double {
    ES::V3d p;
    if (isDisplacement_) {
      p = u.segment<3>(vertexIndices_[i] * 3) + restpAll_.segment<3>(vertexIndices_[i] * 3);
    }
    else {
      p = u.segment<3>(vertexIndices_[i] * 3);
    }

    ES::V3d diff = p - tgtp_.segment<3>(i * 3);
    diff = diff.cwiseProduct(masks_.segment<3>(i * 3));

    return diff.dot(diff) * 0.5 * coeffs_[i];
  };

  double eng = pgo::parallel::parallelReduce(size_t{ 0 }, vertexIndices_.size(), 0.0,  //
    [&](size_t rangeBegin, size_t rangeEnd, double init) -> double {
      for (size_t i = rangeBegin; i != rangeEnd; ++i) {
        init += computeEnergyForVertex(i);
      }
      return init; }, std::plus<double>());

  return eng * coeffAll_;
}

void MultipleVertexPulling::gradient(ES::ConstRefVecXd u, ES::RefVecXd grad) const
{
  grad.setZero();

  auto computeGradientForVertex = [&](size_t i) {
    int vtx = vertexIndices_[i];

    ES::V3d p;
    if (isDisplacement_) {
      p = u.segment<3>(vertexIndices_[i] * 3) + restpAll_.segment<3>(vertexIndices_[i] * 3);
    }
    else {
      p = u.segment<3>(vertexIndices_[i] * 3);
    }

    ES::V3d diff = p - tgtp_.segment<3>(i * 3);
    diff = diff.cwiseProduct(masks_.segment<3>(i * 3)).cwiseProduct(masks_.segment<3>(i * 3));

    grad.segment<3>(vtx * 3) = diff;
    grad.segment<3>(vtx * 3) *= coeffs_[i];
  };

  pgo::parallel::parallelForChunks(size_t{ 0 }, vertexIndices_.size(),
    [&](size_t rangeBegin, size_t rangeEnd) {
      for (size_t i = rangeBegin; i != rangeEnd; ++i) {
        computeGradientForVertex(i);
      }
    });

  grad *= coeffAll_;
}

void MultipleVertexPulling::hessianInPlace(ES::ConstRefVecXd, ES::SpMatD &hess) const
{
  memset(hess.valuePtr(), 0, sizeof(double) * hess.nonZeros());

  auto computeHessianForVertex = [&](size_t vi) {
    ES::V3d w = masks_.segment<3>(vi * 3).cwiseProduct(masks_.segment<3>(vi * 3));

    for (int i = 0; i < 3; i++) {
      hess.valuePtr()[KIndices[vi](i, i)] = w[i] * coeffs_[vi] * coeffAll_;
    }
  };

  pgo::parallel::parallelForChunks(size_t{ 0 }, vertexIndices_.size(),
    [&](size_t rangeBegin, size_t rangeEnd) {
      for (size_t vi = rangeBegin; vi != rangeEnd; ++vi) {
        computeHessianForVertex(vi);
      }
    });
}

void MultipleVertexPulling::printErrorInfo(ES::ConstRefVecXd u) const
{
  ES::VXd diff(vertexIndices_.size() * 3);
  ES::VXd diff1(vertexIndices_.size() * 3);
  for (size_t i = 0; i < vertexIndices_.size(); i++) {
    ES::V3d p;
    if (isDisplacement_) {
      p = u.segment<3>(vertexIndices_[i] * 3) + restpAll_.segment<3>(vertexIndices_[i] * 3);
    }
    else {
      p = u.segment<3>(vertexIndices_[i] * 3);
    }

    diff.segment<3>(i * 3) = p - tgtp_.segment<3>(i * 3);
    diff.segment<3>(i * 3) = diff.segment<3>(i * 3).cwiseProduct(masks_.segment<3>(i * 3));
    diff1.segment<3>(i * 3) = diff.segment<3>(i * 3) * coeffs_[i];
  }

  std::cout << "  ||Wu - bcu||=" << diff.squaredNorm() << std::endl;
  std::cout << "  ||Wu - bcu||_Z=" << diff1.dot(diff) << std::endl;
  std::cout << "  func(u)=" << func(u) << std::endl;

  double mind = 1e100, maxd = 0, avgd = 0;
  for (ES::IDX i = 0; i < diff.size() / 3; i++) {
    if (coeffs_[i] < 1e-9)
      continue;

    double d = diff.segment<3>(i * 3).norm();
    mind = std::min(mind, d);
    maxd = std::max(maxd, d);
    avgd += d;
  }
  avgd /= (double)(diff.size() / 3);
  std::cout << "distance error info: " << mind << '/' << avgd << '/' << maxd << std::endl;
}
