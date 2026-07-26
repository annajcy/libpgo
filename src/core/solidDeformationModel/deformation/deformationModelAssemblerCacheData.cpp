/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "deformationModelAssemblerCacheData.h"

#include <algorithm>
#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

DeformationModelAssemblerCacheData::ElementScratch::ElementScratch(
  int localDofs, int maxMaterialLocations, int maxMaterialParams, int maxLocalParams,
  const DeformationModel &model):
  cacheData_(model.allocateCacheData())
{
  if (!cacheData_)
    throw std::runtime_error("Element model returned null cache data.");

  localPosition.resize(localDofs);
  localDirection.resize(localDofs);
  localGradient.resize(localDofs);

  elasticParamValues.resize(maxMaterialLocations * maxMaterialParams);
  plasticParamValues.resize(maxMaterialLocations * maxMaterialParams);
  localParamValues.resize(maxLocalParams);
  rawParamGradient.resize(maxMaterialParams);
  localParamGradient.resize(maxLocalParams);

  localParamHessian.resize(maxLocalParams, maxLocalParams);
  paramWorkMatrix.resize(maxMaterialParams, maxLocalParams);
  localMixedMatrix.resize(localDofs, maxLocalParams);
  paramDerivativeData.resize(maxMaterialParams, maxLocalParams);
  paramDerivativeData2.resize(maxMaterialParams, maxLocalParams);
  localMatrixData.resize(static_cast<std::size_t>(std::max({ localDofs * localDofs,
    localDofs * maxMaterialParams,
    maxMaterialParams * maxMaterialParams })));
  materialLocationValues.resize(std::max(16, maxMaterialLocations));
}

namespace
{
std::span<EigenSupport::MXd> prepareMappingHessians(
  std::vector<EigenSupport::MXd> &hessians,
  int numChannels,
  int numLocalDofs)
{
  hessians.resize(static_cast<std::size_t>(numChannels));
  for (EigenSupport::MXd &hessian : hessians) {
    if (hessian.rows() != numLocalDofs || hessian.cols() != numLocalDofs)
      hessian.resize(numLocalDofs, numLocalDofs);
  }
  return std::span<EigenSupport::MXd>(hessians.data(), hessians.size());
}
}  // namespace

std::span<EigenSupport::MXd>
DeformationModelAssemblerCacheData::ElementScratch::preparePlasticParamMappingHessians(
  int numChannels, int numLocalDofs)
{
  return prepareMappingHessians(
    plasticParamMappingHessians, numChannels, numLocalDofs);
}

std::span<EigenSupport::MXd>
DeformationModelAssemblerCacheData::ElementScratch::prepareElasticParamMappingHessians(
  int numChannels, int numLocalDofs)
{
  return prepareMappingHessians(
    elasticParamMappingHessians, numChannels, numLocalDofs);
}

DeformationModelAssemblerCacheData::DeformationModelAssemblerCacheData(
  int localDofs, int maxMaterialLocations, int maxMaterialParams, int maxLocalParams,
  std::span<const std::reference_wrapper<const DeformationModel>> models)
{
  elementScratch_.reserve(models.size());
  for (const auto &modelRef : models) {
    elementScratch_.emplace_back(
      localDofs, maxMaterialLocations, maxMaterialParams, maxLocalParams,
      modelRef.get());
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo
