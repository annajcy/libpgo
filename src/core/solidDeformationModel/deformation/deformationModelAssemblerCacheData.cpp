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
  std::size_t maxMappingHessianEntries,
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
  paramDerivativeData.resize(static_cast<std::size_t>(maxMaterialParams) * maxLocalParams);
  paramDerivativeData2.resize(static_cast<std::size_t>(maxMaterialParams) * maxLocalParams);
  paramMappingHessianData.resize(maxMappingHessianEntries);
  localMatrixData.resize(static_cast<std::size_t>(std::max({ localDofs * localDofs,
    localDofs * maxMaterialParams,
    maxMaterialParams * maxMaterialParams })));
  materialLocationValues.resize(std::max(16, maxMaterialLocations));
}

DeformationModelAssemblerCacheData::DeformationModelAssemblerCacheData(
  int localDofs, int maxMaterialLocations, int maxMaterialParams, int maxLocalParams,
  std::size_t maxMappingHessianEntries,
  const std::vector<const DeformationModel *> &models)
{
  elementScratch_.reserve(models.size());
  for (const DeformationModel *model : models) {
    if (model == nullptr)
      throw std::invalid_argument("DeformationModelAssemblerCacheData requires non-null element models.");
    elementScratch_.emplace_back(
      localDofs, maxMaterialLocations, maxMaterialParams, maxLocalParams,
      maxMappingHessianEntries, *model);
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo
