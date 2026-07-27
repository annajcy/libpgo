/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "deformationModelAssemblerElementWorkspace.h"

#include <algorithm>
#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

DeformationModelAssemblerElementWorkspace::DeformationModelAssemblerElementWorkspace(
  int localDofs, int maxMaterialLocations, int maxMaterialParams,
  int maxLocalParams, const DeformationModel &model):
  evaluator_(model.createEvaluator())
{
  if (!evaluator_)
    throw std::runtime_error("Element model returned null evaluator.");

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
std::span<EigenSupport::MXd> prepareEvaluatorHessians(
  std::vector<EigenSupport::MXd> &hessians,
  int numChannels, int numLocalParameters)
{
  hessians.resize(static_cast<std::size_t>(numChannels));
  for (EigenSupport::MXd &hessian : hessians) {
    if (hessian.rows() != numLocalParameters || hessian.cols() != numLocalParameters)
      hessian.resize(numLocalParameters, numLocalParameters);
  }
  return std::span<EigenSupport::MXd>(hessians.data(), hessians.size());
}
}  // namespace

std::span<EigenSupport::MXd>
DeformationModelAssemblerElementWorkspace::preparePlasticParamEvaluatorHessians(
  int numChannels, int numLocalParameters)
{
  return prepareEvaluatorHessians(
    plasticParamEvaluatorHessians, numChannels, numLocalParameters);
}

std::span<EigenSupport::MXd>
DeformationModelAssemblerElementWorkspace::prepareElasticParamEvaluatorHessians(
  int numChannels, int numLocalParameters)
{
  return prepareEvaluatorHessians(
    elasticParamEvaluatorHessians, numChannels, numLocalParameters);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
