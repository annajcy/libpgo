#include "shellDeformationModelCacheData.h"

#include <algorithm>

namespace pgo
{
namespace SolidDeformationModel
{

ShellDeformationModelCacheData::ShellDeformationModelCacheData(
  int numPlasticParams_, int numElasticParams_):
  numPlasticParams(numPlasticParams_),
  numElasticParams(numElasticParams_),
  plasticParamsValue(numPlasticParams_),
  elasticParamsValue(numElasticParams_)
{
  for (auto &xi : x)
    xi.setZero();
  plasticParamsValue.setZero();
  elasticParamsValue.setZero();

  const int maxParams = std::max(numPlasticParams, numElasticParams);
  elasticDpsiDparamScratch.resize(numElasticParams);
  plasticDAreaDparamScratch.resize(numPlasticParams);
  plasticDAbarDparamScratch.resize(4, numPlasticParams);
  plasticDBbarDparamScratch.resize(4, numPlasticParams);
  elasticDpsiDaDparamScratch.resize(4, numElasticParams);
  elasticDpsiDbDparamScratch.resize(4, numElasticParams);
  elasticDpsiDabarDparamScratch.resize(4, numElasticParams);
  elasticDpsiDbbarDparamScratch.resize(4, numElasticParams);
  mixedDerivativeScratch.resize(18, maxParams);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
