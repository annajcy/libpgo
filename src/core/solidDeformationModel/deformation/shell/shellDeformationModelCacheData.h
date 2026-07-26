#pragma once

#include "EigenSupport.h"

#include <array>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

struct ShellDeformationModelCacheData
{
  int numPlasticParams = 0;
  int numElasticParams = 0;

  std::array<ES::V3d, 6> x;
  ES::M2d a = ES::M2d::Zero();
  ES::M2d abar = ES::M2d::Zero();
  ES::M2d b = ES::M2d::Zero();
  ES::M2d bbar = ES::M2d::Zero();
  double area = 0.0;

  ES::VXd plasticParamsValue;
  ES::VXd elasticParamsValue;

  mutable ES::VXd elasticDpsiDparamScratch;
  mutable ES::VXd plasticDAreaDparamScratch;
  mutable ES::MXd plasticDAbarDparamScratch;
  mutable ES::MXd plasticDBbarDparamScratch;
  mutable ES::MXd elasticDpsiDaDparamScratch;
  mutable ES::MXd elasticDpsiDbDparamScratch;
  mutable ES::MXd elasticDpsiDabarDparamScratch;
  mutable ES::MXd elasticDpsiDbbarDparamScratch;
  mutable ES::MXd mixedDerivativeScratch;

  ShellDeformationModelCacheData(int numPlasticParams, int numElasticParams);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
