#pragma once

#include "deformation/deformationModel.h"
#include "EigenSupport.h"

#include <array>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

struct ShellDeformationModelCacheData : public DeformationModelCacheData
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

  ShellDeformationModelCacheData(int numPlasticParams, int numElasticParams);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
