#pragma once

#include "../../deformationModel.h"
#include "../../elasticModel2DFundamentalForms.h"
#include "../../plasticModel2DFundamentalForms.h"
#include "EigenSupport.h"

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

struct KoiterShellElementModelCacheData : public DeformationModelCacheData
{
  ES::V3d x[6];
  ES::M2d a, abar, b, bbar;
  ES::V18d elasticParams, plasticParams;
  double area;

  ElasticModel2DFundamentalForms *elasticModel;
  PlasticModel2DFundamentalForms *plasticModel;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
