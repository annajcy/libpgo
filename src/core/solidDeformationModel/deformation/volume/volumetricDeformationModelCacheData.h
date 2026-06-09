#pragma once

#include "deformation/deformationModel.h"
#include "EigenSupport.h"

#include <vector>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

struct VolumetricDeformationModelCacheData : public DeformationModelCacheData
{
  using M3xN = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  using M9xNDOF = Eigen::Matrix<double, 9, Eigen::Dynamic>;

  int numNodes = 0;
  int numQuadPts = 0;
  int localDofs = 0;
  int numPlasticParams = 0;
  int numElasticParams = 0;

  M3xN x;

  std::vector<ES::M3d> Fp;
  std::vector<ES::M3d> FpInv;
  std::vector<double> detFp;

  std::vector<ES::VXd> ddetA_da;
  std::vector<ES::MXd> d2detA_da2;
  std::vector<std::vector<ES::M3d>> dAInv_dai;
  std::vector<std::vector<ES::M3d>> d2AInv_dai_daj;

  std::vector<ES::M3d> Fref;
  std::vector<ES::M3d> Fe;
  std::vector<ES::M3d> U, V;
  std::vector<ES::V3d> S;
  std::vector<M9xNDOF> dFdx;
  std::vector<M3xN> Bm;

  std::vector<ES::VXd> plasticParamsValue;
  std::vector<ES::VXd> elasticParamsValue;

  VolumetricDeformationModelCacheData(int numNodes, int numQuadPts,
    int numPlasticParams, int numElasticParams);

  ES::M3d &d2AInv(int q, int i, int j)
  {
    return d2AInv_dai_daj[q][i * numPlasticParams + j];
  }
  const ES::M3d &d2AInv(int q, int i, int j) const
  {
    return d2AInv_dai_daj[q][i * numPlasticParams + j];
  }
};

}  // namespace SolidDeformationModel
}  // namespace pgo
