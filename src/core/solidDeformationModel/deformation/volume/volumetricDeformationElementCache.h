#pragma once

#include "EigenSupport.h"
#include "material/elastic/elasticModel3DDeformationGradient.h"

#include <vector>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

struct VolumetricDeformationElementCache
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
  std::vector<std::vector<ES::M3d>> dAInv_dai;

  std::vector<ES::M3d> Fref;
  std::vector<SpectralState> spectralState;
  std::vector<M9xNDOF> dFdx;
  std::vector<M3xN> Bm;

  // Material values are element-wise and shared by every quadrature point.
  ES::VXd plasticParamsValue;
  ES::VXd elasticParamsValue;

  mutable ES::VXd dpsiDxScratch;
  mutable ES::VXd localDofScratch;
  mutable M9xNDOF d2FdxdaScratch;
  mutable ES::VXd dpsiDparamScratch;
  mutable ES::MXd dPdbScratch;
  mutable ES::VXd materialGradientScratch;
  mutable ES::MXd materialMixedScratch;
  mutable ES::MXd materialMixedLocationScratch;

  VolumetricDeformationElementCache() = default;
  VolumetricDeformationElementCache(int numNodes, int numQuadPts,
    int numPlasticParams, int numElasticParams);
};

}  // namespace SolidDeformationModel
}  // namespace pgo
