#include "volumetricElementModelCacheData.h"

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

VolumetricElementModelCacheData::VolumetricElementModelCacheData(
  int numNodes, int numQuadPts, int np, int ne):
  numNodes(numNodes),
  numQuadPts(numQuadPts),
  localDofs(numNodes * 3),
  numPlasticParams(np),
  numElasticParams(ne)
{
  x.resize(3, numNodes);

  Fp.resize(numQuadPts, ES::M3d::Identity());
  FpInv.resize(numQuadPts, ES::M3d::Identity());
  detFp.resize(numQuadPts, 1.0);

  ddetA_da.resize(numQuadPts, ES::VXd::Zero(np));
  d2detA_da2.resize(numQuadPts, ES::MXd::Zero(np, np));
  dAInv_dai.resize(numQuadPts);
  d2AInv_dai_daj.resize(numQuadPts);
  for (int q = 0; q < numQuadPts; q++) {
    dAInv_dai[q].resize(np, ES::M3d::Zero());
    d2AInv_dai_daj[q].resize(np * np, ES::M3d::Zero());
  }

  Fref.resize(numQuadPts, ES::M3d::Zero());
  Fe.resize(numQuadPts, ES::M3d::Zero());
  U.resize(numQuadPts, ES::M3d::Zero());
  V.resize(numQuadPts, ES::M3d::Zero());
  S.resize(numQuadPts, ES::V3d::Zero());

  dFdx.resize(numQuadPts);
  Bm.resize(numQuadPts);
  for (int q = 0; q < numQuadPts; q++) {
    dFdx[q].resize(9, localDofs);
    dFdx[q].setZero();
    Bm[q].resize(3, numNodes);
    Bm[q].setZero();
  }

  plasticParamsValue.setZero(np);
  plasticParamsDeriv.setIdentity(np, np);
  elasticParamsValue.setZero(ne);
  elasticParamsDeriv.setIdentity(ne, ne);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
