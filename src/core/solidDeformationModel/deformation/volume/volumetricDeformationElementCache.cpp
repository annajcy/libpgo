#include "volumetricDeformationElementCache.h"

#include <algorithm>

namespace pgo
{
namespace ES = pgo::EigenSupport;
namespace SolidDeformationModel
{

VolumetricDeformationElementCache::VolumetricDeformationElementCache(
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
  dAInv_dai.resize(numQuadPts);
  for (int q = 0; q < numQuadPts; q++) {
    dAInv_dai[q].resize(np, ES::M3d::Zero());
  }

  Fref.resize(numQuadPts, ES::M3d::Zero());
  spectralState.resize(numQuadPts);

  dFdx.resize(numQuadPts);
  Bm.resize(numQuadPts);
  for (int q = 0; q < numQuadPts; q++) {
    dFdx[q].resize(9, localDofs);
    dFdx[q].setZero();
    Bm[q].resize(3, numNodes);
    Bm[q].setZero();
  }

  plasticParamsValue = ES::VXd::Zero(np);
  elasticParamsValue = ES::VXd::Zero(ne);

  dpsiDxScratch.resize(localDofs);
  localDofScratch.resize(localDofs);
  d2FdxdaScratch.resize(9, localDofs);
  dpsiDparamScratch.resize(ne);
  dPdbScratch.resize(9, ne);
  const int maxParams = std::max(np, ne);
  materialGradientScratch.resize(maxParams);
  materialMixedScratch.resize(localDofs, maxParams);
  materialMixedLocationScratch.resize(localDofs, maxParams);
}

}  // namespace SolidDeformationModel
}  // namespace pgo
