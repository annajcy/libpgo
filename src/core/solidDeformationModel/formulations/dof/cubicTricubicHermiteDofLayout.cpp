#include "cubicTricubicHermiteDofLayout.h"

#include "simulation/simulationMesh.h"
#include "EigenSupport.h"

#include <atomic>

namespace pgo
{
namespace SolidDeformationModel
{

namespace ES = EigenSupport;

CubicTricubicHermiteDofLayout::CubicTricubicHermiteDofLayout(const SimulationMesh &mesh):
  mesh_(mesh)
{
}

int CubicTricubicHermiteDofLayout::numGlobalDofs() const
{
  return mesh_.getNumVertices() * kDofsPerVertex;
}

int CubicTricubicHermiteDofLayout::numLocalDofs(int) const
{
  return kLocalDofs;
}

void CubicTricubicHermiteDofLayout::getGlobalDofIndices(int ele, std::vector<int> &indices) const
{
  indices.resize(kLocalDofs);
  for (int corner = 0; corner < 8; corner++) {
    int vid = mesh_.getVertexIndex(ele, corner);
    for (int mode = 0; mode < kModesPerVertex; mode++) {
      for (int coord = 0; coord < 3; coord++) {
        int local = (corner * kModesPerVertex + mode) * 3 + coord;
        indices[local] = (vid >= 0) ? (vid * kDofsPerVertex + mode * 3 + coord) : -1;
      }
    }
  }
}

void CubicTricubicHermiteDofLayout::gather(int ele, const double *global, double *local) const
{
  for (int corner = 0; corner < 8; corner++) {
    int vid = mesh_.getVertexIndex(ele, corner);
    for (int mode = 0; mode < kModesPerVertex; mode++) {
      for (int coord = 0; coord < 3; coord++) {
        int localIdx = (corner * kModesPerVertex + mode) * 3 + coord;
        if (vid >= 0)
          local[localIdx] = global[vid * kDofsPerVertex + mode * 3 + coord];
        else
          local[localIdx] = 0.0;
      }
    }
  }
}

void CubicTricubicHermiteDofLayout::scatterAddGradient(int ele, const double *local, double *global) const
{
  for (int corner = 0; corner < 8; corner++) {
    int vid = mesh_.getVertexIndex(ele, corner);
    if (vid < 0)
      continue;
    for (int mode = 0; mode < kModesPerVertex; mode++) {
      for (int coord = 0; coord < 3; coord++) {
        int localIdx = (corner * kModesPerVertex + mode) * 3 + coord;
        std::atomic_ref<double> atomicGrad(global[vid * kDofsPerVertex + mode * 3 + coord]);
        atomicGrad.fetch_add(local[localIdx]);
      }
    }
  }
}

void CubicTricubicHermiteDofLayout::addHessianSparsity(int ele,
  std::vector<ES::TripletD> &entries) const
{
  std::vector<int> g;
  getGlobalDofIndices(ele, g);
  for (int i = 0; i < kLocalDofs; i++) {
    if (g[i] < 0)
      continue;
    for (int j = 0; j < kLocalDofs; j++) {
      if (g[j] < 0)
        continue;
      entries.emplace_back(g[i], g[j], 1.0);
    }
  }
}

void CubicTricubicHermiteDofLayout::buildLocalToGlobalMatrixIndices(int ele,
  const ES::SpMatD &KTemplate,
  DynamicIndexMatrix &indices) const
{
  std::vector<int> g;
  getGlobalDofIndices(ele, g);

  indices.resize(kLocalDofs, kLocalDofs);
  indices.setConstant(-1);
  for (int i = 0; i < kLocalDofs; i++) {
    if (g[i] < 0)
      continue;
    for (int j = 0; j < kLocalDofs; j++) {
      if (g[j] < 0)
        continue;
      indices(i, j) = ES::findEntryOffset(KTemplate, g[i], g[j]);
    }
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo
