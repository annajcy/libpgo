#include "vertex3DofLayout.h"

#include "simulation/simulationMesh.h"
#include "EigenSupport.h"

#include <atomic>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

namespace ES = EigenSupport;

Vertex3DofLayout::Vertex3DofLayout(const SimulationMesh &mesh):
  mesh_(mesh)
{
}

int Vertex3DofLayout::numGlobalDofs() const
{
  return mesh_.getNumVertices() * 3;
}

int Vertex3DofLayout::numLocalDofs(int) const
{
  return mesh_.getNumElementVertices() * 3;
}

void Vertex3DofLayout::getGlobalDofIndices(int ele, std::vector<int> &indices) const
{
  const int neleVtx = mesh_.getNumElementVertices();
  indices.resize(neleVtx * 3);
  for (int j = 0; j < neleVtx; j++) {
    int vid = mesh_.getVertexIndex(ele, j);
    if (vid >= 0) {
      indices[j * 3 + 0] = vid * 3 + 0;
      indices[j * 3 + 1] = vid * 3 + 1;
      indices[j * 3 + 2] = vid * 3 + 2;
    }
    else {
      indices[j * 3 + 0] = -1;
      indices[j * 3 + 1] = -1;
      indices[j * 3 + 2] = -1;
    }
  }
}

void Vertex3DofLayout::gather(int ele, const double *global, double *local) const
{
  const int neleVtx = mesh_.getNumElementVertices();
  for (int j = 0; j < neleVtx; j++) {
    int vid = mesh_.getVertexIndex(ele, j);
    if (vid >= 0) {
      local[j * 3 + 0] = global[vid * 3 + 0];
      local[j * 3 + 1] = global[vid * 3 + 1];
      local[j * 3 + 2] = global[vid * 3 + 2];
    }
    else {
      local[j * 3 + 0] = 0.0;
      local[j * 3 + 1] = 0.0;
      local[j * 3 + 2] = 0.0;
    }
  }
}

void Vertex3DofLayout::scatterAddGradient(int ele, const double *local, double *global) const
{
  const int neleVtx = mesh_.getNumElementVertices();
  for (int v = 0; v < neleVtx; v++) {
    int vid = mesh_.getVertexIndex(ele, v);
    if (vid >= 0) {
      for (int dof = 0; dof < 3; dof++) {
        std::atomic_ref<double> atomicGrad(global[vid * 3 + dof]);
        atomicGrad.fetch_add(local[v * 3 + dof]);
      }
    }
  }
}

void Vertex3DofLayout::addHessianSparsity(int ele,
  std::vector<ES::TripletD> &entries) const
{
  const int neleVtx = mesh_.getNumElementVertices();
  for (int vi = 0; vi < neleVtx; vi++) {
    int vidI = mesh_.getVertexIndex(ele, vi);
    if (vidI < 0)
      continue;
    for (int vj = 0; vj < neleVtx; vj++) {
      int vidJ = mesh_.getVertexIndex(ele, vj);
      if (vidJ < 0)
        continue;
      for (int dofi = 0; dofi < 3; dofi++) {
        for (int dofj = 0; dofj < 3; dofj++) {
          entries.emplace_back(vidI * 3 + dofi, vidJ * 3 + dofj, 1.0);
        }
      }
    }
  }
}

void Vertex3DofLayout::buildLocalToGlobalMatrixIndices(int ele,
  const ES::SpMatD &KTemplate,
  DynamicIndexMatrix &indices) const
{
  const int neleVtx = mesh_.getNumElementVertices();
  const int localDOFs = neleVtx * 3;
  indices.resize(localDOFs, localDOFs);
  indices.setConstant(-1);

  for (int vi = 0; vi < neleVtx; vi++) {
    int vidI = mesh_.getVertexIndex(ele, vi);
    for (int vj = 0; vj < neleVtx; vj++) {
      int vidJ = mesh_.getVertexIndex(ele, vj);
      for (int dofi = 0; dofi < 3; dofi++) {
        for (int dofj = 0; dofj < 3; dofj++) {
          int localRow = vi * 3 + dofi;
          int localCol = vj * 3 + dofj;

          if (vidI >= 0 && vidJ >= 0) {
            int globalRow = vidI * 3 + dofi;
            int globalCol = vidJ * 3 + dofj;
            indices(localRow, localCol) = ES::findEntryOffset(KTemplate, globalRow, globalCol);
          }
          else {
            indices(localRow, localCol) = -1;
          }
        }
      }
    }
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo
