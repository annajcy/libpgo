#include "ipc/broadPhase/surfaceIPCBroadPhase.h"

#include "ipc/broadPhase/surfaceIPCBroadPhaseInternal.h"
#include "ipc/geometry/ipcDistancePrimitives.h"


#include <algorithm>
#include <cstdint>
#include <vector>

namespace pgo
{
namespace Contact
{
namespace IPC
{

using namespace broad_phase_detail;

void buildSelfPairs(
  const SurfaceIPCTopology &topology,
  EigenSupport::ConstRefVecXd positions,
  double dhat,
  SelfPairSet &pairs)
{
  using namespace pgo::EigenSupport;


  pairs.clear();

  const double inflate = dhat;

  auto getV = [&](int i) -> V3d {
    return positions.segment<3>(3 * i);
  };

  int nTri = (int)topology.triangles.size();
  int nEdge = (int)topology.edges.size();

  // --- Build AABBs ---
  std::vector<SpatialHashGrid::AABB> vertBox(topology.numVerts);
  std::vector<SpatialHashGrid::AABB> triBox(nTri);
  std::vector<SpatialHashGrid::AABB> edgeBox(nEdge);
  {
    buildVertexAABBs(vertBox, topology.numVerts, getV, inflate);
    buildTriangleAABBs(triBox, nTri, topology.triangles, getV, inflate);
    buildEdgeAABBs(edgeBox, nEdge, topology.edges, getV, inflate);
  }

  double avgBoxDiag = 0;
  for (const auto &aabb : triBox) {
    avgBoxDiag += (aabb.hi - aabb.lo).norm();
  }

  double cellSize = nTri > 0 ? std::max(avgBoxDiag / nTri, 1e-6) : std::max(1e-6, dhat);
  double dhat2 = dhat * dhat;

  // --- PT pairs ---
  {
    SpatialHashGrid triHash(nTri);
    triHash.setCellSize(cellSize);
    {
      triHash.build(triBox);
    }
    {
      collectPairsParallel<PTPair>(nTri, 0, topology.numVerts,
        [&](int rangeBegin, int rangeEnd,
          std::vector<int> &visited,
          std::vector<int> &candidates,
          std::vector<PTPair> &localPairs) {
          for (int vi = rangeBegin; vi < rangeEnd; ++vi) {
            candidates.clear();
            triHash.queryOverlapping(vertBox[vi], triBox, -1, visited, vi + 1, candidates);

            V3d vp = getV(vi);
            for (int fi : candidates) {
              auto &tri = topology.triangles[fi];
              if (vi == tri[0] || vi == tri[1] || vi == tri[2])
                continue;

              V3d vt0 = getV(tri[0]), vt1 = getV(tri[1]), vt2 = getV(tri[2]);
              double d2 = distance::computePTSqDist(vp, vt0, vt1, vt2);
              if (d2 < dhat2)
                localPairs.push_back({ vi, tri[0], tri[1], tri[2],
                  topology.vertexArea[vi] * topology.triArea[fi] });
            }
          }
        },
        pairs.ptPairs);
    }
  }

  // --- EE pairs ---
  {
    SpatialHashGrid edgeHash(nEdge);
    edgeHash.setCellSize(cellSize);
    {
      edgeHash.build(edgeBox);
    }
    {
      collectPairsParallel<EEPair>(nEdge, 0, nEdge,
        [&](int rangeBegin, int rangeEnd,
          std::vector<int> &visited,
          std::vector<int> &candidates,
          std::vector<EEPair> &localPairs) {
          for (int ei = rangeBegin; ei < rangeEnd; ++ei) {
            candidates.clear();
            edgeHash.queryOverlappingAfter(edgeBox[ei], edgeBox, ei, visited, ei + 1, candidates);

            int a0 = topology.edges[ei][0], a1 = topology.edges[ei][1];
            V3d va0 = getV(a0), va1 = getV(a1);
            for (int ej : candidates) {
              int b0 = topology.edges[ej][0], b1 = topology.edges[ej][1];
              if (a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1)
                continue;

              V3d vb0 = getV(b0), vb1 = getV(b1);
              double d2 = distance::computeEESqDist(va0, va1, vb0, vb1);
              if (d2 < dhat2)
                localPairs.push_back({ a0, a1, b0, b1,
                  topology.edgeLength[ei] * topology.edgeLength[ej] });
            }
          }
        },
        pairs.eePairs);
    }
  }
}

void buildSelfPairsLineSearchSuperset(
  const SurfaceIPCTopology &topology,
  EigenSupport::ConstRefVecXd positions,
  EigenSupport::ConstRefVecXd displacements,
  double dhat,
  SelfPairSet &pairs)
{
  using namespace pgo::EigenSupport;


  pairs.clear();

  auto getV = [&](int i) -> V3d {
    return positions.segment<3>(3 * i);
  };
  auto getDV = [&](int i) -> V3d {
    return displacements.segment<3>(3 * i);
  };

  const int nTri = static_cast<int>(topology.triangles.size());
  const int nEdge = static_cast<int>(topology.edges.size());

  std::vector<SpatialHashGrid::AABB> vertBox(topology.numVerts);
  std::vector<SpatialHashGrid::AABB> triBox(nTri);
  std::vector<SpatialHashGrid::AABB> edgeBox(nEdge);
  {
    buildSweptVertexAABBs(vertBox, topology.numVerts, getV, getDV, dhat);
    buildSweptTriangleAABBs(triBox, nTri, topology.triangles, getV, getDV, dhat);
    buildSweptEdgeAABBs(edgeBox, nEdge, topology.edges, getV, getDV, dhat);
  }

  double avgBoxDiag = 0.0;
  for (const auto &aabb : triBox)
    avgBoxDiag += (aabb.hi - aabb.lo).norm();
  const double cellSize = nTri > 0 ? std::max(avgBoxDiag / nTri, 1e-6) : std::max(1e-6, dhat);

  {
    SpatialHashGrid triHash(nTri);
    triHash.setCellSize(cellSize);
    {
      triHash.build(triBox);
    }
    {
      collectPairsParallel<PTPair>(nTri, 0, topology.numVerts,
        [&](int rangeBegin, int rangeEnd,
          std::vector<int> &visited,
          std::vector<int> &candidates,
          std::vector<PTPair> &localPairs) {
          for (int vi = rangeBegin; vi < rangeEnd; ++vi) {
            candidates.clear();
            triHash.queryOverlapping(vertBox[vi], triBox, -1, visited, vi + 1, candidates);

            for (int fi : candidates) {
              auto &tri = topology.triangles[fi];
              if (vi == tri[0] || vi == tri[1] || vi == tri[2])
                continue;
              localPairs.push_back({ vi, tri[0], tri[1], tri[2],
                topology.vertexArea[vi] * topology.triArea[fi] });
            }
          }
        },
        pairs.ptPairs);
    }
  }

  {
    SpatialHashGrid edgeHash(nEdge);
    edgeHash.setCellSize(cellSize);
    {
      edgeHash.build(edgeBox);
    }
    {
      collectPairsParallel<EEPair>(nEdge, 0, nEdge,
        [&](int rangeBegin, int rangeEnd,
          std::vector<int> &visited,
          std::vector<int> &candidates,
          std::vector<EEPair> &localPairs) {
          for (int ei = rangeBegin; ei < rangeEnd; ++ei) {
            candidates.clear();
            edgeHash.queryOverlappingAfter(edgeBox[ei], edgeBox, ei, visited, ei + 1, candidates);

            const int a0 = topology.edges[ei][0];
            const int a1 = topology.edges[ei][1];
            for (int ej : candidates) {
              const int b0 = topology.edges[ej][0];
              const int b1 = topology.edges[ej][1];
              if (a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1)
                continue;
              localPairs.push_back({ a0, a1, b0, b1,
                topology.edgeLength[ei] * topology.edgeLength[ej] });
            }
          }
        },
        pairs.eePairs);
    }
  }
}

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
