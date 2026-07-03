/*
author: Bohan Wang
copyright to MIT, USC
*/

#pragma once

#include "EigenDef.h"
#include "triMeshGeo.h"

namespace pgo
{
namespace CGALInterface
{
struct MergeCloseVerticesResult
{
  Mesh::TriMeshGeo mesh;
  int mergedVertices = 0;
  double eps = 0.0;
};

struct RawSurfaceCleanupStats
{
  int vertices = 0;
  int triangles = 0;
  int invalidTriangles = 0;
  int components = 0;
  int boundaryOrNonmanifoldEdges = 0;
  bool isManifold = false;
};

struct RawSurfaceCleanupOptions
{
  int expectedComponents = -1;
  double shortEdgeThreshold = 1e-5;
  int maxPasses = 3;
  int maxCollapses = 10000;
};

struct RawSurfaceCleanupReport
{
  int expectedComponents = -1;
  double shortEdgeThreshold = 0.0;
  int maxPasses = 0;
  int maxCollapses = 0;

  RawSurfaceCleanupStats before;
  RawSurfaceCleanupStats after;
  bool topologyPreserved = false;
  bool cleanupComplete = false;

  int attemptedDeletions = 0;
  int acceptedDeletions = 0;
  int attemptedCollapses = 0;
  int acceptedCollapses = 0;
  int rejectedByTopology = 0;
  int rejectedByInvalidCount = 0;
};

struct RawSurfaceCleanupResult
{
  Mesh::TriMeshGeo mesh;
  RawSurfaceCleanupReport report;
};

Mesh::TriMeshGeo isotropicRemeshing(const Mesh::TriMeshGeo &mesh, double targetEdgeLength, int numIter, double angleInDegree,
  std::vector<int> *constrainedVertices = nullptr, std::vector<std::pair<int, int>> *constrainedEdges = nullptr,
  std::vector<int> *faceSubset = nullptr);
Mesh::TriMeshGeo smoothMesh(const Mesh::TriMeshGeo &mesh, int numIter, double angleInDegree,
  std::vector<int> *constrainedVertices = nullptr, std::vector<std::pair<int, int>> *constrainedEdges = nullptr);
Mesh::TriMeshGeo smoothShape(const Mesh::TriMeshGeo &mesh, double time, int numIter);

Mesh::TriMeshGeo simplifyMesh(const Mesh::TriMeshGeo &meshIn, double edgeStoppingRatio);
Mesh::TriMeshGeo simplifyMeshGH(const Mesh::TriMeshGeo &meshIn, const std::string &method, double edgeStoppingRatio);

Mesh::TriMeshGeo refineMesh(const Mesh::TriMeshGeo &mesh, double density);
Mesh::TriMeshGeo refineSharpRegionOnMesh(const Mesh::TriMeshGeo &mesh, double density, double angle);
Mesh::TriMeshGeo subdivideMesh(const Mesh::TriMeshGeo &meshIn, int nIter, double smallSize = -1);
bool corefineAndComputeUnion(const Mesh::TriMeshGeo &mesh1, const Mesh::TriMeshGeo &mesh2, Mesh::TriMeshGeo &unionMesh);
bool corefineAndComputeIntersection(const Mesh::TriMeshGeo &mesh1, const Mesh::TriMeshGeo &mesh2, Mesh::TriMeshGeo &intersectionMesh);
void corefineOnly(Mesh::TriMeshGeo &mesh1, Mesh::TriMeshGeo &mesh2, bool noModify1 = false, bool noModify2 = false, double edgeLengthThreshold = 1e-5);

Mesh::TriMeshGeo triangulateHolePolyline(const std::vector<Vec3d> &polyline);
Mesh::TriMeshGeo triangulateRefineFairHole(const Mesh::TriMeshGeo &mesh);
Mesh::TriMeshGeo triangulate(const std::vector<Vec3d> &vtx, const std::vector<std::vector<int>> &faces);

Mesh::TriMeshGeo clipMesh(const Mesh::TriMeshGeo &meshIn, const Vec3d &planeN, const Vec3d &planeP);
Mesh::TriMeshGeo clipMesh(const Mesh::TriMeshGeo &meshIn, const Mesh::TriMeshGeo &volMesh);

void segmentMesh(const Mesh::TriMeshGeo &meshIn, int nClusters, std::vector<int> &classID);

void convexHullMesh(const std::vector<Mesh::TriMeshGeo> &meshes, Mesh::TriMeshGeo &meshOut);

bool isSelfIntersected(const Mesh::TriMeshGeo &meshIn);
Mesh::TriMeshGeo repairSelfIntersections(const Mesh::TriMeshGeo &meshIn, const std::string &method = "autorefine", bool *allFixed = nullptr);
MergeCloseVerticesResult mergeCloseVertices(const Mesh::TriMeshGeo &meshIn, double eps = -1.0);
RawSurfaceCleanupResult rawSurfaceCleanup(const Mesh::TriMeshGeo &meshIn, const RawSurfaceCleanupOptions &options = RawSurfaceCleanupOptions());
bool isManifold(const Mesh::TriMeshGeo &meshIn);
void getLargestCC(const Mesh::TriMeshGeo &meshIn, Mesh::TriMeshGeo &meshOut);

}  // namespace CGALInterface
}  // namespace pgo
