#include "obstaclePoseCache.h"
#include "parallelism/parallelFor.h"

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

#include <algorithm>
#include <functional>

namespace pgo
{
namespace Contact
{
namespace IPC
{

void buildObstaclePoseCache(
  const EigenSupport::VXd &positions,
  const EigenSupport::MXi &triangles,
  const EigenSupport::MXi &contactEdges,
  ObstaclePoseCache &cache)
{
  const int nVerts = static_cast<int>(positions.size() / 3);
  const int nTri = static_cast<int>(triangles.rows());
  const int nEdge = static_cast<int>(contactEdges.rows());

  cache.triAreas.assign(nTri, 0.0);
  cache.edgeLengths.assign(nEdge, 0.0);
  cache.vertBoxes.resize(nVerts);
  cache.triBoxes.resize(nTri);
  cache.edgeBoxes.resize(nEdge);

  cache.hasSurfaceBox = nVerts > 0;
  if (cache.hasSurfaceBox) {
    cache.surfaceBox.init(positions.segment<3>(0), 0.0);
    for (int vi = 1; vi < nVerts; ++vi)
      cache.surfaceBox.expand(positions.segment<3>(3 * vi));
  }
  else {
    cache.surfaceBox.lo.setZero();
    cache.surfaceBox.hi.setZero();
  }

  // Per-vertex degenerate point AABBs.
  pgo::parallel::parallelFor(0, nVerts,
    pgo::parallel::Options{ .nestedKernelPolicy = pgo::parallel::NestedKernelPolicy::Inherit },
    [&](int vi) {
      cache.vertBoxes[vi].init(positions.segment<3>(3 * vi), 0.0);
    });

  // Triangle AABBs + cached areas.
  pgo::parallel::parallelFor(0, nTri,
    pgo::parallel::Options{ .nestedKernelPolicy = pgo::parallel::NestedKernelPolicy::Inherit },
    [&](int fi) {
      const EigenSupport::V3d v0 = positions.segment<3>(3 * triangles(fi, 0));
      const EigenSupport::V3d v1 = positions.segment<3>(3 * triangles(fi, 1));
      const EigenSupport::V3d v2 = positions.segment<3>(3 * triangles(fi, 2));
      cache.triAreas[fi] = 0.5 * (v1 - v0).cross(v2 - v0).norm();
      cache.triBoxes[fi].init(v0, 0.0);
      cache.triBoxes[fi].expand(v1);
      cache.triBoxes[fi].expand(v2);
    });

  // Edge AABBs + cached lengths.
  pgo::parallel::parallelFor(0, nEdge,
    pgo::parallel::Options{ .nestedKernelPolicy = pgo::parallel::NestedKernelPolicy::Inherit },
    [&](int ei) {
      const EigenSupport::V3d e0 = positions.segment<3>(3 * contactEdges(ei, 0));
      const EigenSupport::V3d e1 = positions.segment<3>(3 * contactEdges(ei, 1));
      cache.edgeLengths[ei] = (e1 - e0).norm();
      cache.edgeBoxes[ei].init(e0, 0.0);
      cache.edgeBoxes[ei].expand(e1);
    });

  // Average tri AABB diagonal → spatial hash cell size.
  const double diagSum = tbb::parallel_reduce(
    tbb::blocked_range<int>(0, nTri), 0.0,
    [&](const tbb::blocked_range<int> &r, double sum) {
      for (int fi = r.begin(); fi < r.end(); ++fi)
        sum += (cache.triBoxes[fi].hi - cache.triBoxes[fi].lo).norm();
      return sum;
    },
    std::plus<double>());
  cache.cellSize = nTri > 0 ? std::max(diagSum / nTri, 1e-6) : 1e-6;

  // Rebuild spatial hashes. clear() retains bucket capacity so steady-state
  // refresh is rehash-free. Inserts are sequential (SpatialHashGrid is not
  // thread-safe).
  cache.triHash.setCellSize(cache.cellSize);
  cache.triHash.build(cache.triBoxes);

  cache.edgeHash.setCellSize(cache.cellSize);
  cache.edgeHash.build(cache.edgeBoxes);
}

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
