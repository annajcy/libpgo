#include "fields/MeshUnsignedDistanceField.h"

#include "fields/GridField.h"
#include "libiglInterface.h"

#include <cmath>

namespace pgo::ImplicitSurface {

MeshUnsignedDistanceField::MeshUnsignedDistanceField(Mesh::TriMeshGeo mesh)
  : mesh_(std::move(mesh))
{
}

void MeshUnsignedDistanceField::ensureBVH() const
{
  std::call_once(bvhFlag_, [&]() {
    bvh_ = std::make_unique<Mesh::TriMeshBVTree>();
    bvh_->buildByInertiaPartition(mesh_.ref());
  });
}

double MeshUnsignedDistanceField::eval(const V3d &p) const
{
  ensureBVH();
  const auto result = bvh_->closestTriangleQuery(mesh_.ref(), p);
  return std::sqrt(result.dist2);
}

Mesh::LightBoundingBox MeshUnsignedDistanceField::bounds() const
{
  return Mesh::LightBoundingBox(mesh_.positions());
}

GridField MeshUnsignedDistanceField::sampleToGrid(const GridSpec &spec) const
{
  EigenSupport::VXd dist;
  libiglInterface::computeDistanceField(mesh_, spec.bmin, spec.bmax, spec.resolution,
    /*robust=*/1, /*sign=*/0, dist);

  GridField grid(spec);
  for (int i = 0; i < grid.size(); ++i)
    grid[i] = dist[i];
  return grid;
}

}  // namespace pgo::ImplicitSurface
