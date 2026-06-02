#pragma once

#include "core/ImplicitField.h"
#include "boundingVolumeTree.h"
#include "triMeshGeo.h"

#include <memory>
#include <mutex>

namespace pgo::ImplicitSurface {

class MeshUnsignedDistanceField : public ImplicitField {
public:
  explicit MeshUnsignedDistanceField(Mesh::TriMeshGeo mesh);

  double eval(const V3d &p) const override;
  Mesh::LightBoundingBox bounds() const override;
  GridField sampleToGrid(const GridSpec &spec, int numThreads = 0) const override;

private:
  Mesh::TriMeshGeo mesh_;
  mutable std::unique_ptr<Mesh::TriMeshBVTree> bvh_;
  mutable std::once_flag bvhFlag_;

  void ensureBVH() const;
};

}  // namespace pgo::ImplicitSurface
