#pragma once

#include "core/ImplicitField.h"
#include "triMeshGeo.h"

namespace pgo::ImplicitSurface {

class SphereField : public ImplicitField {
public:
  V3d center = V3d::Zero();
  double radius = 0.0;

  SphereField() = default;
  SphereField(const V3d &center, double radius);

  double eval(const V3d &p) const override;
  Mesh::LightBoundingBox bounds() const override;

  static SphereField fromMeshBBox(const Mesh::TriMeshGeo &mesh);
  int projectOpenBoundaryToSphere(Mesh::TriMeshGeo &mesh) const;
};

}  // namespace pgo::ImplicitSurface
