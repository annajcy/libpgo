#pragma once

#include "core/ImplicitField.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pgo::ImplicitSurface {

class BoxField : public ImplicitField {
public:
  V3d center = V3d::Zero();
  V3d halfExtent = V3d::Zero();

  BoxField(const V3d &center_, const V3d &halfExtent_)
    : center(center_), halfExtent(halfExtent_)
  {
    if ((halfExtent.array() <= 0.0).any())
      throw std::runtime_error("BoxField half_extent entries must be positive");
  }

  explicit BoxField(const Mesh::LightBoundingBox &bb)
    : center(bb.center()), halfExtent(bb.halfSides())
  {
    if (!bb.verifyBox())
      throw std::runtime_error("BoxField requires a valid bounding box");
  }

  double eval(const V3d &p) const override
  {
    const V3d q = (p - center).cwiseAbs() - halfExtent;
    return std::max({ q[0], q[1], q[2] });
  }

  Mesh::LightBoundingBox bounds() const override
  {
    return Mesh::LightBoundingBox(center - halfExtent, center + halfExtent);
  }
};

}  // namespace pgo::ImplicitSurface
