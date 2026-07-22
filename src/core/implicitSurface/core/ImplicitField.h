#pragma once

#include "EigenSupport.h"
#include "boundingBox.h"
#include "field/gridSpec.h"

#include <cstddef>

namespace pgo::ImplicitSurface {

using EigenSupport::V3d;

class GridField;

class ImplicitField {
public:
  virtual double eval(const V3d &p) const = 0;

  // Invalid LightBoundingBox means unbounded.
  virtual Mesh::LightBoundingBox bounds() const;

  virtual void evalBatch(const V3d *pts, double *out, std::size_t n) const;

  // Uses the current oneTBB execution context.
  virtual GridField sampleToGrid(const GridSpec &spec) const;

  virtual ~ImplicitField() = default;
};

inline bool isUnbounded(const Mesh::LightBoundingBox &bb) { return !bb.verifyBox(); }

}  // namespace pgo::ImplicitSurface
