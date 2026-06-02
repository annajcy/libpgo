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

  // numThreads:
  //   0: use OpenMP defaults
  //   1: force a serial loop
  //  >1: request that many OpenMP threads
  virtual GridField sampleToGrid(const GridSpec &spec, int numThreads = 0) const;

  virtual ~ImplicitField() = default;
};

inline bool isUnbounded(const Mesh::LightBoundingBox &bb) { return !bb.verifyBox(); }

}  // namespace pgo::ImplicitSurface
