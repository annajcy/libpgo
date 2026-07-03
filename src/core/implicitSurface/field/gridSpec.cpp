#include "field/gridSpec.h"

#include <cmath>
#include <stdexcept>

namespace pgo::ImplicitSurface {

void validateGridSpec(const GridSpec &spec)
{
  if (spec.resolution < 2)
    throw std::runtime_error("GridSpec resolution must be at least 2");

  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(spec.bmin[axis]) || !std::isfinite(spec.bmax[axis]))
      throw std::runtime_error("GridSpec bounds must be finite");
    if (spec.bmax[axis] <= spec.bmin[axis])
      throw std::runtime_error("GridSpec: bmax[" + std::to_string(axis) + "] must be greater than bmin[" + std::to_string(axis) + "]");
  }
}

bool GridSpec::operator==(const GridSpec &other) const
{
  return resolution == other.resolution &&
         bmin == other.bmin &&
         bmax == other.bmax;
}

int linearIndex(int x, int y, int z, int resolution)
{
  return z * resolution * resolution + y * resolution + x;
}

}  // namespace pgo::ImplicitSurface
