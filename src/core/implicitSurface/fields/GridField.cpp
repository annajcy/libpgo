#include "fields/GridField.h"

#include <algorithm>
#include <cmath>

namespace pgo::ImplicitSurface {

GridField::GridField(const GridSpec &spec)
  : spec_(spec)
{
  validateGridSpec(spec_);

  const std::size_t total = static_cast<std::size_t>(spec_.resolution) *
                            static_cast<std::size_t>(spec_.resolution) *
                            static_cast<std::size_t>(spec_.resolution);
  values_.resize(total, 0.0);
}

double GridField::eval(const V3d &p) const
{
  const int r = spec_.resolution;
  V3d coord;
  for (int axis = 0; axis < 3; ++axis) {
    const double t = (p[axis] - spec_.bmin[axis]) / (spec_.bmax[axis] - spec_.bmin[axis]);
    coord[axis] = std::clamp(t * static_cast<double>(r - 1), 0.0, static_cast<double>(r - 1));
  }

  const int x0 = std::min(static_cast<int>(std::floor(coord[0])), r - 1);
  const int y0 = std::min(static_cast<int>(std::floor(coord[1])), r - 1);
  const int z0 = std::min(static_cast<int>(std::floor(coord[2])), r - 1);
  const int x1 = std::min(x0 + 1, r - 1);
  const int y1 = std::min(y0 + 1, r - 1);
  const int z1 = std::min(z0 + 1, r - 1);

  const double tx = coord[0] - static_cast<double>(x0);
  const double ty = coord[1] - static_cast<double>(y0);
  const double tz = coord[2] - static_cast<double>(z0);

  auto lerp = [](double a, double b, double t) { return a + (b - a) * t; };

  const double c00 = lerp(at(x0, y0, z0), at(x1, y0, z0), tx);
  const double c10 = lerp(at(x0, y1, z0), at(x1, y1, z0), tx);
  const double c01 = lerp(at(x0, y0, z1), at(x1, y0, z1), tx);
  const double c11 = lerp(at(x0, y1, z1), at(x1, y1, z1), tx);
  const double c0 = lerp(c00, c10, ty);
  const double c1 = lerp(c01, c11, ty);
  return lerp(c0, c1, tz);
}

Mesh::LightBoundingBox GridField::bounds() const
{
  return Mesh::LightBoundingBox(spec_.bmin, spec_.bmax);
}

double &GridField::at(int x, int y, int z)
{
  return values_[linearIndex(x, y, z, spec_.resolution)];
}

const double &GridField::at(int x, int y, int z) const
{
  return values_[linearIndex(x, y, z, spec_.resolution)];
}

void GridField::fill(double value)
{
  std::fill(values_.begin(), values_.end(), value);
}

void GridField::setZero()
{
  fill(0.0);
}

}  // namespace pgo::ImplicitSurface
