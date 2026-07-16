#include "core/ImplicitField.h"

#include "fields/GridField.h"

#include <cstdint>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace pgo::ImplicitSurface
{

Mesh::LightBoundingBox ImplicitField::bounds() const
{
  return Mesh::LightBoundingBox();
}

void ImplicitField::evalBatch(const V3d *pts, double *out, std::size_t n) const
{
  for (std::size_t i = 0; i < n; ++i)
    out[i] = eval(pts[i]);
}

GridField ImplicitField::sampleToGrid(const GridSpec &spec) const
{
  GridField grid(spec);
  const int resolution = spec.resolution;
  const V3d delta = (spec.bmax - spec.bmin) / static_cast<double>(resolution - 1);
  const std::int64_t rowCount =
    static_cast<std::int64_t>(resolution) * static_cast<std::int64_t>(resolution);

  tbb::parallel_for(std::int64_t{ 0 }, rowCount, [&](std::int64_t row) {
    const int y = static_cast<int>(row % resolution);
    const int z = static_cast<int>(row / resolution);
    for (int x = 0; x < resolution; ++x) {
      const V3d p = spec.bmin + delta.cwiseProduct(V3d(x, y, z).cast<double>());
      grid.at(x, y, z) = eval(p);
    }
  });

  return grid;
}

}  // namespace pgo::ImplicitSurface
